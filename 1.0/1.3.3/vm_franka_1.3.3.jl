# =======================================================================================
# VIRTUAL MODEL CONTROL FOR A FRANKA ARM WITH WHOLE-BODY OBSTACLE AVOIDANCE
#
# This script demonstrates an advanced Virtual Model Control (VMC) strategy for the 
# Franka Emika FR3 robot. The key tasks are:
# 1. To have the robot's end-effector follow a "virtual slider" moving along a "virtual bar".
# 2. To have both the virtual bar and the robot's body actively avoid multiple obstacles.
# 3. To dynamically decide the starting point of the slider based on which end of the
#    virtual bar is initially further away from the obstacles.
#
# The control architecture is built using the VMRobotControl.jl package.
# =======================================================================================

using DifferentialEquations
using GeometryBasics: Vec3f, Point3f
using GLMakie
using LinearAlgebra
using MeshIO
using StaticArrays
using VMRobotControl

# The Franka robot's mesh files are in the .dae (Collada) format.
# FileIO.jl needs to be explicitly told how to handle this format. This is a common
# setup step when working with URDFs that use .dae meshes.
using FileIO, UUIDs
try
    FileIO.add_format(format"DAE", (), ".dae", [:DigitalAssetExchangeFormatIO => UUID("43182933-f65b-495a-9e05-43182933-f65b-495a-9e05-4d939cea427d")])
catch
    # This might fail if the format is already added, so we wrap it in a try-catch block.
end


# =======================================================================================
# --- 1. ROBOT MODEL CONFIGURATION ---
# =======================================================================================
println("1. Setting up the robot model...")

# Suppress warnings from the URDF parser about unsupported tags (e.g., <transmission>)
# that are not necessary for our dynamic simulation.
cfg = URDFParserConfig(;suppress_warnings=true)

# Construct the full path to the robot's URDF file.
module_path = joinpath(splitpath(splitdir(pathof(VMRobotControl))[1])[1:end-1])
robot = parseURDF(joinpath(module_path, "URDFs/franka_description/urdfs/fr3.urdf"), cfg)

# Add a gravity compensator component to the robot. This is crucial for accurate
# dynamics, as it applies joint torques to counteract the force of gravity on the robot's links.
add_gravity_compensation!(robot, VMRobotControl.DEFAULT_GRAVITY)

# Add coordinates for each of the 7 robot joints and apply a small amount of linear
# damping. This improves simulation stability and mimics real-world joint friction.
for i in 1:7
    # `JointSubspace` creates a coordinate that directly represents the state of a joint.
    add_coordinate!(robot, JointSubspace("fr3_joint$i"); id="J$i")
    # `LinearDamper` creates a dissipative force proportional to the joint's velocity.
    add_component!(robot, LinearDamper(0.1, "J$i");      id="JointDamper$i")
end


# =======================================================================================
# --- 2. VIRTUAL MECHANISM DEFINITION (THE ROTATING BAR & SLIDER) ---
# =======================================================================================
println("2. Creating the virtual bar with a mobile slider...")

# This virtual mechanism is a dynamic entity that exists only in software.
# The robot will be controlled to interact with this virtual object.
bar_length = 1.0
bar_inertia_val = (1/12) * 2.0 * bar_length^2 # Inertia for a slender rod.
bar_inertia_tensor = SMatrix{3,3}(bar_inertia_val, 0, 0, 0, 1e-3, 0, 0, 0, bar_inertia_val)

pivot_point = SVector(0.5, 0.3, 0.4) # The world position where the virtual bar is anchored.
vm = Mechanism{Float64}("VirtualBarAndSlider")

# We build the virtual mechanism's kinematics by adding frames and connecting them with joints.
# This kinematic chain allows the bar to pivot freely in 3D (yaw, pitch, roll).
pivot_frame = add_frame!(vm, "pivot_frame")
frame_rz = add_frame!(vm, "frame_rz")
frame_ry = add_frame!(vm, "frame_ry")
bar_body_frame = add_frame!(vm, "bar_body_frame") # This frame is rigidly attached to the bar.

# Anchor the entire mechanism in the world.
add_joint!(vm, Rigid(Transform(pivot_point)); parent=root_frame(vm), child=pivot_frame, id="PivotAnchor")
# Create a sequence of Revolute joints to give the bar 3 rotational degrees of freedom.It can be oriented arbitrarily in 3D space.
# Svector is expressed in the parent frame's coordinates.
add_joint!(vm, Revolute(SVector(0.,0.,1.)); parent=pivot_frame,    child=frame_rz, id="J_yaw")
add_joint!(vm, Revolute(SVector(0.,1.,0.)); parent=frame_rz,       child=frame_ry, id="J_pitch")
add_joint!(vm, Revolute(SVector(1.,0.,0.)); parent=frame_ry,       child=bar_body_frame, id="J_roll")

# Define the slider. It's a `Prismatic` joint allowing linear motion along the bar's local Y-axis.
# The "attraction_point_frame" represents the slider's position. The robot's end-effector
# will be "attracted" to this frame.
# We choofse the bar's local Y-axis as the slider direction.
attraction_point_frame = add_frame!(vm, "attraction_point")
add_joint!(vm, Prismatic(SVector(0.0, 1.0, 0.0)); parent=bar_body_frame, child=attraction_point_frame, id="J_slider")

# We define specific frames on the bar to be used for applying repulsive forces from obstacles.
# These act as "control points".
upper_attach_frame = add_frame!(vm, "upper_attach")
lower_attach_frame = add_frame!(vm, "lower_attach")
add_joint!(vm, Rigid(Transform(SVector(0.0,  0.25 * bar_length, 0.0))); parent=bar_body_frame, child=upper_attach_frame, id="UpperAttachment")
add_joint!(vm, Rigid(Transform(SVector(0.0, -0.25 * bar_length, 0.0))); parent=bar_body_frame, child=lower_attach_frame, id="LowerAttachment")

# Separate frames are added purely for visualization to draw the full length of the bar.
bar_end_upper_frame = add_frame!(vm, "bar_end_upper")
bar_end_lower_frame = add_frame!(vm, "bar_end_lower")
add_joint!(vm, Rigid(Transform(SVector(0.0,  0.5 * bar_length, 0.0))); parent=bar_body_frame, child=bar_end_upper_frame, id="BarEndUpperAttachment")
add_joint!(vm, Rigid(Transform(SVector(0.0, -0.5 * bar_length, 0.0))); parent=bar_body_frame, child=bar_end_lower_frame, id="BarEndLowerAttachment")

# Now we add the dynamic properties to the virtual mechanism to make it behave physically.
#FrameAngularVelocity(frameID) è un tipo speciale di coordinata operativa che rappresenta il vettore della velocità angolare $\boldsymbol{\omega}$ del telaio specificato (frameID) misurato nel sistema di riferimento globale (World Frame).
#it is necessary because to compute forces and torques acting on the virtual mechanism, we need to know how fast it is rotating.
add_coordinate!(vm, FrameAngularVelocity(bar_body_frame); id="BarAngularVelocity")
add_component!(vm, Inertia(bar_inertia_tensor, "BarAngularVelocity"); id="BarInertia")
add_component!(vm, LinearDamper(0.5 * I(3), "BarAngularVelocity"); id="BarRotationalDamper") # Rotational damping.
add_component!(vm, LinearInerter(0.1, "AttractionPoint"); id="SliderInertance") # Mass for the slider.


# =======================================================================================
# --- 3. VIRTUAL CONTROL SYSTEM ASSEMBLY ---
# =======================================================================================
println("3. Setting up the VMS and control components...")
# The VirtualMechanismSystem (VMS) links the robot and the virtual mechanism,
# allowing us to define control components (springs, dampers) that act between them.
vms = VirtualMechanismSystem("SlidingControl", robot, vm)

# === VIRTUAL BAR OBSTACLE AVOIDANCE ===
# We create repulsive forces between the virtual bar's attachment points and the obstacles.
obstacle_position_1 = SVector(0.4, 0.2, 0.5)
obstacle_position_2 = SVector(0.4, -0.1, 0.4)
#obstacles are static, so constcoord and not referencecoord
add_coordinate!(vm, ConstCoord(obstacle_position_1); id="ObstaclePosition1")
add_coordinate!(vm, ConstCoord(obstacle_position_2); id="ObstaclePosition2")

# Define coordinates representing the world positions of the attachment points.
add_coordinate!(vm, FrameOrigin(upper_attach_frame); id="UpperAttachPoint")
add_coordinate!(vm, FrameOrigin(lower_attach_frame); id="LowerAttachPoint")
add_coordinate!(vm, FrameOrigin(bar_end_upper_frame); id="BarEndUpperPoint")
add_coordinate!(vm, FrameOrigin(bar_end_lower_frame); id="BarEndLowerPoint")

# For each obstacle, create repulsive springs acting on the bar's attachment points.
# A `GaussianSpring` with a negative stiffness creates a repulsive force field.
# The `width` parameter controls how localized this force is.
add_coordinate!(vms, CoordDifference(".virtual_mechanism.UpperAttachPoint", ".virtual_mechanism.ObstaclePosition1"); id="UpperError1")
add_coordinate!(vms, CoordDifference(".virtual_mechanism.LowerAttachPoint", ".virtual_mechanism.ObstaclePosition1"); id="LowerError1")
add_component!(vms, GaussianSpring("UpperError1"; stiffness=-150.0, width=0.1); id="UpperObstacleSpring1")
add_component!(vms, LinearDamper(20.0, "UpperError1"); id="UpperObstacleDamper1")
add_component!(vms, GaussianSpring("LowerError1"; stiffness=-150.0, width=0.1); id="LowerObstacleSpring1")
add_component!(vms, LinearDamper(20.0, "LowerError1"); id="LowerObstacleDamper1")

add_coordinate!(vms, CoordDifference(".virtual_mechanism.UpperAttachPoint", ".virtual_mechanism.ObstaclePosition2"); id="UpperError2")
add_coordinate!(vms, CoordDifference(".virtual_mechanism.LowerAttachPoint", ".virtual_mechanism.ObstaclePosition2"); id="LowerError2")
add_component!(vms, GaussianSpring("UpperError2"; stiffness=-150.0, width=0.1); id="UpperObstacleSpring2")
add_component!(vms, LinearDamper(20.0, "UpperError2"); id="UpperObstacleDamper2")
add_component!(vms, GaussianSpring("LowerError2"; stiffness=-150.0, width=0.1); id="LowerObstacleSpring2")
add_component!(vms, LinearDamper(20.0, "LowerError2"); id="LowerObstacleDamper2")

# === ROBOT-TO-VIRTUAL-MECHANISM COUPLING ===
# This is the core of the control. We connect the robot's TCP to the virtual slider
# with a strong spring-damper system. This forces the robot to follow the slider.
add_coordinate!(robot, FramePoint("fr3_link8", SVector(0.0, 0.0, 0.0)); id="TCP") #defined in this way because it is more flexilbe than FrameOrigin
add_coordinate!(vm, FrameOrigin(attraction_point_frame); id="AttractionPoint")
add_coordinate!(vms, CoordDifference(".virtual_mechanism.AttractionPoint", ".robot.TCP"); id="PathError")

K_path = SMatrix{3, 3}(250.0, 0, 0, 0, 250.0, 0, 0, 0, 250.0) # High stiffness for tight tracking.
D_path = SMatrix{3, 3}(100.0, 0, 0, 0, 100.0, 0, 0, 0, 100.0) # Damping for stability.
add_component!(vms, LinearSpring(K_path, "PathError"); id="PathSpring")
add_component!(vms, LinearDamper(D_path, "PathError"); id="PathDamper")

# === ROBOT WHOLE-BODY OBSTACLE AVOIDANCE ===
# In addition to the virtual bar avoiding obstacles, we add another layer of safety
# by making the robot's links themselves repulsive to the obstacles.
links_to_protect = ["fr3_link4","fr3_link5", "fr3_link6", "fr3_link7", "fr3_link8"]
obstacles = [
    (id_num=1, coord_id=".virtual_mechanism.ObstaclePosition1"),
    (id_num=2, coord_id=".virtual_mechanism.ObstaclePosition2")
]
for link_name in links_to_protect
    link_origin_coord_id = "$(link_name)_origin"
    add_coordinate!(robot, FrameOrigin(link_name); id=link_origin_coord_id)
    for obstacle in obstacles
        error_coord_id = "$(link_name)_ObstacleError$(obstacle.id_num)"
        add_coordinate!(vms, CoordDifference(".robot.$(link_origin_coord_id)", obstacle.coord_id); id=error_coord_id)
        spring_id = "$(link_name)_ObstacleSpring$(obstacle.id_num)"
        add_component!(vms, GaussianSpring(error_coord_id; stiffness=-80.0, width=0.1); id=spring_id)
    end
end


# =======================================================================================
# --- 4. ADAPTIVE LOGIC: DYNAMIC STARTING POINT SELECTION ---
# =======================================================================================
println("4. Dynamically choosing the slider's starting point on the bar...")

# This is an elegant piece of adaptive logic. Before the simulation starts, we check
# which end of the bar is further away from the nearest obstacle and choose that as
# the starting point for the slider.

obstacle_positions = [obstacle_position_1, obstacle_position_2]
# In world frame
top_start_pos_local = SVector(0.0, 0.5 * bar_length, 0.0)
bottom_start_pos_local = SVector(0.0, -0.5 * bar_length, 0.0)

# The bar is initially oriented along the world Y-axis at the pivot point.
top_start_pos_world = pivot_point + top_start_pos_local
bottom_start_pos_world = pivot_point + bottom_start_pos_local

# Calculate the distance from each bar end to each obstacle.
dist_top_to_obstacles = [norm(top_start_pos_world - obs) for obs in obstacle_positions]
dist_bottom_to_obstacles = [norm(bottom_start_pos_world - obs) for obs in obstacle_positions]

# The "safety score" is the distance to the *closest* obstacle.
safety_score_top = minimum(dist_top_to_obstacles)
safety_score_bottom = minimum(dist_bottom_to_obstacles)

local slider_initial_pos::Float64
local start_from_bottom::Bool

if safety_score_bottom >= safety_score_top
    println("Decision: Starting from the bottom. It is further from obstacles.")
    start_from_bottom = true
    slider_initial_pos = -0.5 * bar_length
else
    println("Decision: Starting from the top. It is further from obstacles.")
    start_from_bottom = false
    slider_initial_pos = 0.5 * bar_length
end

# --- Dynamic Configuration of Virtual Control Components ---

# 1. SLIDER CONTROL
# A virtual spring-damper system is set up to drive the slider from its starting
# position towards the center of the bar (the equilibrium point).
add_coordinate!(vm, JointSubspace("J_slider"); id="SliderCoord")
add_coordinate!(vm, ConstCoord(SVector(0.0)); id="SliderTarget") # Target is the center of the bar.
add_coordinate!(vms, CoordDifference(".virtual_mechanism.SliderCoord", ".virtual_mechanism.SliderTarget"); id="SliderDrivingError")
K_drive = 100.0
D_drive = 50.0
add_component!(vms, LinearSpring(K_drive, "SliderDrivingError"); id="SliderDrivingSpring")
add_component!(vms, LinearDamper(D_drive, "SliderDrivingError"); id="SliderDrivingDamper")

# 2. ORIENTATION CONTROL (Adapts to the starting direction)
# We want the robot's tool (Z-axis) to align with the direction of the slider's motion.
add_coordinate!(vm, FrameOrigin(bar_body_frame); id="BarOrigin")
add_coordinate!(vm, FramePoint(bar_body_frame, SVector(0.0, -1.0, 0.0)); id="BarAxisPoint") # Vector along bar's local -Y
add_coordinate!(vm, CoordDifference("BarAxisPoint", "BarOrigin"); id="BarAxisVector")
add_coordinate!(vm, FramePoint(bar_body_frame, SVector(0.0, 1.0, 0.0)); id="InvertedBarAxisPoint") # Vector along bar's local +Y
add_coordinate!(vm, CoordDifference("InvertedBarAxisPoint", "BarOrigin"); id="InvertedBarAxisVector")

add_coordinate!(robot, FrameOrigin("fr3_link8"); id="ToolOrigin")
add_coordinate!(robot, FramePoint("fr3_link8", SVector(0.0, 0.0, 1.0)); id="ToolAxisPoint") # Tool's Z-axis
add_coordinate!(robot, CoordDifference("ToolAxisPoint", "ToolOrigin"); id="ToolAxisVector")

# Based on the starting position, we select the correct target vector for alignment.
# If starting from the bottom, the tool should point towards +Y (InvertedBarAxisVector) to move to the center.
# If starting from the top, the tool should point towards -Y (BarAxisVector) to move to the center.
orientation_target_vector_id = start_from_bottom ? ".virtual_mechanism.InvertedBarAxisVector" : ".virtual_mechanism.BarAxisVector"
println("Orientation target vector (corrected): $orientation_target_vector_id")

add_coordinate!(vms, CoordDifference(orientation_target_vector_id, ".robot.ToolAxisVector"); id="OrientationError")
K_rot = 100.0
D_rot = 50.0
add_component!(vms, LinearSpring(K_rot, "OrientationError"); id="OrientationSpring")
add_component!(vms, LinearDamper(D_rot, "OrientationError"); id="OrientationDamper")


# =======================================================================================
# --- 5. SIMULATION ---
# =======================================================================================
println("5. Starting the simulation...")
tspan = (0.0, 15.0)

# Set initial configurations for the virtual mechanism and the robot.
# The slider's initial position is determined by our adaptive logic.
q_vm_initial = [0.0, 0.0, 0.0, slider_initial_pos] # [yaw, pitch, roll, slider_pos]
q_initial = ([0.0, -π/4, 0.0, -3*π/4, 0.0, π/2, π/4], q_vm_initial)
q̇_initial = (zeros(7), zeros(4))
g = VMRobotControl.DEFAULT_GRAVITY

# Compile the VMS into an efficient, immutable structure for computation.
compiled_vms = compile(vms)
# Create a cache to store intermediate results of the dynamics calculations.
dcache = new_dynamics_cache(compiled_vms)
# Set up the Ordinary Differential Equation (ODE) problem.
prob = get_ode_problem(dcache, g, q_initial, q̇_initial, tspan)
# Solve the ODE problem using the Tsit5 solver, a standard choice for non-stiff problems.
sol = solve(prob, Tsit5(); maxiters=3e5, abstol=1e-6, reltol=1e-6)


# =======================================================================================
# --- 6. VISUALIZATION ---
# =======================================================================================
println("6. Generating the animation...")
fig = Figure(size = (800, 800))
display(fig)
ls = LScene(fig[1, 1]; show_axis=true)
cam = cam3d!(ls, camera=:perspective, center=false)
cam.lookat[] = [0.4, 0.1, 0.5]
cam.eyeposition[] = [1.7, 0.8, 1.0]

# An `Observable` kinematics cache is used for plotting. When this cache is updated,
# all visual elements that depend on it will automatically update their positions.
plotting_kcache = Observable(new_kinematics_cache(compiled_vms))
robotvisualize!(ls, plotting_kcache)
mesh!(ls, Sphere(Point3f(pivot_point), 0.03), color=(:green, 0.9), label="Pivot")
mesh!(ls, Sphere(Point3f(obstacle_position_1), 0.04), color=(:orange, 0.9), label="Obstacle 1")
mesh!(ls, Sphere(Point3f(obstacle_position_2), 0.04), color=(:magenta, 0.9), label="Obstacle 2")

# Get compiled IDs for the coordinates we want to visualize.
obs1_id = get_compiled_coordID(compiled_vms, ".virtual_mechanism.ObstaclePosition1")
obs2_id = get_compiled_coordID(compiled_vms, ".virtual_mechanism.ObstaclePosition2")
upper_attach_id = get_compiled_coordID(compiled_vms, ".virtual_mechanism.UpperAttachPoint")
lower_attach_id = get_compiled_coordID(compiled_vms, ".virtual_mechanism.LowerAttachPoint")
attraction_id = get_compiled_coordID(compiled_vms, ".virtual_mechanism.AttractionPoint")
bar_end_upper_id = get_compiled_coordID(compiled_vms, ".virtual_mechanism.BarEndUpperPoint")
bar_end_lower_id = get_compiled_coordID(compiled_vms, ".virtual_mechanism.BarEndLowerPoint")

# Create `Observable` points by mapping the configuration of each coordinate from the plotting cache.
obs1_pos = map(c -> Point3f(configuration(c, obs1_id)), plotting_kcache)
obs2_pos = map(c -> Point3f(configuration(c, obs2_id)), plotting_kcache)
upper_attach_pos = map(c -> Point3f(configuration(c, upper_attach_id)), plotting_kcache)
lower_attach_pos = map(c -> Point3f(configuration(c, lower_attach_id)), plotting_kcache)
attraction_pos = map(c -> Point3f(configuration(c, attraction_id)), plotting_kcache)
bar_end_upper_pos = map(c -> Point3f(configuration(c, bar_end_upper_id)), plotting_kcache)
bar_end_lower_pos = map(c -> Point3f(configuration(c, bar_end_lower_id)), plotting_kcache)

# Visualize the virtual components.
# Draw dashed lines to represent the repulsive virtual springs.
lines!(ls, map((o, u) -> [o, u], obs1_pos, upper_attach_pos), color=:purple, linewidth=2, linestyle=:dash)
lines!(ls, map((o, l) -> [o, l], obs1_pos, lower_attach_pos), color=:purple, linewidth=2, linestyle=:dash)
lines!(ls, map((o, u) -> [o, u], obs2_pos, upper_attach_pos), color=:cyan, linewidth=2, linestyle=:dash)
lines!(ls, map((o, l) -> [o, l], obs2_pos, lower_attach_pos), color=:cyan, linewidth=2, linestyle=:dash)

# Draw the virtual bar itself.
lines!(ls, map((u, l) -> [u, l], bar_end_upper_pos, bar_end_lower_pos), color=:red, linewidth=5)

# Draw the attraction point (the slider).
scatter!(ls, attraction_pos, color=:blue, markersize=20, label="Attraction Point")

# Use the helper function to generate and save the animation from the ODE solution.
savepath = joinpath(@__DIR__, "franka_whole_body_avoidance_final.mp4")
animate_robot_odesolution(fig, sol, plotting_kcache, savepath)
@info "Animation saved to: $savepath"