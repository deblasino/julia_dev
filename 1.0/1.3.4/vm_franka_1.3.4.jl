using DifferentialEquations
using GeometryBasics: Vec3f, Point3f
using GLMakie
using LinearAlgebra
using MeshIO
using StaticArrays
using VMRobotControl

# Necessario per caricare i mesh del robot Franka in formato .dae
using FileIO, UUIDs
try
    FileIO.add_format(format"DAE", (), ".dae", [:DigitalAssetExchangeFormatIO => UUID("43182933-f65b-495a-9e05-4d939cea427d")])
catch
end

# --- 1. SETUP DEL ROBOT (Invariato) ---
println("1. Setup del robot...")
cfg = URDFParserConfig(;suppress_warnings=true)
module_path = joinpath(splitpath(splitdir(pathof(VMRobotControl))[1])[1:end-1])
robot = parseURDF(joinpath(module_path, "URDFs/franka_description/urdfs/fr3.urdf"), cfg)

add_gravity_compensation!(robot, VMRobotControl.DEFAULT_GRAVITY)
for i in 1:7
    add_coordinate!(robot, JointSubspace("fr3_joint$i"); id="J$i")
    add_component!(robot, LinearDamper(0.1, "J$i");      id="Joint damper $i")
end

# --- 2. DEFINIZIONE DEL MECCANISMO VIRTUALE (Sbarra Rotante + Cursore) (Invariato) ---
println("2. Creazione della sbarra virtuale con cursore mobile...")
bar_length = 1.0
bar_inertia_val = (1/12) * 2.0 * bar_length^2
bar_inertia_tensor = SMatrix{3,3}(bar_inertia_val, 0, 0, 0, 1e-3, 0, 0, 0, bar_inertia_val)

pivot_point = SVector(0.5, 0.3, 0.4)
vm = Mechanism{Float64}("VirtualBarAndSlider")

pivot_frame = add_frame!(vm, "pivot_frame")
frame_rz = add_frame!(vm, "frame_rz")
frame_ry = add_frame!(vm, "frame_ry")
bar_body_frame = add_frame!(vm, "bar_body_frame")
add_joint!(vm, Rigid(Transform(pivot_point)); parent=root_frame(vm), child=pivot_frame, id="PivotAnchor")
add_joint!(vm, Revolute(SVector(0.,0.,1.)); parent=pivot_frame,    child=frame_rz, id="J_yaw")
add_joint!(vm, Revolute(SVector(0.,1.,0.)); parent=frame_rz,       child=frame_ry, id="J_pitch")
add_joint!(vm, Revolute(SVector(1.,0.,0.)); parent=frame_ry,       child=bar_body_frame, id="J_roll")
attraction_point_frame = add_frame!(vm, "attraction_point")
add_joint!(vm, Prismatic(SVector(0.0, 1.0, 0.0)); parent=bar_body_frame, child=attraction_point_frame, id="J_slider")

# Frame per i PUNTI DI ATTACCO delle molle (logica di controllo)
upper_attach_frame = add_frame!(vm, "upper_attach")
lower_attach_frame = add_frame!(vm, "lower_attach")
add_joint!(vm, Rigid(Transform(SVector(0.0,  0.25 * bar_length, 0.0))); parent=bar_body_frame, child=upper_attach_frame, id="UpperAttachment")
add_joint!(vm, Rigid(Transform(SVector(0.0, -0.25 * bar_length, 0.0))); parent=bar_body_frame, child=lower_attach_frame, id="LowerAttachment")

# Frame separati per la VISUALIZZAZIONE degli estremi della sbarra
bar_end_upper_frame = add_frame!(vm, "bar_end_upper")
bar_end_lower_frame = add_frame!(vm, "bar_end_lower")
add_joint!(vm, Rigid(Transform(SVector(0.0,  0.5 * bar_length, 0.0))); parent=bar_body_frame, child=bar_end_upper_frame, id="BarEndUpperAttachment")
add_joint!(vm, Rigid(Transform(SVector(0.0, -0.5 * bar_length, 0.0))); parent=bar_body_frame, child=bar_end_lower_frame, id="BarEndLowerAttachment")

add_coordinate!(vm, FrameAngularVelocity(bar_body_frame); id="BarAngularVelocity")
add_component!(vm, Inertia(bar_inertia_tensor, "BarAngularVelocity"); id="BarInertia")
add_component!(vm, LinearDamper(0.5 * I(3), "BarAngularVelocity"); id="BarRotationalDamper")
add_component!(vm, LinearInerter(0.1, "AttractionPoint"); id="SliderInertance")

# --- 3. DEFINIZIONE DEI COMPONENTI VIRTUALI DI CONTROLLO ---
println("3. Setup del VMS e dei componenti di controllo...")
vms = VirtualMechanismSystem("SlidingControl", robot, vm)

# === COMPONENTI PER L'INTERAZIONE CON GLI OSTACOLI (PER LA SBARRA VIRTUALE) ===
obstacle_position_1 = SVector(0.4, 0.2, 0.5)
obstacle_position_2 = SVector(0.4, -0.1, 0.4)
obstacle_position_3 = SVector(0.5, 0.4, 0.5) 

add_coordinate!(vm, ConstCoord(obstacle_position_1); id="ObstaclePosition1")
add_coordinate!(vm, ConstCoord(obstacle_position_2); id="ObstaclePosition2")
add_coordinate!(vm, ConstCoord(obstacle_position_3); id="ObstaclePosition3")

add_coordinate!(vm, FrameOrigin(upper_attach_frame); id="UpperAttachPoint")
add_coordinate!(vm, FrameOrigin(lower_attach_frame); id="LowerAttachPoint")
add_coordinate!(vm, FrameOrigin(bar_end_upper_frame); id="BarEndUpperPoint")
add_coordinate!(vm, FrameOrigin(bar_end_lower_frame); id="BarEndLowerPoint")

# Ostacolo 1
add_coordinate!(vms, CoordDifference(".virtual_mechanism.UpperAttachPoint", ".virtual_mechanism.ObstaclePosition1"); id="UpperError1")
add_coordinate!(vms, CoordDifference(".virtual_mechanism.LowerAttachPoint", ".virtual_mechanism.ObstaclePosition1"); id="LowerError1")
add_component!(vms, GaussianSpring("UpperError1"; stiffness=-150.0, width=0.1); id="UpperObstacleSpring1")
add_component!(vms, LinearDamper(20.0, "UpperError1"); id="UpperObstacleDamper1")
add_component!(vms, GaussianSpring("LowerError1"; stiffness=-150.0, width=0.1); id="LowerObstacleSpring1")
add_component!(vms, LinearDamper(20.0, "LowerError1"); id="LowerObstacleDamper1")

# Ostacolo 2
add_coordinate!(vms, CoordDifference(".virtual_mechanism.UpperAttachPoint", ".virtual_mechanism.ObstaclePosition2"); id="UpperError2")
add_coordinate!(vms, CoordDifference(".virtual_mechanism.LowerAttachPoint", ".virtual_mechanism.ObstaclePosition2"); id="LowerError2")
add_component!(vms, GaussianSpring("UpperError2"; stiffness=-150.0, width=0.1); id="UpperObstacleSpring2")
add_component!(vms, LinearDamper(20.0, "UpperError2"); id="UpperObstacleDamper2")
add_component!(vms, GaussianSpring("LowerError2"; stiffness=-150.0, width=0.1); id="LowerObstacleSpring2")
add_component!(vms, LinearDamper(20.0, "LowerError2"); id="LowerObstacleDamper2")

# Componenti per l'interazione con l'Ostacolo 3
add_coordinate!(vms, CoordDifference(".virtual_mechanism.UpperAttachPoint", ".virtual_mechanism.ObstaclePosition3"); id="UpperError3")
add_coordinate!(vms, CoordDifference(".virtual_mechanism.LowerAttachPoint", ".virtual_mechanism.ObstaclePosition3"); id="LowerError3")
add_component!(vms, GaussianSpring("UpperError3"; stiffness=-150.0, width=0.1); id="UpperObstacleSpring3")
add_component!(vms, LinearDamper(20.0, "UpperError3"); id="UpperObstacleDamper3")
add_component!(vms, GaussianSpring("LowerError3"; stiffness=-150.0, width=0.1); id="LowerObstacleSpring3")
add_component!(vms, LinearDamper(20.0, "LowerError3"); id="LowerObstacleDamper3")


# === COMPONENTI PER IL CONTROLLO DEL ROBOT (Invariato) ===
add_coordinate!(robot, FramePoint("fr3_link8", SVector(0.0, 0.0, 0.0)); id="TCP")
add_coordinate!(vm, FrameOrigin(attraction_point_frame); id="AttractionPoint")
add_coordinate!(vms, CoordDifference(".virtual_mechanism.AttractionPoint", ".robot.TCP"); id="PathError")
K_path = SMatrix{3, 3}(250.0, 0, 0, 0, 250.0, 0, 0, 0, 250.0)
D_path = SMatrix{3, 3}(100.0, 0, 0, 0, 100.0, 0, 0, 0, 100.0)
add_component!(vms, LinearSpring(K_path, "PathError"); id="PathSpring")
add_component!(vms, LinearDamper(D_path, "PathError"); id="PathDamper")

# === COMPONENTI PER L'EVITAMENTO DEGLI OSTACOLI MULTI-LINK (Invariato) ===
links_da_proteggere = ["fr3_link4","fr3_link5", "fr3_link6", "fr3_link7", "fr3_link8"]
ostacoli = [
    (id_num=1, coord_id=".virtual_mechanism.ObstaclePosition1"),
    (id_num=2, coord_id=".virtual_mechanism.ObstaclePosition2"),
    (id_num=3, coord_id=".virtual_mechanism.ObstaclePosition3") # AGGIUNGIAMO L'OSTACOLO 3 ALLA LISTA
]
for link_name in links_da_proteggere
    link_origin_coord_id = "$(link_name)_origin"
    add_coordinate!(robot, FrameOrigin(link_name); id=link_origin_coord_id)
    for ostacolo in ostacoli
        error_coord_id = "$(link_name)_ObstacleError$(ostacolo.id_num)"
        add_coordinate!(vms, CoordDifference(".robot.$(link_origin_coord_id)", ostacolo.coord_id); id=error_coord_id)
        spring_id = "$(link_name)_ObstacleSpring$(ostacolo.id_num)"
        add_component!(vms, GaussianSpring(error_coord_id; stiffness=-80.0, width=0.1); id=spring_id)
    end
end

# =======================================================================================
# --- LOGICA ADATTIVA: SCELTA DINAMICA DEL PUNTO DI PARTENZA ---
# =======================================================================================
println("3.1. Scelta dinamica del punto di partenza sulla sbarra...")

obstacle_positions = [obstacle_position_1, obstacle_position_2, obstacle_position_3] # AGGIUNGIAMO LA POSIZIONE 3

top_start_pos_local = SVector(0.0, 0.5 * bar_length, 0.0)
bottom_start_pos_local = SVector(0.0, -0.5 * bar_length, 0.0)
top_start_pos_world = pivot_point + top_start_pos_local
bottom_start_pos_world = pivot_point + bottom_start_pos_local

dist_top_to_obstacles = [norm(top_start_pos_world - obs) for obs in obstacle_positions]
dist_bottom_to_obstacles = [norm(bottom_start_pos_world - obs) for obs in obstacle_positions]

safety_score_top = minimum(dist_top_to_obstacles)
safety_score_bottom = minimum(dist_bottom_to_obstacles)

local slider_initial_pos::Float64
local start_from_bottom::Bool

if safety_score_bottom >= safety_score_top
    println("Decisione: Partenza dal basso. È più lontano dagli ostacoli.")
    start_from_bottom = true
    slider_initial_pos = -0.5 * bar_length
else
    println("Decisione: Partenza dall'alto. È più lontano dagli ostacoli.")
    start_from_bottom = false
    slider_initial_pos = 0.5 * bar_length
end

# --- Componenti di controllo del meccanismo virtuale (Configurazione Dinamica) ---

# 1. CONTROLLO DEL CURSORE (SLIDER)
add_coordinate!(vm, JointSubspace("J_slider"); id="SliderCoord")
add_coordinate!(vm, ConstCoord(SVector(0.0)); id="SliderTarget") # Target fisso al centro (posizione di equilibrio)
add_coordinate!(vms, CoordDifference(".virtual_mechanism.SliderCoord", ".virtual_mechanism.SliderTarget"); id="SliderDrivingError")
K_drive = 100.0
D_drive = 50.0
add_component!(vms, LinearSpring(K_drive, "SliderDrivingError"); id="SliderDrivingSpring")
add_component!(vms, LinearDamper(D_drive, "SliderDrivingError"); id="SliderDrivingDamper")

# 2. CONTROLLO DELL'ORIENTAMENTO (si adatta alla direzione di approccio al centro)
add_coordinate!(vm, FrameOrigin(bar_body_frame); id="BarOrigin")
add_coordinate!(vm, FramePoint(bar_body_frame, SVector(0.0, -1.0, 0.0)); id="BarAxisPoint") # Vettore Y-
add_coordinate!(vm, CoordDifference("BarAxisPoint", "BarOrigin"); id="BarAxisVector")
add_coordinate!(vm, FramePoint(bar_body_frame, SVector(0.0, 1.0, 0.0)); id="InvertedBarAxisPoint") # Vettore Y+
add_coordinate!(vm, CoordDifference("InvertedBarAxisPoint", "BarOrigin"); id="InvertedBarAxisVector")

add_coordinate!(robot, FrameOrigin("fr3_link8"); id="ToolOrigin")
add_coordinate!(robot, FramePoint("fr3_link8", SVector(0.0, 0.0, 1.0)); id="ToolAxisPoint") # Asse Z del Tool
add_coordinate!(robot, CoordDifference("ToolAxisPoint", "ToolOrigin"); id="ToolAxisVector")

# === CORREZIONE DELLA LOGICA DI ORIENTAMENTO ===
# Se partiamo dal basso, dobbiamo muoverci in direzione Y+ per arrivare al centro.
# Se partiamo dall'alto, dobbiamo muoverci in direzione Y- per arrivare al centro.
orientation_target_vector_id = start_from_bottom ? ".virtual_mechanism.InvertedBarAxisVector" : ".virtual_mechanism.BarAxisVector"
println("Vettore target per l'orientamento (corretto): $orientation_target_vector_id")

add_coordinate!(vms, CoordDifference(orientation_target_vector_id, ".robot.ToolAxisVector"); id="OrientationError")
K_rot = 100.0
D_rot = 50.0
add_component!(vms, LinearSpring(K_rot, "OrientationError"); id="OrientationSpring")
add_component!(vms, LinearDamper(D_rot, "OrientationError"); id="OrientationDamper")

# =======================================================================================
# --- FINE LOGICA ADATTIVA ---
# =======================================================================================

# --- 4. SIMULAZIONE ---
println("4. Avvio della simulazione...")
tspan = (0.0, 15.0)

q_vm_initial = [0.0, 0.0, 0.0, slider_initial_pos]
q_initial = ([0.0, -π/4, 0.0, -3*π/4, 0.0, π/2, π/4], q_vm_initial)
q̇_initial = (zeros(7), zeros(4))
g = VMRobotControl.DEFAULT_GRAVITY

compiled_vms = compile(vms)
dcache = new_dynamics_cache(compiled_vms)
prob = get_ode_problem(dcache, g, q_initial, q̇_initial, tspan)
sol = solve(prob, Tsit5(); maxiters=3e5, abstol=1e-6, reltol=1e-6)

# --- 5. VISUALIZZAZIONE ---
println("5. Generazione dell'animazione...")
fig = Figure(size = (800, 800))
display(fig)
ls = LScene(fig[1, 1]; show_axis=true)
cam = cam3d!(ls, camera=:perspective, center=false)
cam.lookat[] = [0.4, 0.1, 0.5]
cam.eyeposition[] = [1.7, 0.8, 1.0]
plotting_kcache = Observable(new_kinematics_cache(compiled_vms))
robotvisualize!(ls, plotting_kcache)
mesh!(ls, Sphere(Point3f(pivot_point), 0.03), color=(:green, 0.9), label="Pivot")
mesh!(ls, Sphere(Point3f(obstacle_position_1), 0.04), color=(:orange, 0.9), label="Ostacolo 1")
mesh!(ls, Sphere(Point3f(obstacle_position_2), 0.04), color=(:magenta, 0.9), label="Ostacolo 2")
mesh!(ls, Sphere(Point3f(obstacle_position_3), 0.04), color=(:yellow, 0.9), label="Ostacolo 3")

# ID per le coordinate
obs1_id = get_compiled_coordID(compiled_vms, ".virtual_mechanism.ObstaclePosition1")
obs2_id = get_compiled_coordID(compiled_vms, ".virtual_mechanism.ObstaclePosition2")
obs3_id = get_compiled_coordID(compiled_vms, ".virtual_mechanism.ObstaclePosition3")
upper_attach_id = get_compiled_coordID(compiled_vms, ".virtual_mechanism.UpperAttachPoint")
lower_attach_id = get_compiled_coordID(compiled_vms, ".virtual_mechanism.LowerAttachPoint")
attraction_id = get_compiled_coordID(compiled_vms, ".virtual_mechanism.AttractionPoint")
bar_end_upper_id = get_compiled_coordID(compiled_vms, ".virtual_mechanism.BarEndUpperPoint")
bar_end_lower_id = get_compiled_coordID(compiled_vms, ".virtual_mechanism.BarEndLowerPoint")

# Osservabili per le posizioni
obs1_pos = map(c -> Point3f(configuration(c, obs1_id)), plotting_kcache)
obs2_pos = map(c -> Point3f(configuration(c, obs2_id)), plotting_kcache)
obs3_pos = map(c -> Point3f(configuration(c, obs3_id)), plotting_kcache)
upper_attach_pos = map(c -> Point3f(configuration(c, upper_attach_id)), plotting_kcache)
lower_attach_pos = map(c -> Point3f(configuration(c, lower_attach_id)), plotting_kcache)
attraction_pos = map(c -> Point3f(configuration(c, attraction_id)), plotting_kcache)
bar_end_upper_pos = map(c -> Point3f(configuration(c, bar_end_upper_id)), plotting_kcache)
bar_end_lower_pos = map(c -> Point3f(configuration(c, bar_end_lower_id)), plotting_kcache)

# Disegna le molle virtuali
lines!(ls, map((o, u) -> [o, u], obs1_pos, upper_attach_pos), color=:purple, linewidth=2, linestyle=:dash)
lines!(ls, map((o, l) -> [o, l], obs1_pos, lower_attach_pos), color=:purple, linewidth=2, linestyle=:dash)
lines!(ls, map((o, u) -> [o, u], obs2_pos, upper_attach_pos), color=:cyan, linewidth=2, linestyle=:dash)
lines!(ls, map((o, l) -> [o, l], obs2_pos, lower_attach_pos), color=:cyan, linewidth=2, linestyle=:dash)
# NUOVE LINEE PER OSTACOLO 3
lines!(ls, map((o, u) -> [o, u], obs3_pos, upper_attach_pos), color=:yellow, linewidth=2, linestyle=:dash)
lines!(ls, map((o, l) -> [o, l], obs3_pos, lower_attach_pos), color=:yellow, linewidth=2, linestyle=:dash)


# Disegna la sbarra virtuale
lines!(ls, map((u, l) -> [u, l], bar_end_upper_pos, bar_end_lower_pos), color=:red, linewidth=5)

# Disegna il punto di attrazione/cursore
scatter!(ls, attraction_pos, color=:blue, markersize=20, label="Punto di Contatto")

# Creazione dell'animazione
savepath = joinpath(@__DIR__, "franka_whole_body_avoidance_final_3_obstacles.mp4")
animate_robot_odesolution(fig, sol, plotting_kcache, savepath)
@info "Animazione salvata in: $savepath"