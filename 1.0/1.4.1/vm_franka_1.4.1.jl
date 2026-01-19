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

upper_attach_frame = add_frame!(vm, "upper_attach")
lower_attach_frame = add_frame!(vm, "lower_attach")
add_joint!(vm, Rigid(Transform(SVector(0.0,  0.25 * bar_length, 0.0))); parent=bar_body_frame, child=upper_attach_frame, id="UpperAttachment")
add_joint!(vm, Rigid(Transform(SVector(0.0, -0.25 * bar_length, 0.0))); parent=bar_body_frame, child=lower_attach_frame, id="LowerAttachment")

bar_end_upper_frame = add_frame!(vm, "bar_end_upper")
bar_end_lower_frame = add_frame!(vm, "bar_end_lower")
add_joint!(vm, Rigid(Transform(SVector(0.0,  0.5 * bar_length, 0.0))); parent=bar_body_frame, child=bar_end_upper_frame, id="BarEndUpperAttachment")
add_joint!(vm, Rigid(Transform(SVector(0.0, -0.5 * bar_length, 0.0))); parent=bar_body_frame, child=bar_end_lower_frame, id="BarEndLowerAttachment")

bar_inertia_val = (1/12) * 2.0 * bar_length^2
bar_inertia_tensor = SMatrix{3,3}(bar_inertia_val, 0, 0, 0, 1e-3, 0, 0, 0, bar_inertia_val)
add_coordinate!(vm, FrameAngularVelocity(bar_body_frame); id="BarAngularVelocity")
add_component!(vm, Inertia(bar_inertia_tensor, "BarAngularVelocity"); id="BarInertia")
add_component!(vm, LinearDamper(0.5 * I(3), "BarAngularVelocity"); id="BarRotationalDamper")
add_component!(vm, LinearInerter(0.1, "AttractionPoint"); id="SliderInertance")

# --- 3. DEFINIZIONE DEI COMPONENTI VIRTUALI DI CONTROLLO ---
println("3. Setup del VMS e dei componenti di controllo...")
vms = VirtualMechanismSystem("SlidingControl", robot, vm)

# === COMPONENTI PER L'INTERAZIONE CON GLI OSTACOLI DINAMICI ===
obstacle_position_1 = SVector(0.4, 0.2, 0.5)
# NUOVA DEFINIZIONE: Posizione iniziale del secondo ostacolo
obstacle_position_2 = SVector(0.5, 0.5, 0.3)

# MODIFICA: Uso di ReferenceCoord per permettere l'aggiornamento della posizione
println("3.1. Creazione degli ostacoli dinamici...")
add_coordinate!(vm, ReferenceCoord(Ref(obstacle_position_1)); id="ObstaclePosition1")
# NUOVA AGGIUNTA: Aggiungiamo il secondo ostacolo come coordinata referenziata
add_coordinate!(vm, ReferenceCoord(Ref(obstacle_position_2)); id="ObstaclePosition2")

add_coordinate!(vm, FrameOrigin(upper_attach_frame); id="UpperAttachPoint")
add_coordinate!(vm, FrameOrigin(lower_attach_frame); id="LowerAttachPoint")
add_coordinate!(vm, FrameOrigin(bar_end_upper_frame); id="BarEndUpperPoint")
add_coordinate!(vm, FrameOrigin(bar_end_lower_frame); id="BarEndLowerPoint")

# Molle repulsive per l'ostacolo 1
add_coordinate!(vms, CoordDifference(".virtual_mechanism.UpperAttachPoint", ".virtual_mechanism.ObstaclePosition1"); id="UpperError1")
add_coordinate!(vms, CoordDifference(".virtual_mechanism.LowerAttachPoint", ".virtual_mechanism.ObstaclePosition1"); id="LowerError1")
add_component!(vms, GaussianSpring("UpperError1"; stiffness=-150.0, width=0.1); id="UpperObstacleSpring1")
add_component!(vms, LinearDamper(20.0, "UpperError1"); id="UpperObstacleDamper1")
add_component!(vms, GaussianSpring("LowerError1"; stiffness=-150.0, width=0.1); id="LowerObstacleSpring1")
add_component!(vms, LinearDamper(20.0, "LowerError1"); id="LowerObstacleDamper1")

#Molle repulsive per l'ostacolo 2
add_coordinate!(vms, CoordDifference(".virtual_mechanism.UpperAttachPoint", ".virtual_mechanism.ObstaclePosition2"); id="UpperError2")
add_coordinate!(vms, CoordDifference(".virtual_mechanism.LowerAttachPoint", ".virtual_mechanism.ObstaclePosition2"); id="LowerError2")
add_component!(vms, GaussianSpring("UpperError2"; stiffness=-150.0, width=0.1); id="UpperObstacleSpring2")
add_component!(vms, LinearDamper(20.0, "UpperError2"); id="UpperObstacleDamper2")
add_component!(vms, GaussianSpring("LowerError2"; stiffness=-150.0, width=0.1); id="LowerObstacleSpring2")
add_component!(vms, LinearDamper(20.0, "LowerError2"); id="LowerObstacleDamper2")

# === COMPONENTI PER IL CONTROLLO DEL ROBOT (Invariato) ===
add_coordinate!(robot, FramePoint("fr3_link8", SVector(0.0, 0.0, 0.0)); id="TCP")
add_coordinate!(vm, FrameOrigin(attraction_point_frame); id="AttractionPoint")
add_coordinate!(vms, CoordDifference(".virtual_mechanism.AttractionPoint", ".robot.TCP"); id="PathError")
K_path = SMatrix{3, 3}(250.0, 0, 0, 0, 250.0, 0, 0, 0, 250.0)
D_path = SMatrix{3, 3}(100.0, 0, 0, 0, 100.0, 0, 0, 0, 100.0)
add_component!(vms, LinearSpring(K_path, "PathError"); id="PathSpring")
add_component!(vms, LinearDamper(D_path, "PathError"); id="PathDamper")

# === COMPONENTI PER L'EVITAMENTO DEGLI OSTACOLI MULTI-LINK ===
links_da_proteggere = ["fr3_link4","fr3_link5", "fr3_link6", "fr3_link7", "fr3_link8"]
# MODIFICA: Aggiungiamo il secondo ostacolo alla lista
ostacoli = [
    (id_num=1, coord_id=".virtual_mechanism.ObstaclePosition1"),
    (id_num=2, coord_id=".virtual_mechanism.ObstaclePosition2"),
]
for link_name in links_da_proteggere
    link_origin_coord_id = "$(link_name)_origin"
    add_coordinate!(robot, FrameOrigin(link_name); id=link_origin_coord_id)
    for ostacolo in ostacoli
        error_coord_id = "$(link_name)_ObstacleError$(ostacolo.id_num)"
        add_coordinate!(vms, CoordDifference(".robot.$(link_origin_coord_id)", ostacolo.coord_id); id=error_coord_id)
        spring_id = "$(link_name)_ObstacleSpring$(ostacolo.id_num)"
        add_component!(vms, GaussianSpring(error_coord_id; stiffness=-500.0, width=0.1); id=spring_id)
    end
end

# =======================================================================================
# --- LOGICA ADATTIVA ---
# =======================================================================================
println("3.2. Scelta dinamica del punto di partenza sulla sbarra...")

# MODIFICA: La logica ora si basa sulla posizione iniziale di entrambi gli ostacoli
obstacle_positions = [obstacle_position_1, obstacle_position_2] 
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

# --- Componenti di controllo del meccanismo virtuale (Configurazione Dinamica, Invariato) ---
add_coordinate!(vm, JointSubspace("J_slider"); id="SliderCoord")
add_coordinate!(vm, ConstCoord(SVector(0.0)); id="SliderTarget")
add_coordinate!(vms, CoordDifference(".virtual_mechanism.SliderCoord", ".virtual_mechanism.SliderTarget"); id="SliderDrivingError")
add_component!(vms, LinearSpring(100.0, "SliderDrivingError"); id="SliderDrivingSpring")
add_component!(vms, LinearDamper(75.0, "SliderDrivingError"); id="SliderDrivingDamper")

add_coordinate!(vm, FrameOrigin(bar_body_frame); id="BarOrigin")
add_coordinate!(vm, FramePoint(bar_body_frame, SVector(0.0, -1.0, 0.0)); id="BarAxisPoint")
add_coordinate!(vm, CoordDifference("BarAxisPoint", "BarOrigin"); id="BarAxisVector")
add_coordinate!(vm, FramePoint(bar_body_frame, SVector(0.0, 1.0, 0.0)); id="InvertedBarAxisPoint")
add_coordinate!(vm, CoordDifference("InvertedBarAxisPoint", "BarOrigin"); id="InvertedBarAxisVector")
add_coordinate!(robot, FrameOrigin("fr3_link8"); id="ToolOrigin")
add_coordinate!(robot, FramePoint("fr3_link8", SVector(0.0, 0.0, 1.0)); id="ToolAxisPoint")
add_coordinate!(robot, CoordDifference("ToolAxisPoint", "ToolOrigin"); id="ToolAxisVector")

orientation_target_vector_id = start_from_bottom ? ".virtual_mechanism.InvertedBarAxisVector" : ".virtual_mechanism.BarAxisVector"
println("Vettore target per l'orientamento: $orientation_target_vector_id")
add_coordinate!(vms, CoordDifference(orientation_target_vector_id, ".robot.ToolAxisVector"); id="OrientationError")
add_component!(vms, LinearSpring(100.0, "OrientationError"); id="OrientationSpring")
add_component!(vms, LinearDamper(50.0, "OrientationError"); id="OrientationDamper")

# =======================================================================================
# --- 4. SIMULAZIONE CON OSTACOLI DINAMICI ---
# =======================================================================================
println("4. Avvio della simulazione con ostacoli dinamici...")
tspan = (0.0, 15.0)

q_vm_initial = [0.0, 0.0, 0.0, slider_initial_pos]
q_initial = ([0.0, -π/4, 0.0, -3*π/4, 0.0, π/2, π/4], q_vm_initial)
q̇_initial = (zeros(7), zeros(4))
g = VMRobotControl.DEFAULT_GRAVITY

# MODIFICA: Definizione della traiettoria degli ostacoli e delle callback
base_obstacle_pos_1 = obstacle_position_1
amplitude_1 = 0.15 # Metri (movimento su asse Y)
frequency_1 = 0.2  # Hertz

base_obstacle_pos_2 = obstacle_position_2
amplitude_2 = 0.2  # Metri (movimento su asse Z)
frequency_2 = 0.25 # Hertz

""" Definisce la posizione dell'ostacolo 1 in funzione del tempo. """
obstacle_1_pos(t) = base_obstacle_pos_1 + SVector(0.0, amplitude_1 * sin(2 * π * frequency_1 * t), 0.0)

""" NUOVA FUNZIONE: Definisce la posizione dell'ostacolo 2 in funzione del tempo (movimento lungo Z). """
obstacle_2_pos(t) = base_obstacle_pos_2 + SVector(0.0, 0.0, amplitude_2 * sin(2 * π * frequency_2 * t))

""" Funzione di setup: chiamata una volta prima della simulazione per ottenere i riferimenti. """
function f_setup(cache)
    obs1_coord_id = get_compiled_coordID(cache, ".virtual_mechanism.ObstaclePosition1")
    obs1_pos_ref = cache[obs1_coord_id].coord_data.val
    
    obs2_coord_id = get_compiled_coordID(cache, ".virtual_mechanism.ObstaclePosition2")
    obs2_pos_ref = cache[obs2_coord_id].coord_data.val
    
    return (obs1_pos_ref, obs2_pos_ref)
end

""" Funzione di controllo: chiamata ad ogni passo della simulazione per aggiornare i valori. """
function f_control(cache, t, args, extra)
    (obs1_pos_ref, obs2_pos_ref) = args
    obs1_pos_ref[] = obstacle_1_pos(t)
    obs2_pos_ref[] = obstacle_2_pos(t)
    return false
end

compiled_vms = compile(vms)
dcache = new_dynamics_cache(compiled_vms)
prob = get_ode_problem(dcache, g, q_initial, q̇_initial, tspan; f_setup, f_control)
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

# ID per le coordinate
obs1_id = get_compiled_coordID(compiled_vms, ".virtual_mechanism.ObstaclePosition1")
obs2_id = get_compiled_coordID(compiled_vms, ".virtual_mechanism.ObstaclePosition2")
upper_attach_id = get_compiled_coordID(compiled_vms, ".virtual_mechanism.UpperAttachPoint")
lower_attach_id = get_compiled_coordID(compiled_vms, ".virtual_mechanism.LowerAttachPoint")
attraction_id = get_compiled_coordID(compiled_vms, ".virtual_mechanism.AttractionPoint")
bar_end_upper_id = get_compiled_coordID(compiled_vms, ".virtual_mechanism.BarEndUpperPoint")
bar_end_lower_id = get_compiled_coordID(compiled_vms, ".virtual_mechanism.BarEndLowerPoint")

# Osservabili per le posizioni
obs1_pos = map(c -> Point3f(configuration(c, obs1_id)), plotting_kcache)
obs2_pos = map(c -> Point3f(configuration(c, obs2_id)), plotting_kcache)
upper_attach_pos = map(c -> Point3f(configuration(c, upper_attach_id)), plotting_kcache)
lower_attach_pos = map(c -> Point3f(configuration(c, lower_attach_id)), plotting_kcache)
attraction_pos = map(c -> Point3f(configuration(c, attraction_id)), plotting_kcache)
bar_end_upper_pos = map(c -> Point3f(configuration(c, bar_end_upper_id)), plotting_kcache)
bar_end_lower_pos = map(c -> Point3f(configuration(c, bar_end_lower_id)), plotting_kcache)

# Visualizzazione degli ostacoli
meshscatter!(
    ls, obs1_pos, marker = Sphere(Point3f(0), 1), markersize = 0.04,
    color = (:orange, 0.9), label = "Ostacolo 1"
)
meshscatter!(
    ls, obs2_pos, marker = Sphere(Point3f(0), 1), markersize = 0.04,
    color = (:cyan, 0.9), label = "Ostacolo 2"
)

# Disegna le molle virtuali
lines!(ls, map((o, u) -> [o, u], obs1_pos, upper_attach_pos), color=:purple, linewidth=2, linestyle=:dash)
lines!(ls, map((o, l) -> [o, l], obs1_pos, lower_attach_pos), color=:purple, linewidth=2, linestyle=:dash)
lines!(ls, map((o, u) -> [o, u], obs2_pos, upper_attach_pos), color=:turquoise, linewidth=2, linestyle=:dash)
lines!(ls, map((o, l) -> [o, l], obs2_pos, lower_attach_pos), color=:turquoise, linewidth=2, linestyle=:dash)

# Disegna la sbarra virtuale e il punto di attrazione
lines!(ls, map((u, l) -> [u, l], bar_end_upper_pos, bar_end_lower_pos), color=:red, linewidth=5)
scatter!(ls, attraction_pos, color=:blue, markersize=20, label="Punto di Contatto")

# Creazione dell'animazione
savepath = joinpath(@__DIR__, "franka_dynamic_avoidance_2_obstacles.mp4")
animate_robot_odesolution(fig, sol, plotting_kcache, savepath; f_setup, f_control)
@info "Animazione salvata in: $savepath"