# =======================================================================================
# VIRTUAL MODEL CONTROL FOR A FRANKA ARM WITH A FIXED GRIPPER AND WHOLE-BODY OBSTACLE AVOIDANCE
#
# Questo script è un adattamento della strategia di Virtual Model Control (VMC) per il
# robot Franka Emika FR3 con una mano/gripper semplificato (dita fisse).
# =======================================================================================

using DifferentialEquations
using GeometryBasics: Vec3f, Point3f
using GLMakie
using LinearAlgebra
using MeshIO
using StaticArrays
using VMRobotControl

# Il caricamento dei file di mesh del robot Franka (formato .dae/Collada) richiede
# la configurazione esplicita per FileIO.jl.
using FileIO, UUIDs
try
    FileIO.add_format(format"DAE", (), ".dae", [:DigitalAssetExchangeFormatIO => UUID("43182933-f65b-495a-9e05-4d939cea427d")])
catch
    # Questo blocco try-catch gestisce il caso in cui il formato sia già stato aggiunto.
end

# =======================================================================================
# --- 1. ROBOT MODEL CONFIGURATION ---
# =======================================================================================
println("1. Setup del robot (con dita fisse definite nell'URDF)...")

# Sopprime gli avvisi del parser URDF (es. per tag <transmission> non necessari per la simulazione).
cfg = URDFParserConfig(;suppress_warnings=true)

# Costruisce il percorso completo del file URDF.
module_path = joinpath(splitpath(splitdir(pathof(VMRobotControl))[1])[1:end-1])

# === DIFFERENZA: Il modello URDF è diverso (fr3_franka_hand(copy).urdf). ===
# Questo URDF include un modello di pinza con dita fisse.
robot = parseURDF(joinpath(module_path, "URDFs/franka_description/urdfs/fr3_franka_hand(copy).urdf"), cfg)

# Aggiunge il compensatore di gravità, essenziale per una dinamica accurata.
add_gravity_compensation!(robot, VMRobotControl.DEFAULT_GRAVITY)

# === MODIFICA: Configurazione dei giunti del robot. ===
# Il robot completo (braccio) ha 7 gradi di libertà (DOF). Vengono configurati solo questi.
actuated_arm_joints = [
    "fr3_joint1", "fr3_joint2", "fr3_joint3", "fr3_joint4",
    "fr3_joint5", "fr3_joint6", "fr3_joint7"
]

println("INFO: Aggiunta di smorzamento ai 7 giunti del braccio.")
# Aggiunge coordinate e smorzamento lineare (friction simulation) per ogni giunto del braccio.
for joint_name in actuated_arm_joints
    coord_id = "$(joint_name)_coord"
    damper_id = "$(joint_name)_damper"
    # `JointSubspace` crea una coordinata che rappresenta lo stato del giunto.
    add_coordinate!(robot, JointSubspace(joint_name); id=coord_id)
    # `LinearDamper` aggiunge una forza dissipativa proporzionale alla velocità del giunto.
    add_component!(robot, LinearDamper(0.1, coord_id); id=damper_id)
end

# NON è più necessario aggiungere componenti per le dita.
# Sono considerate fisse (`RigidJoint` nell'URDF modificato) e non attuate. 
println("Setup del robot completato. Gradi di libertà totali: 7.")


# =======================================================================================
# --- 2. VIRTUAL MECHANISM DEFINITION (THE ROTATING BAR & SLIDER) ---
# =======================================================================================
println("2. Creazione della sbarra virtuale con cursore mobile...")

# Questo meccanismo virtuale è un'entità dinamica software con cui il robot interagisce.
bar_length = 1.0
bar_inertia_val = (1/12) * 2.0 * bar_length^2 # Inerzia per un'asta snella.
bar_inertia_tensor = SMatrix{3,3}(bar_inertia_val, 0, 0, 0, 1e-3, 0, 0, 0, bar_inertia_val)

pivot_point = SVector(0.5, 0.3, 0.4) # Posizione di ancoraggio (nel mondo) della sbarra.
vm = Mechanism{Float64}("VirtualBarAndSlider")

# Costruzione della cinematica del meccanismo virtuale (3 gradi di libertà rotazionali).
pivot_frame = add_frame!(vm, "pivot_frame")
frame_rz = add_frame!(vm, "frame_rz")
frame_ry = add_frame!(vm, "frame_ry")
bar_body_frame = add_frame!(vm, "bar_body_frame") # Il frame principale della sbarra.
# Ancoraggio del meccanismo al mondo.
add_joint!(vm, Rigid(Transform(pivot_point)); parent=root_frame(vm), child=pivot_frame, id="PivotAnchor")
# Sequenza di giunti Revolute (Yaw, Pitch, Roll) per la rotazione libera nello spazio.
add_joint!(vm, Revolute(SVector(0.,0.,1.)); parent=pivot_frame,    child=frame_rz, id="J_yaw")
add_joint!(vm, Revolute(SVector(0.,1.,0.)); parent=frame_rz,       child=frame_ry, id="J_pitch")
add_joint!(vm, Revolute(SVector(1.,0.,0.)); parent=frame_ry,       child=bar_body_frame, id="J_roll")

# Definizione del cursore (slider).
# È un giunto `Prismatic` che consente il movimento lineare lungo l'asse Y locale della sbarra.
attraction_point_frame = add_frame!(vm, "attraction_point")
# Il robot sarà attratto da questo frame.
add_joint!(vm, Prismatic(SVector(0.0, 1.0, 0.0)); parent=bar_body_frame, child=attraction_point_frame, id="J_slider")

# Frames specifici (punti di controllo) sulla sbarra per l'applicazione delle forze repulsive.
upper_attach_frame = add_frame!(vm, "upper_attach")
lower_attach_frame = add_frame!(vm, "lower_attach")
add_joint!(vm, Rigid(Transform(SVector(0.0,  0.25 * bar_length, 0.0))); parent=bar_body_frame, child=upper_attach_frame, id="UpperAttachment")
add_joint!(vm, Rigid(Transform(SVector(0.0, -0.25 * bar_length, 0.0))); parent=bar_body_frame, child=lower_attach_frame, id="LowerAttachment")

# Frames aggiuntivi solo per la visualizzazione dell'intera lunghezza della sbarra.
bar_end_upper_frame = add_frame!(vm, "bar_end_upper")
bar_end_lower_frame = add_frame!(vm, "bar_end_lower")
add_joint!(vm, Rigid(Transform(SVector(0.0,  0.5 * bar_length, 0.0))); parent=bar_body_frame, child=bar_end_upper_frame, id="BarEndUpperAttachment")
add_joint!(vm, Rigid(Transform(SVector(0.0, -0.5 * bar_length, 0.0))); parent=bar_body_frame, child=bar_end_lower_frame, id="BarEndLowerAttachment")

# Aggiunta delle proprietà dinamiche (massa/inerzia) al meccanismo virtuale.
add_coordinate!(vm, FrameAngularVelocity(bar_body_frame); id="BarAngularVelocity")
add_component!(vm, Inertia(bar_inertia_tensor, "BarAngularVelocity"); id="BarInertia")
add_component!(vm, LinearDamper(0.5 * I(3), "BarAngularVelocity"); id="BarRotationalDamper") # Smorzamento rotazionale.
add_component!(vm, LinearInerter(0.1, "AttractionPoint"); id="SliderInertance") # "Massa" per il cursore.


# =======================================================================================
# --- 3. VIRTUAL CONTROL SYSTEM ASSEMBLY ---
# =======================================================================================
println("3. Setup del VMS e dei componenti di controllo...")
# Il VirtualMechanismSystem (VMS) connette il robot e il meccanismo virtuale.
vms = VirtualMechanismSystem("SlidingControl", robot, vm)

# === VIRTUAL BAR OBSTACLE AVOIDANCE (SBARRA VIRTUALE) ===
# Definizione delle posizioni fisse degli ostacoli.
obstacle_position_1 = SVector(0.4, 0.2, 0.5)
add_coordinate!(vm, ConstCoord(obstacle_position_1); id="ObstaclePosition1")
obstacle_position_2 = SVector(0.4, -0.1, 0.4)
add_coordinate!(vm, ConstCoord(obstacle_position_2); id="ObstaclePosition2")

# Coordinate che rappresentano la posizione (nel mondo) dei punti di attacco della sbarra.
add_coordinate!(vm, FrameOrigin(upper_attach_frame); id="UpperAttachPoint")
add_coordinate!(vm, FrameOrigin(lower_attach_frame); id="LowerAttachPoint")
add_coordinate!(vm, FrameOrigin(bar_end_upper_frame); id="BarEndUpperPoint")
add_coordinate!(vm, FrameOrigin(bar_end_lower_frame); id="BarEndLowerPoint")

# Calcolo degli errori di posizione tra i punti di attacco e gli ostacoli.
add_coordinate!(vms, CoordDifference(".virtual_mechanism.UpperAttachPoint", ".virtual_mechanism.ObstaclePosition1"); id="UpperError1")
add_coordinate!(vms, CoordDifference(".virtual_mechanism.LowerAttachPoint", ".virtual_mechanism.ObstaclePosition1"); id="LowerError1")
# Le molle repulsive sono create con `GaussianSpring` a rigidezza negativa (`stiffness=-150.0`).
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

# === ROBOT-TO-VIRTUAL-MECHANISM COUPLING (ACCOPPIAMENTO) ===
# Il cuore del controllo: una molla/smorzatore forte che forza l'effettore finale del robot a seguire il cursore virtuale.
# === DIFFERENZA: Il frame dell'End-Effector è ora "fr3_hand_tcp". ===
# Questo punto è definito nell'URDF modificato e rappresenta il Tool Center Point (TCP) della pinza.
add_coordinate!(robot, FrameOrigin("fr3_hand_tcp"); id="TCP")
add_coordinate!(vm, FrameOrigin(attraction_point_frame); id="AttractionPoint")
add_coordinate!(vms, CoordDifference(".virtual_mechanism.AttractionPoint", ".robot.TCP"); id="PathError")

K_path = SMatrix{3, 3}(250.0, 0, 0, 0, 250.0, 0, 0, 0, 250.0) # Alta rigidezza per un inseguimento preciso.
D_path = SMatrix{3, 3}(100.0, 0, 0, 0, 100.0, 0, 0, 0, 100.0) # Smorzamento per la stabilità.
add_component!(vms, LinearSpring(K_path, "PathError"); id="PathSpring")
add_component!(vms, LinearDamper(D_path, "PathError"); id="PathDamper")

# === ROBOT WHOLE-BODY OBSTACLE AVOIDANCE (EVITAMENTO OSTACOLI CORPO INTERO) ===
# Componenti repulsive aggiuntivi per i link del robot.
# === DIFFERENZA: La lista dei link da proteggere include ora "fr3_hand". ===
# Il corpo della pinza viene incluso nella logica di evitamento ostacoli.
links_da_proteggere = ["fr3_link4", "fr3_link5", "fr3_link6", "fr3_link7", "fr3_link8", "fr3_hand"]
ostacoli = [
    (id_num=1, coord_id=".virtual_mechanism.ObstaclePosition1"),
    (id_num=2, coord_id=".virtual_mechanism.ObstaclePosition2")
]
for link_name in links_da_proteggere
    link_origin_coord_id = "$(link_name)_origin"
    add_coordinate!(robot, FrameOrigin(link_name); id=link_origin_coord_id)
    for ostacolo in ostacoli
        error_coord_id = "$(link_name)_ObstacleError$(ostacolo.id_num)"
        add_coordinate!(vms, CoordDifference(".robot.$(link_origin_coord_id)", ostacolo.coord_id); id=error_coord_id)
        spring_id = "$(link_name)_ObstacleSpring$(ostacolo.id_num)"
        # La rigidezza è stata leggermente alzata a -120.0 (rispetto a -80.0 in 1.3.3) per compensare il nuovo link.
        add_component!(vms, GaussianSpring(error_coord_id; stiffness=-120.0, width=0.1); id=spring_id)
    end
end

# =======================================================================================
# --- 4. ADAPTIVE LOGIC: DYNAMIC STARTING POINT SELECTION ---
# =======================================================================================
println("3.1. Scelta dinamica del punto di partenza sulla sbarra...")

# Logica adattiva: seleziona il punto di partenza del cursore (slider) più lontano dall'ostacolo più vicino.

obstacle_positions = [obstacle_position_1, obstacle_position_2]
top_start_pos_local = SVector(0.0, 0.5 * bar_length, 0.0)
bottom_start_pos_local = SVector(0.0, -0.5 * bar_length, 0.0)

# Calcola le posizioni di partenza nel mondo (assumendo orientamento iniziale lungo Y).
top_start_pos_world = pivot_point + top_start_pos_local
bottom_start_pos_world = pivot_point + bottom_start_pos_local

# Calcola la distanza di ciascuna estremità da tutti gli ostacoli.
dist_top_to_obstacles = [norm(top_start_pos_world - obs) for obs in obstacle_positions]
dist_bottom_to_obstacles = [norm(bottom_start_pos_world - obs) for obs in obstacle_positions]

# Il "punteggio di sicurezza" è la distanza dall'ostacolo *più vicino*.
safety_score_top = minimum(dist_top_to_obstacles)
safety_score_bottom = minimum(dist_bottom_to_obstacles)

local slider_initial_pos::Float64
local start_from_bottom::Bool

if safety_score_bottom >= safety_score_top
    println("Decisione: Partenza dal basso.È più lontano dagli ostacoli.")
    start_from_bottom = true
    slider_initial_pos = -0.5 * bar_length
else
    println("Decisione: Partenza dall'alto. È più lontano dagli ostacoli.")
    start_from_bottom = false
    slider_initial_pos = 0.5 * bar_length
end

# --- Dynamic Configuration of Virtual Control Components ---

# 1. SLIDER CONTROL
# Molla/smorzatore che guida il cursore dalla sua posizione iniziale (slider_initial_pos)
# verso il centro della sbarra (Target è 0.0).
add_coordinate!(vm, JointSubspace("J_slider"); id="SliderCoord")
add_coordinate!(vm, ConstCoord(SVector(0.0)); id="SliderTarget") # Target è il centro della sbarra.
add_coordinate!(vms, CoordDifference(".virtual_mechanism.SliderCoord", ".virtual_mechanism.SliderTarget"); id="SliderDrivingError")
K_drive = 100.0
D_drive = 50.0
add_component!(vms, LinearSpring(K_drive, "SliderDrivingError"); id="SliderDrivingSpring")
add_component!(vms, LinearDamper(D_drive, "SliderDrivingError"); id="SliderDrivingDamper")

# 2. ORIENTATION CONTROL (Adattamento alla direzione di partenza)
# Allinea l'asse Z dell'effettore del robot (Tool Axis) con la direzione lungo la sbarra,
# in modo da "spingere" il cursore verso il centro.
add_coordinate!(vm, FrameOrigin(bar_body_frame); id="BarOrigin")
add_coordinate!(vm, FramePoint(bar_body_frame, SVector(0.0, -1.0, 0.0)); id="BarAxisPoint") # Vettore lungo -Y locale
add_coordinate!(vm, CoordDifference("BarAxisPoint", "BarOrigin"); id="BarAxisVector")
add_coordinate!(vm, FramePoint(bar_body_frame, SVector(0.0, 1.0, 0.0));id="InvertedBarAxisPoint") # Vettore lungo +Y locale
add_coordinate!(vm, CoordDifference("InvertedBarAxisPoint", "BarOrigin"); id="InvertedBarAxisVector")

# === DIFFERENZA: Uso del nuovo TCP per l'orientamento. ===
add_coordinate!(robot, FrameOrigin("fr3_hand_tcp"); id="ToolOrigin")
add_coordinate!(robot, FramePoint("fr3_hand_tcp", SVector(0.0, 0.0, 1.0)); id="ToolAxisPoint") # Asse Z del Tool (TCP)
add_coordinate!(robot, CoordDifference("ToolAxisPoint", "ToolOrigin"); id="ToolAxisVector")

# Seleziona il vettore target corretto per l'orientamento (punta verso il centro della sbarra).
# Se si parte dal basso, il tool deve puntare verso +Y locale (InvertedBarAxisVector).
orientation_target_vector_id = start_from_bottom ? ".virtual_mechanism.InvertedBarAxisVector" : ".virtual_mechanism.BarAxisVector"
println("Vettore target per l'orientamento (corretto): $orientation_target_vector_id")

add_coordinate!(vms, CoordDifference(orientation_target_vector_id, ".robot.ToolAxisVector"); id="OrientationError")
K_rot = 100.0
D_rot = 50.0
add_component!(vms, LinearSpring(K_rot, "OrientationError"); id="OrientationSpring")
add_component!(vms, LinearDamper(D_rot, "OrientationError"); id="OrientationDamper")


# =======================================================================================
# --- 5. SIMULAZIONE ---
# =======================================================================================
println("4. Avvio della simulazione...") # Numero di sezione aggiornato a 4
tspan = (0.0, 15.0)

# Impostazione delle configurazioni iniziali per il meccanismo virtuale e il robot.
# La posizione iniziale del cursore è determinata dalla logica adattiva.
q_vm_initial = [0.0, 0.0, 0.0, slider_initial_pos] # [yaw, pitch, roll, slider_pos]

# === MODIFICA: La configurazione iniziale del robot ora ha ESATTAMENTE 7 giunti. ===
q_initial = ([0.0, -π/4, 0.0, -3*π/4, 0.0, π/2, π/4], q_vm_initial)
q̇_initial = (zeros(7), zeros(4)) # Anche la velocità iniziale del robot ha 7 elementi.

g = VMRobotControl.DEFAULT_GRAVITY

# Compila il VMS in una struttura immutabile ed efficiente per il calcolo.
compiled_vms = compile(vms)
# Crea una cache per i risultati intermedi della dinamica.
dcache = new_dynamics_cache(compiled_vms)
# Imposta il problema di Equazione Differenziale Ordinaria (ODE).
prob = get_ode_problem(dcache, g, q_initial, q̇_initial, tspan)
# Risolve il problema ODE con l'algoritmo Tsit5.
sol = solve(prob, Tsit5();maxiters=3e5, abstol=1e-6, reltol=1e-6)


# =======================================================================================
# --- 6. VISUALIZZAZIONE ---
# =======================================================================================
println("5. Generazione dell'animazione...") # Numero di sezione aggiornato a 5

# Configurazione della scena di visualizzazione GLMakie.
fig = Figure(size = (800, 800))
display(fig)
ls = LScene(fig[1, 1]; show_axis=true)
cam = cam3d!(ls, camera=:perspective, center=false)
cam.lookat[] = [0.4, 0.1, 0.5]
cam.eyeposition[] = [1.7, 0.8, 1.0]

# Una cache cinematica `Observable` aggiorna automaticamente gli elementi visuali.
plotting_kcache = Observable(new_kinematics_cache(compiled_vms))
robotvisualize!(ls, plotting_kcache)

# Visualizzazione del pivot e degli ostacoli.
mesh!(ls, Sphere(Point3f(pivot_point), 0.03), color=(:green, 0.9), label="Pivot")
mesh!(ls, Sphere(Point3f(obstacle_position_1), 0.04), color=(:orange, 0.9), label="Ostacolo 1")
mesh!(ls, Sphere(Point3f(obstacle_position_2), 0.04), color=(:magenta, 0.9), label="Ostacolo 2")

# Ottiene gli ID delle coordinate da visualizzare.
obs1_id = get_compiled_coordID(compiled_vms, ".virtual_mechanism.ObstaclePosition1")
obs2_id = get_compiled_coordID(compiled_vms, ".virtual_mechanism.ObstaclePosition2")
upper_attach_id = get_compiled_coordID(compiled_vms, ".virtual_mechanism.UpperAttachPoint")
lower_attach_id = get_compiled_coordID(compiled_vms, ".virtual_mechanism.LowerAttachPoint")
attraction_id = get_compiled_coordID(compiled_vms, ".virtual_mechanism.AttractionPoint")
bar_end_upper_id = get_compiled_coordID(compiled_vms, ".virtual_mechanism.BarEndUpperPoint")
bar_end_lower_id = get_compiled_coordID(compiled_vms, ".virtual_mechanism.BarEndLowerPoint")

# Crea punti `Observable` mappando la configurazione dalla cache di plotting.
obs1_pos = map(c -> Point3f(configuration(c, obs1_id)), plotting_kcache)
obs2_pos = map(c -> Point3f(configuration(c, obs2_id)), plotting_kcache)
upper_attach_pos = map(c -> Point3f(configuration(c, upper_attach_id)), plotting_kcache)
lower_attach_pos = map(c -> Point3f(configuration(c, lower_attach_id)), plotting_kcache)
attraction_pos = map(c -> Point3f(configuration(c, attraction_id)), plotting_kcache)
bar_end_upper_pos = map(c -> Point3f(configuration(c, bar_end_upper_id)), plotting_kcache)
bar_end_lower_pos = map(c -> Point3f(configuration(c, bar_end_lower_id)), plotting_kcache)

# Visualizzazione dei componenti virtuali.
# Disegna le linee tratteggiate per rappresentare le molle virtuali repulsive.
lines!(ls, map((o, u) -> [o, u], obs1_pos, upper_attach_pos), color=:purple, linewidth=2, linestyle=:dash)
lines!(ls, map((o, l) -> [o, l], obs1_pos, lower_attach_pos), color=:purple, linewidth=2, linestyle=:dash)
lines!(ls, map((o, u) -> [o, u], obs2_pos, upper_attach_pos), color=:cyan, linewidth=2, linestyle=:dash)
lines!(ls, map((o, l) -> [o, l], obs2_pos, lower_attach_pos), color=:cyan, linewidth=2, linestyle=:dash)

# Disegna la sbarra virtuale.
lines!(ls, map((u, l) -> [u, l], bar_end_upper_pos, bar_end_lower_pos), color=:red, linewidth=5)

# Disegna il punto di attrazione (il cursore).
scatter!(ls, attraction_pos, color=:blue, markersize=20, label="Punto di Contatto")

# Genera e salva l'animazione dalla soluzione ODE.
savepath = joinpath(@__DIR__, "franka_whole_body_avoidance_final.mp4")
animate_robot_odesolution(fig, sol, plotting_kcache, savepath)
@info "Animazione salvata in: $savepath"