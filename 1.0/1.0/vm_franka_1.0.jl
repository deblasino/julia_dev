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

# --- 2. DEFINIZIONE DEL MECCANISMO VIRTUALE (La Sbarra Rotante) ---
println("2. Creazione della sbarra virtuale rotante...")

bar_length = 0.5 # Lunghezza della sbarra
bar_inertia_val = (1/12) * 2.0 * bar_length^2 # Massa virtuale di 2kg
bar_inertia_tensor = SMatrix{3,3}(bar_inertia_val, 0, 0, 0, 1e-3, 0, 0, 0, bar_inertia_val)

# Il centro della sbarra è fisso in questo punto, che funge da perno.
pivot_point = SVector(0.5, 0.2, 0.6)

# Creiamo il meccanismo virtuale (vm)
vm = Mechanism{Float64}("VirtualRotatingBar")

pivot_frame = add_frame!(vm, "pivot_frame") 
frame_rz = add_frame!(vm, "frame_rz")
frame_ry = add_frame!(vm, "frame_ry")
bar_body_frame = add_frame!(vm, "bar_body_frame") 

# 1. Fissiamo il punto di rotazione nello spazio
add_joint!(vm, Rigid(Transform(pivot_point)); parent=root_frame(vm), child=pivot_frame, id="PivotAnchor") # <-- CORREZIONE QUI

# 2. Aggiungiamo i 3 gradi di libertà rotazionali a partire dal perno
add_joint!(vm, Revolute(SVector(0.,0.,1.)); parent=pivot_frame,    child=frame_rz, id="J_yaw")
add_joint!(vm, Revolute(SVector(0.,1.,0.)); parent=frame_rz,       child=frame_ry, id="J_pitch")
add_joint!(vm, Revolute(SVector(1.,0.,0.)); parent=frame_ry,       child=bar_body_frame, id="J_roll")

# Aggiungiamo i frame per i punti di attacco sulla sbarra (superiore e inferiore)
upper_attach_frame = add_frame!(vm, "upper_attach")
lower_attach_frame = add_frame!(vm, "lower_attach")
add_joint!(vm, Rigid(Transform(SVector(0.0,  0.5 * bar_length, 0.0))); parent=bar_body_frame, child=upper_attach_frame, id="UpperAttachment") # <-- CORREZIONE QUI
add_joint!(vm, Rigid(Transform(SVector(0.0, -0.5 * bar_length, 0.0))); parent=bar_body_frame, child=lower_attach_frame, id="LowerAttachment") # <-- CORREZIONE QUI


# Aggiungiamo l'inerzia al corpo della sbarra. 
add_coordinate!(vm, FrameAngularVelocity(bar_body_frame); id="BarAngularVelocity")
add_component!(vm, Inertia(bar_inertia_tensor, "BarAngularVelocity"); id="BarInertia")
add_component!(vm, LinearDamper(0.5 * I(3), "BarAngularVelocity"); id="BarRotationalDamper")


# --- 3. DEFINIZIONE DELL'OSTACOLO E DEI COMPONENTI VIRTUALI ---
println("3. Setup del VMS, dell'ostacolo e dei componenti di controllo...")
vms = VirtualMechanismSystem("RotationalAvoidanceControl", robot, vm)

# ... (La parte dell'ostacolo e delle molle repulsive rimane identica) ...
obstacle_position = SVector(0.4, 0.0, 0.5)
add_coordinate!(vm, ConstCoord(obstacle_position); id="ObstaclePosition")
add_coordinate!(vm, FrameOrigin(upper_attach_frame); id="UpperAttachPoint")
add_coordinate!(vm, FrameOrigin(lower_attach_frame); id="LowerAttachPoint")
add_coordinate!(vms, CoordDifference(".virtual_mechanism.UpperAttachPoint", ".virtual_mechanism.ObstaclePosition"); id="UpperError")
add_coordinate!(vms, CoordDifference(".virtual_mechanism.LowerAttachPoint", ".virtual_mechanism.ObstaclePosition"); id="LowerError")
K_obs = -150.0
D_obs = 20.0
add_component!(vms, GaussianSpring("UpperError"; stiffness=K_obs, width=0.1); id="UpperObstacleSpring")
add_component!(vms, LinearDamper(D_obs, "UpperError"); id="UpperObstacleDamper")
add_component!(vms, GaussianSpring("LowerError"; stiffness=K_obs, width=0.1); id="LowerObstacleSpring")
add_component!(vms, LinearDamper(D_obs, "LowerError"); id="LowerObstacleDamper")

# Componente per vincolare il TCP del robot al CENTRO della sbarra rotante
add_coordinate!(robot, FramePoint("fr3_link8", SVector(0.0, 0.0, 0.0)); id="TCP")
add_coordinate!(vm, FrameOrigin(pivot_frame); id="PivotPoint")
add_coordinate!(vms, CoordDifference(".virtual_mechanism.PivotPoint", ".robot.TCP"); id="PathError")
K_path = SMatrix{3, 3}(600.0, 0, 0, 0, 600.0, 0, 0, 0, 600.0)
D_path = SMatrix{3, 3}(80.0, 0, 0, 0, 80.0, 0, 0, 0, 80.0)
add_component!(vms, LinearSpring(K_path, "PathError"); id="PathSpring")
add_component!(vms, LinearDamper(D_path, "PathError"); id="PathDamper")

# --- INIZIO CORREZIONE: ACCOPPIAMENTO ROTAZIONALE TRAMITE VINCOLO VIRTUALE ---
# Definiamo un punto "leva" solidale all'utensile del robot (es. 10cm lungo l'asse Z)
tool_vector = SVector(0.0, 0.0, 0.1)
add_coordinate!(robot, FramePoint("fr3_link8", tool_vector); id="RobotOrientationHandle")

# Definiamo un punto corrispondente solidale alla sbarra virtuale
add_coordinate!(vm, FramePoint(bar_body_frame, tool_vector); id="BarOrientationHandle")

# Creiamo l'errore tra questi due punti.
# Una molla su questo errore tirerà i due punti a coincidere, allineando di fatto gli orientamenti.
add_coordinate!(vms, CoordDifference(".virtual_mechanism.BarOrientationHandle", ".robot.RobotOrientationHandle"); id="OrientationError")
K_rot = 200.0 # Rigidità della molla di allineamento
D_rot = 40.0  # Smorzamento
add_component!(vms, LinearSpring(K_rot, "OrientationError"); id="OrientationSpring")
add_component!(vms, LinearDamper(D_rot, "OrientationError"); id="OrientationDamper")
# --- FINE CORREZIONE ---


# --- 4. SIMULAZIONE ---
println("4. Avvio della simulazione...")
tspan = (0.0, 10.0)

# Stato iniziale: q_robot + q_vm. Il vm ha 3 DoF (yaw, pitch, roll).
q_vm_initial = [0.0, 0.0, 0.0] # Angoli iniziali
q_initial = ([0.0, -π/4, 0.0, -3*π/4, 0.0, π/2, π/4], q_vm_initial)
q̇_initial = (zeros(7), zeros(3)) # Parte da fermo

g = VMRobotControl.DEFAULT_GRAVITY
compiled_vms = compile(vms)
dcache = new_dynamics_cache(compiled_vms)
prob = get_ode_problem(dcache, g, q_initial, q̇_initial, tspan)
sol = solve(prob, Tsit5(); maxiters=2e5, abstol=1e-6, reltol=1e-6)

## --- 5. VISUALIZZAZIONE ---
println("5. Generazione dell'animazione...")
fig = Figure(size = (800, 800))
display(fig)
ls = LScene(fig[1, 1]; show_axis=true)
cam = cam3d!(ls, camera=:perspective, center=false)
cam.lookat[] = [0.4, 0.1, 0.5]
cam.eyeposition[] = [1.2, 0.8, 1.0]

plotting_kcache = Observable(new_kinematics_cache(compiled_vms))

# Visualizza il robot (ma non più lo sketch del meccanismo virtuale)
robotvisualize!(ls, plotting_kcache)

# Visualizza il perno (target) e l'ostacolo
mesh!(ls, Sphere(Point3f(pivot_point), 0.03), color=(:green, 0.9))
mesh!(ls, Sphere(Point3f(obstacle_position), 0.04), color=(:orange, 0.9))

# Otteniamo gli ID delle coordinate dei punti che ci servono
obs_id = get_compiled_coordID(compiled_vms, ".virtual_mechanism.ObstaclePosition")
upper_id = get_compiled_coordID(compiled_vms, ".virtual_mechanism.UpperAttachPoint")
lower_id = get_compiled_coordID(compiled_vms, ".virtual_mechanism.LowerAttachPoint")

# Creiamo degli "Observable" per le posizioni 3D
obs_pos = map(c -> Point3f(configuration(c, obs_id)), plotting_kcache)
upper_pos = map(c -> Point3f(configuration(c, upper_id)), plotting_kcache)
lower_pos = map(c -> Point3f(configuration(c, lower_id)), plotting_kcache)

# Visualizziamo le molle repulsive come linee tratteggiate
lines!(ls, map((o, u) -> [o, u], obs_pos, upper_pos), color=:purple, linewidth=2, linestyle=:dash)
lines!(ls, map((o, l) -> [o, l], obs_pos, lower_pos), color=:purple, linewidth=2, linestyle=:dash)

# --- INIZIO CORREZIONE: Disegniamo la sbarra come una singola linea ---
# Invece di `robotsketch!`, usiamo `lines!` per collegare i due punti di attacco.
# Questo disegna la sbarra senza mostrare i giunti interni.
lines!(ls, map((u, l) -> [u, l], upper_pos, lower_pos), color=:red, linewidth=5)
# --- FINE CORREZIONE ---

# Creazione dell'animazione
savepath = joinpath(@__DIR__, "franka_rotating_bar_obstacle.mp4")
animate_robot_odesolution(fig, sol, plotting_kcache, savepath)
@info "Animazione salvata in: $savepath"