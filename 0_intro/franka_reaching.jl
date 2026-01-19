using DifferentialEquations
using GeometryBasics: Vec3f, Point3f
using GLMakie
using LinearAlgebra
using MeshIO
using StaticArrays
using VMRobotControl

using FileIO, UUIDs
try
    FileIO.add_format(format"DAE", (), ".dae", [:DigitalAssetExchangeFormatIO => UUID("43182933-f65b-495a-9e05-4d939cea427d")])
catch
end

# --- 1. SETUP DEL ROBOT ---
cfg = URDFParserConfig(;suppress_warnings=true)
module_path = joinpath(splitpath(splitdir(pathof(VMRobotControl))[1])[1:end-1])
robot = parseURDF(joinpath(module_path, "URDFs/franka_description/urdfs/fr3.urdf"), cfg)

add_gravity_compensation!(robot, VMRobotControl.DEFAULT_GRAVITY)
for i in 1:7
    add_coordinate!(robot, JointSubspace("fr3_joint$i"); id="J$i")
    add_component!(robot, LinearDamper(0.1, "J$i");      id="Joint damper $i")
end

# --- 2. SETUP DEL VIRTUAL MECHANISM SYSTEM E DEI TARGET ---

# NUOVO: Definiamo i target desiderati in modo chiaro
target_position = SVector(0.5, 0.3, 0.4)  # Target: [x, y, z] in metri
target_orientation = AxisAngle(SVector(0.0, 1.0, 0.0), π) # Target: rotazione di 90 gradi attorno all'asse X

vms = VirtualMechanismSystem("franka_reaching_task", robot)

# Definiamo le coordinate operative del robot (TCP)
add_coordinate!(robot, FramePoint("fr3_link8", SVector(0.0, 0.0, 0.0));     id="TCP position")
add_coordinate!(vms, QuaternionAttitude(".robot.fr3_link8", target_orientation); id="TCP orientation")

# Definiamo le coordinate del target e dell'errore
root = root_frame(vms.robot)
add_coordinate!(vms, FramePoint(".robot.$root", target_position);                id="Target position")
add_coordinate!(vms, CoordDifference(".robot.TCP position", "Target position");  id="Position error")

# --- 3. DEFINIZIONE DEI COMPONENTI VIRTUALI (CONTROLLORE) ---
# Rigidezza più alta per un posizionamento più preciso
K_pos = SMatrix{3, 3}(100.0, 0, 0, 0, 100.0, 0, 0, 0, 100.0) 
D_pos = SMatrix{3, 3}(45.0, 0, 0, 0, 45.0, 0, 0, 0, 45.0) # Smorzamento critico per evitare oscillazioni
add_component!(vms, LinearSpring(K_pos, "Position error"); id="Position Spring")
add_component!(vms, LinearDamper(D_pos, "Position error"); id="Position Damper")

# Componenti per l'orientamento
K_rot = SMatrix{3, 3}(5.0, 0, 0, 0, 5.0, 0, 0, 0, 5.0)
D_rot = SMatrix{3, 3}(5.0, 0, 0, 0, 5.0, 0, 0, 0, 5.0)
add_component!(vms, LinearSpring(K_rot, "TCP orientation"); id="Orientation Spring")
add_component!(vms, LinearDamper(D_rot, "TCP orientation"); id="Orientation Damper")

# --- 4. SIMULAZIONE ---
tspan = (0.0, 8.0) # Tempo di simulazione aumentato per dare tempo al robot di raggiungere il target
# Posizione iniziale del robot (q)
q_initial = ([0.0, -π/4, 0.0, -3*π/4, 0.0, π/2, π/4], Float64[])
q̇_initial = (zeros(7), Float64[]) # Parte da fermo
g = VMRobotControl.DEFAULT_GRAVITY

# Compilazione e creazione del problema ODE (senza disturbo esterno)
compiled_vms = compile(vms)
dcache = new_dynamics_cache(compiled_vms)
prob = get_ode_problem(dcache, g, q_initial, q̇_initial, tspan) # Rimosso f_control e f_setup

@info "Simulando il task di raggiungimento del Franka."
sol = solve(prob, Tsit5(); maxiters=1e5, abstol=1e-6, reltol=1e-6)

# --- 5. VISUALIZZAZIONE E ANIMAZIONE ---
fig = Figure(size = (800, 800))
display(fig)
ls = LScene(fig[1, 1]; show_axis=true)
cam = cam3d!(ls, camera=:perspective, center=false)

cam.lookat[] = [0.3, 0.0, 0.3]
cam.eyeposition[] = [1.2, 1.2, 1.0]

plotting_kcache = Observable(new_kinematics_cache(compile(robot)))
robotvisualize!(ls, plotting_kcache;)

# --- NUOVO & CORRETTO: Visualizzazione della POSA (Posizione + Orientazione) Target ---

# 1. Visualizza una sfera nel punto target
mesh!(ls, Sphere(Point3f(target_position), 0.05), color=(:red, 0.7))

# 2. Ruota i vettori degli assi base usando direttamente l'oggetto target_orientation
#    Questa è la maniera corretta e più semplice per ottenere il frame target.
target_frame_x = target_orientation * SVector(0.15, 0.0, 0.0) # Asse X del target
target_frame_y = target_orientation * SVector(0.0, 0.15, 0.0) # Asse Y del target
target_frame_z = target_orientation * SVector(0.0, 0.0, 0.15) # Asse Z del target

# 3. Disegna le frecce che rappresentano il frame target
origin_point = [Point3f(target_position)]
arrows!(ls, origin_point, [Vec3f(target_frame_x)], color=:red,   linewidth=0.02, arrowsize=0.03)
arrows!(ls, origin_point, [Vec3f(target_frame_y)], color=:green, linewidth=0.02, arrowsize=0.03)
arrows!(ls, origin_point, [Vec3f(target_frame_z)], color=:blue,  linewidth=0.02, arrowsize=0.03)

# Creazione dell'animazione
savepath = joinpath(@__DIR__, "franka_reaching_task.mp4")
animate_robot_odesolution(fig, sol, plotting_kcache, savepath);

@info "Animazione salvata in: $savepath"
