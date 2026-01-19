using DifferentialEquations
using GeometryBasics: Vec3f, Point3f
using GLMakie
using LinearAlgebra
using MeshIO
using StaticArrays
using VMRobotControl
using VMRobotControl.Splines: CubicSpline

# Necessario per caricare i mesh del robot Franka in formato .dae
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

# --- 2. DEFINIZIONE DELLA SBARRA E DEL MECCANISMO VIRTUALE ---
#Nel VMC, una sbarra o un percorso rettilineo è un giunto di tipo Rail basato su una CubicSpline  definita da due soli punti: l'inizio e la fine

#IN QUESTO CODICE SI SETTA IL TARGET E IL PUNTO DI INIZIO DELLA SBARRA, DOVE VIENE MESSO IL CARRELLO VIRTUALE 
target_position = SVector(0.5, 0.2, 0.6) #x e y sono scambiati nel grafico, 0.7 è la y
start_position_bar = target_position - SVector(0.0, -0.4, 0.0)
rail_length = norm(target_position - start_position_bar)

# Convertiamo il risultato di vcat in una Matrix mutabile standard
spline_knots = Matrix(vcat(start_position_bar', target_position'))
spline = CubicSpline(spline_knots)

vm = Mechanism{Float64}("SbarraVirtuale") # [cite: 94]
cart_frame = add_frame!(vm, "Cart") # [cite: 94]
add_joint!(vm, Rail(spline, zero(Transform{Float64})); 
        parent=root_frame(vm), child=cart_frame, id="RailJoint") # [cite: 94]

add_coordinate!(vm, FrameOrigin(cart_frame); id="CartPosition") # [cite: 94, 129]
add_coordinate!(vm, JointSubspace("RailJoint"); id="CartDistance") # [cite: 91, 94]
add_coordinate!(vm, ConstCoord(SVector(rail_length)); id="TargetDistance")

# Aggiungiamo "massa" e "attrito" al carrello virtuale per renderlo dinamicamente stabile
add_component!(vm, LinearInerter(1.0, "CartPosition");  id="CartInertance") # Massa virtuale
add_component!(vm, LinearDamper(10.0, "CartPosition"); id="CartDamper")    # Attrito virtuale
# --- FINE CORREZIONE ---

# --- 3. CREAZIONE DEL VMS E DEI COMPONENTI VIRTUALI ---
vms = VirtualMechanismSystem("ControlloSuSbarra", robot, vm) # [cite: 101]
add_coordinate!(robot, FramePoint("fr3_link8", SVector(0.0, 0.0, 0.0)); id="TCP")

# Definiamo i target desiderati di orientamento

#target_orientation = AxisAngle(SVector(0.0, 1.0, 0.0), π/2) # Rotazione di 180° attorno all'asse Y

# 1. Calcoliamo il vettore direzione della sbarra
bar_direction = normalize(target_position - start_position_bar)
# 2. Definiamo l'asse Z locale dell'end-effector
local_z_axis = SVector(0.0, 0.0, 1.0)
# 3. Calcoliamo asse e angolo di rotazione
rotation_axis = normalize(cross(local_z_axis, bar_direction))
rotation_angle = acos(dot(local_z_axis, bar_direction))
# 4. Creiamo l'oggetto rotazione
target_orientation = AxisAngle(rotation_axis, rotation_angle)

# Componenti per il vincolo di percorso (TCP <-> Carrello)
add_coordinate!(vms, CoordDifference(".virtual_mechanism.CartPosition", ".robot.TCP"); id="PathError") # [cite: 18, 101]
K_path = SMatrix{3, 3}(500.0, 0, 0, 0, 500.0, 0, 0, 0, 500.0) # Molla 3D rigida [cite: 19]
D_path = SMatrix{3, 3}(50.0, 0, 0, 0, 100.0, 0, 0, 0, 50.0)   # Smorzatore 3D critico [cite: 19]
add_component!(vms, LinearSpring(K_path, "PathError"); id="PathSpring") # [cite: 19]
add_component!(vms, LinearDamper(D_path, "PathError"); id="PathDamper") # [cite: 19]

# --- NUOVO: Componenti per il VINCOLO DI ORIENTAMENTO ---
add_coordinate!(vms, QuaternionAttitude(".robot.fr3_link8", target_orientation); id="OrientationError")
K_rot = SMatrix{3, 3}(50.0, 0, 0, 0, 50.0, 0, 0, 0, 50.0)  # Molla rotazionale
D_rot = SMatrix{3, 3}(10.0, 0, 0, 0, 10.0, 0, 0, 0, 10.0)  # Smorzatore rotazionale
add_component!(vms, LinearSpring(K_rot, "OrientationError"); id="OrientationSpring")
add_component!(vms, LinearDamper(D_rot, "OrientationError"); id="OrientationDamper")

# Componenti per l'avanzamento (guidare il carrello lungo la sbarra)
add_coordinate!(vms, CoordDifference(".virtual_mechanism.CartDistance", ".virtual_mechanism.TargetDistance"); id="DrivingError")
K_drive = 100.0 # Molla 1D
D_drive = 80.0  # Smorzatore 1D
add_component!(vms, LinearSpring(K_drive, "DrivingError"); id="DrivingSpring")
add_component!(vms, LinearDamper(D_drive, "DrivingError"); id="DrivingDamper")

# --- 4. SIMULAZIONE ---
tspan = (0.0, 10.0)
q_initial = ([0.0, -π/4, 0.0, -3*π/4, 0.0, π/2, π/4], [0.0]) # Robot + VM state
q̇_initial = (zeros(7), zeros(1))

g = VMRobotControl.DEFAULT_GRAVITY
compiled_vms = compile(vms)
dcache = new_dynamics_cache(compiled_vms)
prob = get_ode_problem(dcache, g, q_initial, q̇_initial, tspan) # [cite: 21, 49, 107, 131]
@info "Simulando il controllo su sbarra virtuale..."
sol = solve(prob, Tsit5(); maxiters=1e5, abstol=1e-6, reltol=1e-6) # [cite: 22, 28, 52, 110, 131]

# --- 5. VISUALIZZAZIONE ---
fig = Figure(size = (800, 800))
display(fig)
ls = LScene(fig[1, 1]; show_axis=true)
cam = cam3d!(ls, camera=:perspective, center=false)
cam.lookat[] = [0.3, 0.0, 0.3]
cam.eyeposition[] = [1.0, 0.0, 1.5]

plotting_kcache = Observable(new_kinematics_cache(compiled_vms))
plotting_vm_kcache = map(c -> VMRobotControl.virtual_mechanism_cache(c), plotting_kcache)

# Visualizza il robot e la sbarra virtuale
robotvisualize!(ls, plotting_kcache) # [cite: 57]
robotsketch!(ls, plotting_vm_kcache; linecolor=:red, linewidth=5) # [cite: 116, 140]

# Visualizza il target e il carrello virtuale
mesh!(ls, Sphere(Point3f(target_position), 0.03), color=(:red, 0.9))
cartID = get_compiled_coordID(compiled_vms, ".virtual_mechanism.CartPosition")
scatter!(ls, plotting_kcache, cartID; color=:magenta, markersize=15) # [cite: 58, 116]

# Creazione dell'animazione
savepath = joinpath(@__DIR__, "franka_rail_control.mp4")
animate_robot_odesolution(fig, sol, plotting_kcache, savepath)
@info "Animazione salvata in: $savepath"