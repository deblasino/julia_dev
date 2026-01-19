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
cfg = URDFParserConfig(;suppress_warnings=true) # This is just to hide warnings about unsupported URDF features
module_path = joinpath(splitpath(splitdir(pathof(VMRobotControl))[1])[1:end-1])
robot = parseURDF(joinpath(module_path, "URDFs/franka_description/urdfs/fr3.urdf"), cfg)

add_gravity_compensation!(robot, VMRobotControl.DEFAULT_GRAVITY)
for i in 1:7
    add_coordinate!(robot, JointSubspace("fr3_joint$i");    id="J$i")
    add_component!(robot, LinearDamper(0.1, "J$i");         id="Joint damper $i")
end;

target_rot = AxisAngle(SVector(0., 1., 0.), Float64(π))
vms = VirtualMechanismSystem("franka_impedance_control", robot)
add_coordinate!(robot, FramePoint("fr3_link8", SVector(0., 0., 0.));            id="TCP position")
add_coordinate!(vms, QuaternionAttitude(".robot.fr3_link8", target_rot);        id="TCP orientation")
root = root_frame(vms.robot)
add_coordinate!(vms, FramePoint(".robot.$root", SVector(0.5, 0.3, 0.4));        id="Target position")
add_coordinate!(vms, CoordDifference(".robot.TCP position", "Target position"); id="Position error");


K = SMatrix{3, 3}(100., 0., 0., 0., 200., 0., 0., 0., 300.)
add_component!(vms, LinearSpring(K, "Position error");           id="Linear Spring")
D = SMatrix{3, 3}(10., 0., 0., 0., 20.0, 0., 0., 0., 30.)
add_component!(vms, LinearDamper(D, "Position error");           id="Linear Damper")
K_rot = SMatrix{3, 3}(10., 0., 0., 0., 20., 0., 0., 0., 30.)
add_component!(vms, LinearSpring(K_rot, "TCP orientation");       id="Angular Spring")
D_rot = SMatrix{3, 3}(0.1, 0., 0., 0., 0.2, 0., 0., 0., 0.3)
add_component!(vms, LinearDamper(D_rot, "TCP orientation");       id="Angular Damper");



disturbance_func(t) = mod(t, 6) < 3 ? SVector(0., 0., 0.) : SVector(0., 10.0, 0.)
f_setup(cache) = get_compiled_coordID(cache, ".robot.TCP position")
function f_control(cache, t, args, extra)
    tcp_pos_coord_id = args
    F = disturbance_func(t)
    uᵣ, uᵥ = get_u(cache)
    z = configuration(cache, tcp_pos_coord_id)
    J = jacobian(cache, tcp_pos_coord_id)
    mul!(uᵣ, J', F)
    nothing
end
tspan = (0., 12.)
q = ([0.0, 0.3, 0.0, -1.8, 0.0, π/2, 0.0], Float64[]) # Robot joint angle, vm joint angles
q̇ = ([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], Float64[]) # Robot joint velocity, vm joint velocities
g = VMRobotControl.DEFAULT_GRAVITY
dcache = new_dynamics_cache(compile(vms))
prob = get_ode_problem(dcache, g, q, q̇, tspan; f_setup, f_control)
@info "Simulating franka robot with impedance control."
sol = solve(prob, Tsit5(); maxiters=1e5, abstol=1e-6, reltol=1e-6);

fig = Figure(size = (720, 720), figure_padding=0)
display(fig)
ls = LScene(fig[1, 1]; show_axis=false)
cam = cam3d!(ls, camera=:perspective, center=false)
cam.lookat[] = [0.24015087703685004, -0.02303109659518269, 0.37391966173978597]
cam.eyeposition[] = [0.5360994756635347, 0.7578567808643422, 0.5303480879908374]

plotting_t = Observable(0.0)
plotting_kcache = Observable(new_kinematics_cache(compile(robot)))
robotvisualize!(ls, plotting_kcache;)

tcp_pos_id = get_compiled_coordID(plotting_kcache[], "TCP position")
tcp_pos = map(plotting_kcache) do kcache
    Point3f(configuration(kcache, tcp_pos_id))
end
force = map(t -> 0.01 * Vec3f(disturbance_func(t)), plotting_t)
arrowsize = map(f -> 0.1*(f'*f)^(0.25), force)
arrows!(ls, map(p -> [p], tcp_pos), map(f -> [f], force); color = :red, arrowsize)


savepath = joinpath(module_path, "docs/src/assets/franka_impedance_control.mp4")
animate_robot_odesolution(fig, sol, plotting_kcache, savepath; t=plotting_t);