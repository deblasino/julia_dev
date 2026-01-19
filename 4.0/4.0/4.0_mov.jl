# =======================================================================================
# SIMULAZIONE VMC IBRIDA - SBARRA "SNAKE" A LUNGHEZZA VARIABILE CON OSTACOLI DINAMICI
# =======================================================================================

using DifferentialEquations
using GeometryBasics
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

# =======================================================================================
# --- 0. PARAMETRI DI SIMULAZIONE ---
# =======================================================================================

const PIVOT_POINT = SVector(0.6, 0.0, 0.1)

# --- POSIZIONI OSTACOLI ---
const OBSTACLE_POSITION_1 = SVector(0.3, 0.0, 0.4)  
const OBSTACLE_POSITION_2 = SVector(0.5, 0, 0.3) 
const OBSTACLE_POSITION_3 = SVector(1.6, -0.1, 0.1) 

const OBSTACLES_LIST = [OBSTACLE_POSITION_1, OBSTACLE_POSITION_2, OBSTACLE_POSITION_3]

# --- PARAMETRI MOVIMENTO OSTACOLI ---
const OBS_FREQ = 2.0  # Frequenza oscillazione (rad/s)
const OBS_AMP  = 0.15 # Ampiezza movimento (m)

const GRIP_BAR_LENGTH_HALF = 0.20
const GRIP_TRIANGLE_HEIGHT = 0.20

# --- PARAMETRI PER GENERAZIONE RICORSIVA ---
const MAX_SEGMENTS = 7           
const SEGMENT_LENGTH = 0.15      
const TARGET_TOLERANCE = 0.15    

global NUM_ACTUAL_SEGMENTS = MAX_SEGMENTS

# =======================================================================================
# --- 1. ROBOT MODEL CONFIGURATION ---
# =======================================================================================
println("1. Setup del robot...") 
cfg = URDFParserConfig(; suppress_warnings=true)
module_path = joinpath(splitpath(splitdir(pathof(VMRobotControl))[1])[1:end-1])
robot = parseURDF(joinpath(module_path, "URDFs/franka_description/urdfs/fr3_franka_hand(copy).urdf"), cfg)
add_gravity_compensation!(robot, VMRobotControl.DEFAULT_GRAVITY)

joint_limits = cfg.joint_limits

println("INFO: Applicazione Safety Springs e TanhDampers...")

for (i, τ_coulomb) in zip(1:7, [2.5, 2.5, 2.5, 2.5, 1.5, 1.5, 1.5])
    joint_name = "fr3_joint$i"
    coord_id = "J$i"
    add_coordinate!(robot, JointSubspace(joint_name); id=coord_id)
    limits = joint_limits[joint_name]
    if !isnothing(limits) && !isnothing(limits.lower) && !isnothing(limits.upper)
        add_deadzone_springs!(robot, 50.0, (limits.lower + 0.1, limits.upper - 0.1), coord_id)
    end
    β = 1e-1 
    add_component!(robot, TanhDamper(τ_coulomb, β, coord_id); id="JointDamper$i")
end

add_coordinate!(robot, FramePoint("fr3_hand_tcp", SVector(0.0, GRIP_BAR_LENGTH_HALF, 0.0)); id="P_grip_1")
add_coordinate!(robot, FramePoint("fr3_hand_tcp", SVector(0.0, -GRIP_BAR_LENGTH_HALF, 0.0)); id="P_grip_2")
add_coordinate!(robot, FramePoint("fr3_hand_tcp", SVector(GRIP_TRIANGLE_HEIGHT, 0.0, 0.0)); id="P_grip_3")
add_coordinate!(robot, FrameOrigin("fr3_hand_tcp"); id="TCP")

# =======================================================================================
# --- 1.5 PRE-CALCOLO TOPOLOGIA ---
# =======================================================================================
println("1.5. Generazione Topologia tramite Simulazione Fisica Iterativa...")

robot_compiled_temp = compile(robot)
kcache_temp = new_kinematics_cache(robot_compiled_temp)
q_robot_temp = [0.0, -π / 4, 0.0, -3 * π / 4, 0.0, π / 2, π / 4]
global n_dof_rob = config_size(robot_compiled_temp)
global q_robot_full = (n_dof_rob > 7 ? vcat(q_robot_temp, zeros(n_dof_rob - 7)) : q_robot_temp)
kinematics!(kcache_temp, 0.0, q_robot_full)

try
    tcp_id = get_compiled_coordID(robot_compiled_temp, "TCP")
    global P_EE_INITIAL = configuration(kcache_temp, tcp_id)
catch
    global P_EE_INITIAL = SVector(0.5, 0.0, 0.5)
end
println("   Target Attrattore: $P_EE_INITIAL")

function vector_to_yaw_pitch(v)
    v_norm = normalize(v)
    yaw = atan(v_norm[1], -v_norm[2]) 
    horiz = sqrt(v_norm[1]^2 + v_norm[2]^2)
    pitch = -atan(v_norm[3], horiz)
    return yaw, pitch 
end

vm_gen = Mechanism{Float64}("SnakeGenerator")
gen_pivot = add_frame!(vm_gen, "gen_pivot")
gen_ry = add_frame!(vm_gen, "gen_ry")
add_joint!(vm_gen, Rigid(Transform(PIVOT_POINT)); parent=root_frame(vm_gen), child=gen_pivot, id="Gen_Pivot_Joint")
add_joint!(vm_gen, Revolute(SVector(0., 0., 1.)); parent=gen_pivot, child=gen_ry, id="Gen_Base_Yaw")
add_coordinate!(vm_gen, FrameOrigin(gen_pivot); id="gen_pivot")
add_coordinate!(vm_gen, ConstCoord(P_EE_INITIAL); id="Gen_Target")

prev_frame = gen_ry
segment_frames = []
const K_ATTR_GEN = 50.0   
const K_REPEL_GEN = -20.0  
const DAMPING_MOVING = 5.0 
const DAMPING_FIXED = 100.0 
const SEG_MASS = 0.5 
const SEG_INERTIA_VAL = (1/12) * SEG_MASS * SEGMENT_LENGTH^2
const SEG_INERTIA_TENSOR = SMatrix{3,3}(SEG_INERTIA_VAL, 0, 0, 0, 1e-4, 0, 0, 0, SEG_INERTIA_VAL)

global q_prev = Float64[] 
global segment_vectors = SVector{3, Float64}[]
global segment_angles = Tuple{Float64, Float64}[]

for i in 1:MAX_SEGMENTS
    global prev_frame
    global q_prev 
    
    j_frame = add_frame!(vm_gen, "gen_j$(i)")
    s_frame = add_frame!(vm_gen, "gen_seg$(i)")
    push!(segment_frames, s_frame)
    
    axis = (i == 1) ? SVector(1., 0., 0.) : SVector(1., 0., 0.) 
    parent_j = (i == 1) ? prev_frame : add_frame!(vm_gen, "gen_int$(i)")
    
    if i > 1
        add_joint!(vm_gen, Revolute(SVector(0., 0., 1.)); parent=prev_frame, child=parent_j, id="Gen_J$(i)_Yaw")
    end
    
    add_joint!(vm_gen, Revolute(axis); parent=parent_j, child=j_frame, id="Gen_J$(i)_Pitch")
    add_joint!(vm_gen, Rigid(Transform(SVector(0.0, -SEGMENT_LENGTH, 0.0))); parent=j_frame, child=s_frame, id="Gen_Seg_Rigid$(i)")

    add_coordinate!(vm_gen, FrameAngularVelocity(j_frame); id="Gen_Vel$(i)")
    add_component!(vm_gen, Inertia(SEG_INERTIA_TENSOR, "Gen_Vel$(i)"); id="Gen_Inertia$(i)")
    add_component!(vm_gen, LinearDamper(DAMPING_MOVING * I(3), "Gen_Vel$(i)"); id="Gen_Damper$(i)")

    add_coordinate!(vm_gen, FrameOrigin(s_frame); id="Gen_Pos$(i)")
    
    for (k, obs) in enumerate(OBSTACLES_LIST)
        obs_coord_name = "Gen_Obs$(i)_$(k)"
        if !haskey(vm_gen.coordinates, obs_coord_name) 
             add_coordinate!(vm_gen, ConstCoord(obs); id=obs_coord_name)
        end
        add_coordinate!(vm_gen, CoordDifference("Gen_Pos$(i)", obs_coord_name); id="Gen_Err_Obs$(i)_$(k)")
        add_component!(vm_gen, GaussianSpring("Gen_Err_Obs$(i)_$(k)"; stiffness=K_REPEL_GEN, width=0.15); id="Gen_Repel$(i)_$(k)")
    end

    if i > 1
        if haskey(vm_gen.components, "Gen_Attract$(i-1)") delete!(vm_gen.components, "Gen_Attract$(i-1)") end
        if haskey(vm_gen.components, "Gen_Damper$(i-1)") delete!(vm_gen.components, "Gen_Damper$(i-1)") end
        add_component!(vm_gen, LinearDamper(DAMPING_FIXED * I(3), "Gen_Vel$(i-1)"); id="Gen_Damper$(i-1)")
    end
    
    add_coordinate!(vm_gen, CoordDifference("Gen_Pos$(i)", "Gen_Target"); id="Gen_Err_Attr$(i)")
    # --- FIX SINTASSI QUI: Parentesi chiusa prima di ; id= ---
    add_component!(vm_gen, LinearSpring(K_ATTR_GEN, "Gen_Err_Attr$(i)"); id="Gen_Attract$(i)")

    prev_frame = s_frame
    m_step = compile(vm_gen)
    
    local q_start
    if i == 1
        q_start = 1e-4 * ones(config_size(m_step))
    else
        n_prev = length(q_prev)
        n_curr = config_size(m_step)
        n_new = n_curr - n_prev
        q_start = [q_prev; zeros(n_new)]
    end

    println("   -> Ottimizzazione Segmento $i...")
    cache_step = new_dynamics_cache(m_step)
    dt_local = 5.0 
    prob_step = get_ode_problem(cache_step, VMRobotControl.DEFAULT_GRAVITY, q_start, zero_q̇(m_step), (0.0, dt_local))
    sol_step = solve(prob_step, Tsit5(); abstol=1e-4, reltol=1e-4)
    q_prev = sol_step.u[end][1:config_size(m_step)]
    
    k_vis = new_kinematics_cache(m_step)
    kinematics!(k_vis, 0.0, q_prev)
    
    id_curr = get_compiled_coordID(m_step, "Gen_Pos$(i)")
    id_prev = (i==1) ? get_compiled_coordID(m_step, "gen_pivot") : get_compiled_coordID(m_step, "Gen_Pos$(i-1)")
    
    pos_curr = configuration(k_vis, id_curr)
    pos_prev = (i==1) ? PIVOT_POINT : configuration(k_vis, id_prev)
    
    vec_res = pos_curr - pos_prev
    push!(segment_vectors, vec_res)
    
    y, p = vector_to_yaw_pitch(vec_res)
    push!(segment_angles, (y, p))

    dist_err = norm(pos_curr - P_EE_INITIAL)
    println("      Dist Target: $(round(dist_err, digits=3)) m")

    if dist_err < TARGET_TOLERANCE
        global NUM_ACTUAL_SEGMENTS = i
        println("   -> Target raggiunto ($i segmenti). Stop.")
        break
    end
end

global BAR_LENGTH_TOTAL = NUM_ACTUAL_SEGMENTS * SEGMENT_LENGTH

# =======================================================================================
# --- 2. VIRTUAL MECHANISM DEFINITION ---
# =======================================================================================
println("2. Creazione della 'Snake-Bar' dinamica estensibile ($NUM_ACTUAL_SEGMENTS segmenti)...")

segment_inertia_val = (1 / 12) * 1.0 * SEGMENT_LENGTH^2 
segment_inertia_tensor = SMatrix{3,3}(segment_inertia_val, 0, 0, 0, 1e-3, 0, 0, 0, segment_inertia_val)

vm = Mechanism{Float64}("VirtualSnakeRecursive")

pivot_frame = add_frame!(vm, "pivot_frame")
frame_rz = add_frame!(vm, "frame_rz")
frame_ry = add_frame!(vm, "frame_ry") 
add_joint!(vm, Rigid(Transform(PIVOT_POINT)); parent=root_frame(vm), child=pivot_frame, id="PivotAnchor") 
add_joint!(vm, Revolute(SVector(0., 0., 1.)); parent=pivot_frame, child=frame_rz, id="J_base_yaw")
add_joint!(vm, Revolute(SVector(0., 1., 0.)); parent=frame_rz, child=frame_ry, id="J_base_pitch")

add_coordinate!(vm, FrameOrigin(pivot_frame); id="Pivot_Pos")

global previous_frame = frame_ry
global seg_ids = String[]

for i in 1:NUM_ACTUAL_SEGMENTS
    joint_frame = add_frame!(vm, "joint$(i)_frame") 
    seg_frame = add_frame!(vm, "seg$(i)_frame")     
    
    push!(seg_ids, "seg$(i)_frame")
    prism_joint_id = "J_seg$(i)_length"

    if i == 1
        add_joint!(vm, Revolute(SVector(1., 0., 0.)); parent=previous_frame, child=joint_frame, id="J_roll_base")
        add_joint!(vm, Prismatic(SVector(0.0, -1.0, 0.0)); parent=joint_frame, child=seg_frame, id=prism_joint_id)
    else
        int_frame = add_frame!(vm, "int_frame_$(i)") 
        add_joint!(vm, Revolute(SVector(0., 0., 1.)); parent=previous_frame, child=int_frame, id="J_seg$(i)_yaw")
        add_joint!(vm, Revolute(SVector(1., 0., 0.)); parent=int_frame, child=joint_frame, id="J_seg$(i)_pitch")
        add_joint!(vm, Prismatic(SVector(0.0, -1.0, 0.0)); parent=joint_frame, child=seg_frame, id=prism_joint_id)
    end
    
    add_coordinate!(vm, FrameAngularVelocity(seg_frame); id="Seg$(i)_AngVel") 
    add_component!(vm, Inertia(segment_inertia_tensor, "Seg$(i)_AngVel"); id="Seg$(i)_Inertia")
    add_component!(vm, LinearDamper(0.5 * I(3), "Seg$(i)_AngVel"); id="Seg$(i)_RotDamper")
    
    prism_coord_id = "Seg$(i)_Len_Phys_Coord"
    add_coordinate!(vm, JointSubspace(prism_joint_id); id=prism_coord_id)
    add_component!(vm, LinearInerter(0.5, prism_coord_id); id="Seg$(i)_Len_Phys_Inertia") 
    add_component!(vm, LinearDamper(5.0, prism_coord_id); id="Seg$(i)_Len_Phys_Damper")

    add_coordinate!(vm, FrameOrigin(seg_frame); id="Seg$(i)_Pos")
    add_coordinate!(vm, FrameOrigin(joint_frame); id="Seg$(i)_Base_Pos")
    
    global previous_frame = seg_frame
end

slider_s_frame = add_frame!(vm, "slider_s_frame")
add_joint!(vm, Prismatic(SVector(1., 0., 0.)); parent=root_frame(vm), child=slider_s_frame, id="J_slider_progress")
add_coordinate!(vm, JointSubspace("J_slider_progress"); id="SliderCoord_s")
add_component!(vm, LinearInerter(0.1, "SliderCoord_s"); id="SliderProgressInertance")

initial_slider_pos_world = PIVOT_POINT - SVector(0.0, BAR_LENGTH_TOTAL, 0.0)
add_coordinate!(vm, ReferenceCoord(Ref(initial_slider_pos_world)); id="SliderWorldPos_Ref")
add_coordinate!(vm, ReferenceCoord(Ref(initial_slider_pos_world)); id="P_target_1_ref")
add_coordinate!(vm, ReferenceCoord(Ref(initial_slider_pos_world)); id="P_target_2_ref") 
add_coordinate!(vm, ReferenceCoord(Ref(initial_slider_pos_world)); id="P_target_3_ref")


# =======================================================================================
# --- 3. VMS ASSEMBLY ---
# =======================================================================================
println("3. Setup VMS con Ostacoli Dinamici...")
vms = VirtualMechanismSystem("SlidingControl", robot, vm)

add_coordinate!(vm, ReferenceCoord(Ref(OBSTACLE_POSITION_1)); id="ObstaclePosition1")
add_coordinate!(vm, ReferenceCoord(Ref(OBSTACLE_POSITION_2)); id="ObstaclePosition2")
add_coordinate!(vm, ConstCoord(OBSTACLE_POSITION_3); id="ObstaclePosition3")

ostacoli_vm = ["ObstaclePosition1", "ObstaclePosition2", "ObstaclePosition3"]

# 2. REPULSIONE SEGMENTI 
for i in 1:NUM_ACTUAL_SEGMENTS
    seg_pos_id = "Seg$(i)_Pos"
    for (j, ostacolo_id) in enumerate(ostacoli_vm)
        add_coordinate!(vms, CoordDifference(".virtual_mechanism.$(seg_pos_id)", ".virtual_mechanism.$(ostacolo_id)"); id="Seg$(i)_Err$(j)")
        add_component!(vms, GaussianSpring("Seg$(i)_Err$(j)"; stiffness=-80.0, width=0.08); id="Seg$(i)_Spr$(j)") 
    end
end

# 3. CONTROLLO LONGITUDINALE 
for i in 1:NUM_ACTUAL_SEGMENTS
    if !haskey(vm.coordinates, "Seg$(i)_Len_Q") 
         add_coordinate!(vm, JointSubspace("J_seg$(i)_length"); id="Seg$(i)_Len_Q")
    end
    add_coordinate!(vm, ConstCoord(SVector(SEGMENT_LENGTH)); id="Seg$(i)_Nominal_Len")
    add_coordinate!(vms, CoordDifference(".virtual_mechanism.Seg$(i)_Len_Q", ".virtual_mechanism.Seg$(i)_Nominal_Len"); id="Seg$(i)_Len_Err")
    add_component!(vms, LinearSpring(10.0, "Seg$(i)_Len_Err"); id="Seg$(i)_Len_Spring") 
    add_component!(vms, LinearDamper(5.0, "Seg$(i)_Len_Err"); id="Seg$(i)_Len_Damper")
end

# 4. REPULSIONE ROBOT 
links_robot = ["fr3_link3","fr3_link4", "fr3_link5", "fr3_link6", "fr3_link7", "fr3_link8", "fr3_hand"]
ostacoli_robot_cfg = [(id=1, c=".virtual_mechanism.ObstaclePosition1"), (id=2, c=".virtual_mechanism.ObstaclePosition2"), (id=3, c=".virtual_mechanism.ObstaclePosition3")]
for lnk in links_robot
    add_coordinate!(robot, FrameOrigin(lnk); id="$(lnk)_origin")
    for obs in ostacoli_robot_cfg
        add_coordinate!(vms, CoordDifference(".robot.$(lnk)_origin", obs.c); id="$(lnk)_ObsErr$(obs.id)")
        add_component!(vms, GaussianSpring("$(lnk)_ObsErr$(obs.id)"; stiffness=-350.0, width=0.11); id="$(lnk)_ObsSpr$(obs.id)")
    end
end

println("3.X. Applicazione vincolo rigido sulla punta della Snake-Bar...")
last_seg_frame_name = "seg$(NUM_ACTUAL_SEGMENTS)_frame"
add_coordinate!(vm, FrameOrigin(last_seg_frame_name); id="Snake_Tip_Pos")
add_coordinate!(vm, ConstCoord(P_EE_INITIAL); id="Snake_Tip_Anchor_Ref")
add_coordinate!(vms, CoordDifference(".virtual_mechanism.Snake_Tip_Pos", ".virtual_mechanism.Snake_Tip_Anchor_Ref"); id="Snake_Tip_Anchor_Error")
add_component!(vms, LinearSpring(5.0, "Snake_Tip_Anchor_Error"); id="Snake_Tip_Constraint_Spring")
add_component!(vms, LinearDamper(5.0, "Snake_Tip_Anchor_Error"); id="Snake_Tip_Constraint_Damper")

# 5. CONTROLLO SLIDER e FEEDBACK
add_coordinate!(vm, ConstCoord(SVector(0.0)); id="SliderTarget_s")
add_coordinate!(vms, CoordDifference(".virtual_mechanism.SliderCoord_s", ".virtual_mechanism.SliderTarget_s"); id="SliderDrivingError_s")
add_component!(vms, TanhSpring("SliderDrivingError_s"; max_force=15.0, stiffness=350.0); id="SliderDrivingSpring") 
add_component!(vms, LinearDamper(10.0, "SliderDrivingError_s"); id="SliderDrivingDamper")

add_coordinate!(vm, ReferenceCoord(Ref(SVector(0.0))); id="RobotProgress_s_Ref")
add_coordinate!(vms, CoordDifference(".virtual_mechanism.SliderCoord_s", ".virtual_mechanism.RobotProgress_s_Ref"); id="SliderFeedbackError_s")
add_component!(vms, TanhSpring("SliderFeedbackError_s"; max_force = 15.0, stiffness=350.0); id="SliderFeedbackSpring")
add_component!(vms, LinearDamper(10.0, "SliderFeedbackError_s"); id="SliderFeedbackDamper")

add_coordinate!(vms, CoordDifference(".virtual_mechanism.SliderWorldPos_Ref", ".robot.TCP"); id="PathError")
add_component!(vms, TanhSpring("PathError"; max_force=15.0, stiffness=350.0); id="PathSpring")
add_component!(vms, LinearDamper(10.0, "PathError"); id="PathDamper")

add_coordinate!(vms, CoordDifference(".virtual_mechanism.P_target_1_ref", ".robot.P_grip_2"); id="SErr1")
add_coordinate!(vms, CoordDifference(".virtual_mechanism.P_target_2_ref", ".robot.P_grip_1"); id="SErr2")
add_coordinate!(vms, CoordDifference(".virtual_mechanism.P_target_3_ref", ".robot.P_grip_3"); id="SErr3")

for i in 1:3
    add_component!(vms, TanhSpring("SErr$i"; max_force=5.0, stiffness=150.0); id="SSpr$i")
    add_component!(vms, LinearDamper(20.0, "SErr$i"); id="SDmp$i") 
end

# =======================================================================================
# --- 4. LOGICA DI CONTROLLO (MODIFICATA) ---
# =======================================================================================
println("4. Definizione f_control generalizzata con ostacoli mobili...")

function f_setup(cache)
    swp_ref = cache[get_compiled_coordID(cache, ".virtual_mechanism.SliderWorldPos_Ref")].coord_data.val
    p1_ref = cache[get_compiled_coordID(cache, ".virtual_mechanism.P_target_1_ref")].coord_data.val
    p2_ref = cache[get_compiled_coordID(cache, ".virtual_mechanism.P_target_2_ref")].coord_data.val
    p3_ref = cache[get_compiled_coordID(cache, ".virtual_mechanism.P_target_3_ref")].coord_data.val
    
    obs1_ref = cache[get_compiled_coordID(cache, ".virtual_mechanism.ObstaclePosition1")].coord_data.val
    obs2_ref = cache[get_compiled_coordID(cache, ".virtual_mechanism.ObstaclePosition2")].coord_data.val

    slider_s_id = get_compiled_coordID(cache, ".virtual_mechanism.SliderCoord_s")
    pivot_id = get_compiled_coordID(cache, ".virtual_mechanism.Pivot_Pos")
    seg_ids_compiled = [get_compiled_coordID(cache, ".virtual_mechanism.Seg$(i)_Pos") for i in 1:NUM_ACTUAL_SEGMENTS]
    rob_prog_ref = cache[get_compiled_coordID(cache, ".virtual_mechanism.RobotProgress_s_Ref")].coord_data.val
    tcp_id = get_compiled_coordID(cache, ".robot.TCP") 
    
    return (swp_ref, p1_ref, p2_ref, p3_ref, obs1_ref, obs2_ref, slider_s_id, pivot_id, seg_ids_compiled, rob_prog_ref, tcp_id)
end

function f_control(cache, t, args, extra)
    (swp_ref, p1_ref, p2_ref, p3_ref, obs1_ref, obs2_ref, slider_s_id, pivot_id, seg_ids_compiled, rob_prog_ref, tcp_id) = args

    # --- MODIFICA QUI: Movimento Ostacoli ---
    delta_val = OBS_AMP * sin(OBS_FREQ * t)
    
    # Ostacolo 1 (Movimento asse Y)
    obs_offset1 = SVector(0.0, delta_val, 0.0)
    obs1_ref[] = OBSTACLE_POSITION_1 + obs_offset1
    
    # Ostacolo 2 (Movimento asse X - MODIFICATO SU RICHIESTA)
    obs_offset2 = SVector(delta_val, 0.0, 0.0)
    obs2_ref[] = OBSTACLE_POSITION_2 - obs_offset2 
    # -------------------------------------------

    p_pivot = configuration(cache, pivot_id)
    p_segs = [configuration(cache, id) for id in seg_ids_compiled]
    points_path = pushfirst!(copy(p_segs), p_pivot) 

    p_robot = configuration(cache, tcp_id)
    best_dist_sq = Inf
    s_robot_projected = 0.0
    
    for k in 1:(length(points_path)-1)
        p_start = points_path[k]
        p_end = points_path[k+1]
        v_seg = p_end - p_start
        v_point = p_robot - p_start
        seg_len_sq = dot(v_seg, v_seg)
        if seg_len_sq > 1e-6
            t_proj = dot(v_point, v_seg) / seg_len_sq
            t_clamped = clamp(t_proj, 0.0, 1.0)
        else
            t_clamped = 0.0
        end
        closest_pt = p_start + v_seg * t_clamped
        diff = closest_pt - p_robot
        dist_sq = dot(diff, diff)
        if dist_sq < best_dist_sq
            best_dist_sq = dist_sq
            s_base = (k - 1) * SEGMENT_LENGTH
            s_robot_projected = s_base + t_clamped * SEGMENT_LENGTH
        end
    end
    rob_prog_ref[] = SVector(s_robot_projected)

    s_val = configuration(cache, slider_s_id)[1]
    s_clamped = clamp(s_val, 0.0, BAR_LENGTH_TOTAL) 
    
    seg_index = Int(floor(s_clamped / SEGMENT_LENGTH)) + 1
    if seg_index > NUM_ACTUAL_SEGMENTS seg_index = NUM_ACTUAL_SEGMENTS end
    s_local = s_clamped - (seg_index - 1) * SEGMENT_LENGTH
    ratio = s_local / SEGMENT_LENGTH
    
    p_start = points_path[seg_index]
    p_end = points_path[seg_index+1]
    pos_3D = p_start + (p_end - p_start) * ratio 

    swp_ref[] = pos_3D
    TARGET_SIDE = SVector(0., 1., 0.)
    TARGET_UP = SVector(1., 0., 0.)
    p1_ref[] = pos_3D + TARGET_SIDE * GRIP_BAR_LENGTH_HALF
    p2_ref[] = pos_3D - TARGET_SIDE * GRIP_BAR_LENGTH_HALF
    p3_ref[] = pos_3D + TARGET_UP * GRIP_TRIANGLE_HEIGHT
    
    return false
end

# =======================================================================================
# --- 4.5 CALCOLO STATO INIZIALE VM --- 
# =======================================================================================
println("4.5. Assemblaggio vettore stato iniziale (Inverse Kinematics Ricorsiva)...")

rot_z(θ) = @SMatrix [cos(θ) -sin(θ) 0; sin(θ) cos(θ) 0; 0 0 1]
rot_y(θ) = @SMatrix [cos(θ) 0 sin(θ); 0 1 0; -sin(θ) 0 cos(θ)]
rot_x(θ) = @SMatrix [1 0 0; 0 cos(θ) -sin(θ); 0 sin(θ) cos(θ)]

y1, p1 = segment_angles[1]
q_yaw_base = y1
q_pitch_base = 0.0  
q_roll_base  = p1   

q_vm_components = Float64[q_yaw_base, q_pitch_base, q_roll_base, SEGMENT_LENGTH]
R_prev = rot_z(q_yaw_base) * rot_y(q_pitch_base) * rot_x(q_roll_base)

for i in 2:NUM_ACTUAL_SEGMENTS
    v_target_global = segment_vectors[i]
    v_local = transpose(R_prev) * v_target_global 
    q_rel_yaw, q_rel_pitch = vector_to_yaw_pitch(v_local) 
    push!(q_vm_components, q_rel_yaw)
    push!(q_vm_components, q_rel_pitch)
    push!(q_vm_components, SEGMENT_LENGTH)
    R_curr = R_prev * rot_z(q_rel_yaw) * rot_x(q_rel_pitch)
    global R_prev = R_curr
end

push!(q_vm_components, BAR_LENGTH_TOTAL)
q_vm_initial = q_vm_components
println("   Stato iniziale VM calcolato ($length(q_vm_initial) DOF).")

# =======================================================================================
# --- 5. SIMULAZIONE ---
# =======================================================================================
println("5. Simulazione...")
tspan = (0.0, 15.0)
q_initial = (q_robot_full, q_vm_initial)
q̇_initial = (zeros(n_dof_rob), zeros(length(q_vm_initial)))
prob = get_ode_problem(new_dynamics_cache(compile(vms)), VMRobotControl.DEFAULT_GRAVITY, q_initial, q̇_initial, tspan; f_setup, f_control) 
sol = solve(prob, Tsit5(); maxiters=3e5, abstol=1e-6, reltol=1e-6)

# =======================================================================================
# --- 6. VISUALIZZAZIONE ---
# =======================================================================================
println("6. Visualizzazione...")

fig = Figure(size=(800, 800))
display(fig) 
ls = LScene(fig[1, 1]; show_axis=true)

cam = cam3d!(ls, camera=:perspective, center=false)
cam.lookat[] = [0.4, 0.1, 0.5]
cam.eyeposition[] = [1.7, 0.8, 1.0]

pkcache = Observable(new_kinematics_cache(compile(vms))) 
robotvisualize!(ls, pkcache)

mesh!(ls, Sphere(Point3f(PIVOT_POINT), 0.03), color=:green, label="Pivot")

# --- VISUALIZZAZIONE OSTACOLI MOBILI ---
obs1_id = get_compiled_coordID(compile(vms), ".virtual_mechanism.ObstaclePosition1")
obs2_id = get_compiled_coordID(compile(vms), ".virtual_mechanism.ObstaclePosition2")

obs1_pos_obs = map(c -> Point3f(configuration(c, obs1_id)), pkcache)
obs2_pos_obs = map(c -> Point3f(configuration(c, obs2_id)), pkcache)

obs1_geo = map(p -> Sphere(p, 0.04), obs1_pos_obs)
obs2_geo = map(p -> Sphere(p, 0.04), obs2_pos_obs)

mesh!(ls, obs1_geo, color=:orange, label="Ostacolo 1 (Mobile)")
mesh!(ls, obs2_geo, color=:magenta, label="Ostacolo 2 (Mobile)")

# Ostacolo 3 (Statico)
mesh!(ls, Sphere(Point3f(OBSTACLE_POSITION_3), 0.04), color=:cyan, label="Ostacolo 3 (Statico)")

# Visualizzazione Piano Z=0
mesh!(ls, Rect3f(Vec3f(-1.0, -1.0, -0.001), Vec3f(2.0, 2.0, 0.002)), color=(:blue, 0.1), transparency=true)

# --- Visualizzazione Snake Dinamica ---
points_obs = []
pivot_id = get_compiled_coordID(compile(vms), ".virtual_mechanism.Pivot_Pos")
push!(points_obs, map(c -> Point3f(configuration(c, pivot_id)), pkcache))

for i in 1:NUM_ACTUAL_SEGMENTS
    id = get_compiled_coordID(compile(vms), ".virtual_mechanism.Seg$(i)_Pos")
    push!(points_obs, map(c -> Point3f(configuration(c, id)), pkcache))
end

for i in 1:(length(points_obs)-1) 
    lines!(ls, map((p1, p2) -> [p1, p2], points_obs[i], points_obs[i+1]), color=:red, linewidth=5)
end

cursore_id = get_compiled_coordID(compile(vms), ".virtual_mechanism.SliderWorldPos_Ref")
scatter!(ls, map(c -> Point3f(configuration(c, cursore_id)), pkcache), color=:blue, markersize=20, label="Target (Cursore)")

println("   -> Generazione grafica deformazione telescopica...")

for i in 1:NUM_ACTUAL_SEGMENTS
    base_id = get_compiled_coordID(compile(vms), ".virtual_mechanism.Seg$(i)_Base_Pos")
    tip_id  = get_compiled_coordID(compile(vms), ".virtual_mechanism.Seg$(i)_Pos")
    
    p_base = map(c -> Point3f(configuration(c, base_id)), pkcache)
    p_tip  = map(c -> Point3f(configuration(c, tip_id)), pkcache)
    
    L_nom = Float32(SEGMENT_LENGTH)
    
    p_nominal_tip = map((b, t) -> begin
        vec = t - b
        dist = norm(vec)
        if dist < 1e-6
            return b + Point3f(0, -L_nom, 0)
        else
            dir = vec / dist
            return b + dir * L_nom 
        end
    end, p_base, p_tip)

    mesh!(ls, map((p_nom, p_real) -> Cylinder(p_nom, p_real, 0.02f0), p_nominal_tip, p_tip), 
          color=(:orange, 0.6), transparency=true)
    mesh!(ls, map((b, p_nom) -> Cylinder(b, p_nom, 0.01f0), p_base, p_nominal_tip),
          color=(:gray, 0.3), transparency=true)
end
axislegend(ls, position=:rt)

# --- Esecuzione e Registrazione ---
savepath = joinpath(@__DIR__, "franka_recursive_snake_dynamic_obst.mp4")
println("Rendering in corso su finestra e salvataggio su file...")

animate_robot_odesolution(fig, sol, pkcache, savepath; f_setup, f_control) 

@info "Animazione completata e salvata: $savepath"
