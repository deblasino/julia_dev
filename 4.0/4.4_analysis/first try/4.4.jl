# =======================================================================================
# SIMULAZIONE VMC IBRIDA - SNAKE RICORSIVO ADATTIVO
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

# --- Geometria Robot ---
const GRIP_BAR_LENGTH_HALF = 0.20
const GRIP_TRIANGLE_HEIGHT = 0.20
const PIVOT_POINT = SVector(0.6, 0.0, 0.0)

# --- Ostacoli ---
#const OBSTACLE_POSITION_1 = SVector(0.3, 0.1, 0.4)  
#const OBSTACLE_POSITION_2 = SVector(0.6, -0.1, 0.1)


const OBSTACLE_POSITION_1 = SVector(0.4, 0.1, 0.1)  
const OBSTACLE_POSITION_2 = SVector(0.5,-0.1, 0.4)

#const OBSTACLE_POSITION_1 = SVector(0.3, 1.9715653261613723e-16, 0.4)  
#const OBSTACLE_POSITION_2 = SVector(0.5, 1.9715653261613723e-16, 0.3) 

const OBSTACLES_LIST = [OBSTACLE_POSITION_1, OBSTACLE_POSITION_2]

# --- PARAMETRI SNAKE DINAMICI ---
const MAX_SEGMENTS = 10         # Limite di sicurezza per evitare loop infiniti
const L_SEGMENT_DEFAULT = 0.05  # Lunghezza nominale segmento
# La soglia di arrivo è uguale alla lunghezza del segmento, come richiesto
const ARRIVAL_THRESHOLD = L_SEGMENT_DEFAULT 

# Parametri Molle/Smorzatori
const K_TELESCOPIC = 80.0       
const D_TELESCOPIC = 5.0        
const K_ATTR_TARGET = 80.0      
const K_REPEL_OBS = -70.0       
const W_REPEL_OBS = 0.1       

# =======================================================================================
# --- 1. SETUP ROBOT (Invariato) ---
# =======================================================================================
println("1. Setup del robot...") 
cfg = URDFParserConfig(; suppress_warnings=true)
module_path = joinpath(splitpath(splitdir(pathof(VMRobotControl))[1])[1:end-1])
robot = parseURDF(joinpath(module_path, "URDFs/franka_description/urdfs/fr3_franka_hand(copy).urdf"), cfg)
add_gravity_compensation!(robot, VMRobotControl.DEFAULT_GRAVITY)

# Setup Safety Springs e Dampers
joint_limits = cfg.joint_limits
for (i, τ_coulomb) in zip(1:7, [0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5])
    joint_name = "fr3_joint$i"
    coord_id = "J$i"
    add_coordinate!(robot, JointSubspace(joint_name); id=coord_id)
    limits = joint_limits[joint_name]
    if !isnothing(limits) && !isnothing(limits.lower) && !isnothing(limits.upper)
        add_deadzone_springs!(robot, 50.0, (limits.lower + 0.1, limits.upper - 0.1), coord_id)
    end
    add_component!(robot, TanhDamper(τ_coulomb, 1e-1, coord_id); id="JointDamper$i")
end

add_coordinate!(robot, FramePoint("fr3_hand_tcp", SVector(0.0, GRIP_BAR_LENGTH_HALF, 0.0)); id="P_grip_1")
add_coordinate!(robot, FramePoint("fr3_hand_tcp", SVector(0.0, -GRIP_BAR_LENGTH_HALF, 0.0)); id="P_grip_2")
add_coordinate!(robot, FramePoint("fr3_hand_tcp", SVector(GRIP_TRIANGLE_HEIGHT, 0.0, 0.0)); id="P_grip_3")
add_coordinate!(robot, FrameOrigin("fr3_hand_tcp"); id="TCP")

# =======================================================================================
# --- 1.5 GENERAZIONE TOPOLOGIA RICORSIVA ---
# =======================================================================================
println("1.5. Generazione Topologia Ricorsiva...")

# --- A. Calcolo Target Iniziale ---
robot_compiled_temp = compile(robot)
kcache_temp = new_kinematics_cache(robot_compiled_temp)
q_robot_temp = [0.0, -π / 4, 0.0, -3 * π / 4, 0.0, π / 2, π / 4]
n_dof_rob = config_size(robot_compiled_temp)
global q_robot_full = (n_dof_rob > 7 ? vcat(q_robot_temp, zeros(n_dof_rob - 7)) : q_robot_temp)
kinematics!(kcache_temp, 0.0, q_robot_full)

try
    tcp_id = get_compiled_coordID(robot_compiled_temp, "TCP")
    global P_EE_INITIAL = configuration(kcache_temp, tcp_id)
catch
    global P_EE_INITIAL = SVector(0.5, 0.0, 0.5)
end
println("   Target (EE Robot): $P_EE_INITIAL")

# --- B. Inizializzazione Generatore ---
vm_gen = Mechanism{Float64}("SnakeGenerator")
gen_pivot = add_frame!(vm_gen, "gen_pivot")
gen_ry = add_frame!(vm_gen, "gen_ry")

add_joint!(vm_gen, Rigid(Transform(PIVOT_POINT)); parent=root_frame(vm_gen), child=gen_pivot, id="Gen_Pivot_Joint")
add_joint!(vm_gen, Revolute(SVector(0., 0., 1.)); parent=gen_pivot, child=gen_ry, id="Gen_Base_Yaw")

add_coordinate!(vm_gen, FrameOrigin(gen_pivot); id="gen_pivot")
add_coordinate!(vm_gen, ConstCoord(P_EE_INITIAL); id="Gen_Target")

# Variabili Loop
global prev_frame = gen_ry
global q_prev = Float64[] 
global gen_results_lengths = Float64[]
global gen_results_angles = Tuple{Float64, Float64}[]
global num_generated_segments = 0
global current_tip_pos = PIVOT_POINT

# Parametri Inerziali
const SEG_MASS = 0.5
const SEG_INERTIA_VAL = (1/12) * SEG_MASS * L_SEGMENT_DEFAULT^2
const SEG_INERTIA_TENSOR = SMatrix{3,3}(SEG_INERTIA_VAL, 0, 0, 0, 1e-4, 0, 0, 0, SEG_INERTIA_VAL)

# --- CICLO WHILE DI GENERAZIONE ---
reached_target = false

t_generation_start = Base.time()

while !reached_target && num_generated_segments < MAX_SEGMENTS
    global num_generated_segments += 1
    global reached_target
    i = num_generated_segments
    global prev_frame
    global q_prev
    global current_tip_pos
    
    # 1. Calcolo distanza attuale
    dist_to_target = norm(P_EE_INITIAL - current_tip_pos)
    println("   -> Generazione segmento $i. Distanza attuale: $(round(dist_to_target, digits=3)) m")
    
    # Crea Frames
    joint_frame = add_frame!(vm_gen, "gen_j$(i)")
    s_frame = add_frame!(vm_gen, "gen_seg$(i)")
    
    # 2. Giunti Rotazionali
    parent_j = (i == 1) ? prev_frame : add_frame!(vm_gen, "gen_int$(i)")
    
    if i > 1
        add_joint!(vm_gen, Revolute(SVector(0., 0., 1.)); parent=prev_frame, child=parent_j, id="Gen_J$(i)_Yaw")
    end
    
    axis_p = SVector(1., 0., 0.)
    add_joint!(vm_gen, Revolute(axis_p); parent=parent_j, child=joint_frame, id="Gen_J$(i)_Pitch")

    # 3. Giunto Prismatico (Estensione)
    prism_id = "Gen_Prism$(i)"
    add_joint!(vm_gen, Prismatic(SVector(0.0, 0.0, 1.0)); parent=joint_frame, child=s_frame, id=prism_id) 

    # 4. Fisica Molla Telescopica
    prism_coord_id = "Gen_Len_Phys_Coord$(i)"
    add_coordinate!(vm_gen, JointSubspace(prism_id); id=prism_coord_id)
    add_component!(vm_gen, LinearInerter(SEG_MASS, prism_coord_id); id="Gen_Len_Inertia$(i)") 
    add_component!(vm_gen, LinearDamper(D_TELESCOPIC * 2.0, prism_coord_id); id="Gen_Len_Phys_Damp$(i)")

    add_coordinate!(vm_gen, ConstCoord(SVector(L_SEGMENT_DEFAULT)); id="Gen_Nominal_Len$(i)")
    add_coordinate!(vm_gen, CoordDifference(prism_coord_id, "Gen_Nominal_Len$(i)"); id="Gen_Len_Err$(i)")
    add_component!(vm_gen, LinearSpring(K_TELESCOPIC, "Gen_Len_Err$(i)"); id="Gen_Len_Spring$(i)")
    
    # 5. Inerzia Rotazionale
    add_coordinate!(vm_gen, FrameAngularVelocity(joint_frame); id="Gen_Vel$(i)")
    add_component!(vm_gen, Inertia(SEG_INERTIA_TENSOR, "Gen_Vel$(i)"); id="Gen_Inertia$(i)")
    add_component!(vm_gen, LinearDamper(0.1 * I(3), "Gen_Vel$(i)"); id="Gen_RotDamp$(i)")

    # 6. Posizione Punta Segmento Corrente
    add_coordinate!(vm_gen, FrameOrigin(s_frame); id="Gen_Pos$(i)")

    # 7. Repulsione Ostacoli
    for (k, obs) in enumerate(OBSTACLES_LIST)
        obs_name = "Gen_Obs$(i)_$(k)"
        if !haskey(vm_gen.coordinates, obs_name)
            add_coordinate!(vm_gen, ConstCoord(obs); id=obs_name)
        end
        add_coordinate!(vm_gen, CoordDifference("Gen_Pos$(i)", obs_name); id="Gen_RepelErr$(i)_$(k)")
        add_component!(vm_gen, GaussianSpring("Gen_RepelErr$(i)_$(k)"; stiffness=K_REPEL_OBS, width=W_REPEL_OBS); id="Gen_Repel$(i)_$(k)")
    end

    # 8. Attrazione al Target (Sposta l'attrattore sull'ultimo segmento aggiunto)
    if i > 1
        if haskey(vm_gen.components, "Gen_Attract$(i-1)")
            delete!(vm_gen.components, "Gen_Attract$(i-1)")
        end
    end
    add_coordinate!(vm_gen, CoordDifference("Gen_Pos$(i)", "Gen_Target"); id="Gen_AttrErr$(i)")
    add_component!(vm_gen, LinearSpring(K_ATTR_TARGET, "Gen_AttrErr$(i)"); id="Gen_Attract$(i)")
    
    prev_frame = s_frame

    # --- SIMULAZIONE INCREMENTALE ---
    m_step = compile(vm_gen)
    
    local q_start
    if i == 1
        q_start = [0.0, 0.0, L_SEGMENT_DEFAULT] 
    else
        q_start = [q_prev; 0.0; 0.0; L_SEGMENT_DEFAULT]
    end
    
    cache = new_dynamics_cache(m_step)
    # Eseguiamo una simulazione breve per trovare l'equilibrio
    prob = get_ode_problem(cache, VMRobotControl.DEFAULT_GRAVITY, q_start, zero_q̇(m_step), (0.0, 5.0))
    sol = solve(prob, Tsit5(); abstol=1e-5, reltol=1e-5)
    
    q_prev = sol.u[end][1:config_size(m_step)]
    
    # Recuperiamo la lunghezza attuale dell'ultimo segmento generato
    # Nel vettore di stato q, l'ultima variabile è proprio l'estensione del giunto prismatico
    current_segment_length = q_prev[end]
    
    # Calcoliamo la distanza residua
    k_check = new_kinematics_cache(m_step)
    kinematics!(k_check, 0.0, q_prev)
    tip_id_compiled = get_compiled_coordID(m_step, "Gen_Pos$(i)")
    current_tip_pos = configuration(k_check, tip_id_compiled)
    dist_after_solve = norm(P_EE_INITIAL - current_tip_pos)
    
   
    # Ci fermiamo solo se:
    # 1. Siamo vicini al target
    # 2. E il segmento non è "stirato" oltre il 10% della sua lunghezza nominale
   
    #Da cambiare in base alla lunghezza dei segmenti, per csegmenti piccoli tolleranza alta e viceversa
    tolleranza_estensione = L_SEGMENT_DEFAULT * 1.3 # Max 10% di allungamento accettabile
    
    if dist_after_solve <= ARRIVAL_THRESHOLD && current_segment_length <= tolleranza_estensione
        println("   Target raggiunto stabilmente. Stop.")
        reached_target = true
    else
        println("   Target toccato ma segmento troppo esteso ($current_segment_length m). Aggiungo un altro segmento.")
        reached_target = false # Forziamo il ciclo a continuare
    end

end

if !reached_target
    println("   *** Raggiunto numero massimo segmenti ($MAX_SEGMENTS). ***")
end

t_generation_elapsed = Base.time() - t_generation_start
println("   Generazione completata in $(round(t_generation_elapsed, digits=3)) secondi.")
println("   Estrazione configurazione...")

# Estrazione Risultati
println("   Estrazione configurazione finale...")
global idx = 1
push!(gen_results_angles, (q_prev[idx], q_prev[idx+1])) # Base (Pitch, Len) no, Base ha (Yaw, Pitch, Len)
# Attenzione: Base ha Pivot(Rigid)->BaseYaw(Rev)->BasePitch(Rev)->Prismatic
# Indici: 1:Yaw, 2:Pitch, 3:Len
push!(gen_results_lengths, q_prev[idx+2])
global idx += 3

for i in 2:num_generated_segments
    global idx
    push!(gen_results_angles, (q_prev[idx], q_prev[idx+1]))
    push!(gen_results_lengths, q_prev[idx+2])
    idx += 3
end

global BAR_LENGTH_TOTAL = sum(gen_results_lengths)
println("   Numero Segmenti Totali: $num_generated_segments")
println("   Lunghezza Totale Percorso: $(round(BAR_LENGTH_TOTAL, digits=3)) m")


# =======================================================================================
# --- 2. VMS ASSEMBLY (Dinamico basato su num_generated_segments) ---
# =======================================================================================
println("2. Setup VMS Finale...")

vm = Mechanism{Float64}("VirtualSnake")

pivot_frame = add_frame!(vm, "pivot_frame")
frame_rz = add_frame!(vm, "frame_rz")
frame_ry = add_frame!(vm, "frame_ry") 
add_joint!(vm, Rigid(Transform(PIVOT_POINT)); parent=root_frame(vm), child=pivot_frame, id="PivotAnchor") 
add_joint!(vm, Revolute(SVector(0., 0., 1.)); parent=pivot_frame, child=frame_rz, id="J_base_yaw")
add_joint!(vm, Revolute(SVector(0., 1., 0.)); parent=frame_rz, child=frame_ry, id="J_base_pitch")
add_coordinate!(vm, FrameOrigin(pivot_frame); id="Pivot_Pos")

global previous_frame = frame_ry

# Costruzione Catena nel VMS
for i in 1:num_generated_segments
    j_frame = add_frame!(vm, "joint$(i)_frame")
    s_frame = add_frame!(vm, "seg$(i)_frame")
    
    prism_id = "J_seg$(i)_length"
    
    if i == 1
        add_joint!(vm, Revolute(SVector(1., 0., 0.)); parent=previous_frame, child=j_frame, id="J_roll_base")
    else
        int_frame = add_frame!(vm, "int_frame_$(i)")
        add_joint!(vm, Revolute(SVector(0., 0., 1.)); parent=previous_frame, child=int_frame, id="J_seg$(i)_yaw")
        add_joint!(vm, Revolute(SVector(1., 0., 0.)); parent=int_frame, child=j_frame, id="J_seg$(i)_pitch")
    end
    
    add_joint!(vm, Prismatic(SVector(0.0, 0.0, 1.0)); parent=j_frame, child=s_frame, id=prism_id)
    
    add_coordinate!(vm, FrameAngularVelocity(s_frame); id="Seg$(i)_AngVel") 
    add_component!(vm, Inertia(SEG_INERTIA_TENSOR, "Seg$(i)_AngVel"); id="Seg$(i)_Inertia")
    add_component!(vm, LinearDamper(0.1 * I(3), "Seg$(i)_AngVel"); id="Seg$(i)_RotDamper")
    
    phys_coord_id = "Seg$(i)_Len_Phys_Coord"
    add_coordinate!(vm, JointSubspace(prism_id); id=phys_coord_id)
    add_component!(vm, LinearInerter(0.5, phys_coord_id); id="Seg$(i)_Len_Phys_Inertia")
    add_component!(vm, LinearDamper(D_TELESCOPIC * 2.0, phys_coord_id); id="Seg$(i)_Len_Phys_Damp")
    
    add_coordinate!(vm, JointSubspace(prism_id); id="Seg$(i)_Len_Q")
    add_coordinate!(vm, ConstCoord(SVector(L_SEGMENT_DEFAULT)); id="Seg$(i)_Nominal_Len")
    
    add_coordinate!(vm, FrameOrigin(s_frame); id="Seg$(i)_Pos")
    add_coordinate!(vm, FrameOrigin(j_frame); id="Seg$(i)_Base_Pos") 
    
    global previous_frame = s_frame
end

# Slider e Target (Invariati)
slider_s_frame = add_frame!(vm, "slider_s_frame")
add_joint!(vm, Prismatic(SVector(1., 0., 0.)); parent=root_frame(vm), child=slider_s_frame, id="J_slider_progress")
add_coordinate!(vm, JointSubspace("J_slider_progress"); id="SliderCoord_s")
add_component!(vm, LinearInerter(0.1, "SliderCoord_s"); id="SliderProgressInertance")

add_coordinate!(vm, ReferenceCoord(Ref(P_EE_INITIAL)); id="SliderWorldPos_Ref")
add_coordinate!(vm, ReferenceCoord(Ref(P_EE_INITIAL)); id="P_target_1_ref")
add_coordinate!(vm, ReferenceCoord(Ref(P_EE_INITIAL)); id="P_target_2_ref") 
add_coordinate!(vm, ReferenceCoord(Ref(P_EE_INITIAL)); id="P_target_3_ref")

# =======================================================================================
# --- 3. VMS COMPONENTS ASSEMBLY ---
# =======================================================================================
vms = VirtualMechanismSystem("SlidingControl", robot, vm)

# 1. Molle Telescopiche
for i in 1:num_generated_segments
    add_coordinate!(vms, CoordDifference(".virtual_mechanism.Seg$(i)_Len_Q", ".virtual_mechanism.Seg$(i)_Nominal_Len"); id="Seg$(i)_Len_Err")
    add_component!(vms, LinearSpring(K_TELESCOPIC, "Seg$(i)_Len_Err"); id="Seg$(i)_Len_Spring")
    add_component!(vms, LinearDamper(D_TELESCOPIC, "Seg$(i)_Len_Err"); id="Seg$(i)_Len_Damper")
end

# 2. Repulsione Ostacoli
for (k, obs) in enumerate(OBSTACLES_LIST)
    obs_name = "ObstaclePosition$(k)"
    add_coordinate!(vm, ConstCoord(obs); id=obs_name)
    
    for i in 1:num_generated_segments
        seg_pos_id = "Seg$(i)_Pos"
        err_id = "Seg$(i)_Err$(k)"
        add_coordinate!(vms, CoordDifference(".virtual_mechanism.$(seg_pos_id)", ".virtual_mechanism.$(obs_name)"); id=err_id)
        add_component!(vms, GaussianSpring(err_id; stiffness=K_REPEL_OBS, width=W_REPEL_OBS); id="Seg$(i)_Spr$(k)") 
    end
end

# 3. Attrazione al Target (ancora la punta)
last_seg_pos_id = "Seg$(num_generated_segments)_Pos"
add_coordinate!(vm, ConstCoord(P_EE_INITIAL); id="Snake_Tip_Target_Fixed")
add_coordinate!(vms, CoordDifference(".virtual_mechanism.$(last_seg_pos_id)", ".virtual_mechanism.Snake_Tip_Target_Fixed"); id="Snake_Tip_Ref_Err")
add_component!(vms, LinearSpring(K_ATTR_TARGET, "Snake_Tip_Ref_Err"); id="Snake_Tip_Anchor_Spring")
add_component!(vms, LinearDamper(10.0, "Snake_Tip_Ref_Err"); id="Snake_Tip_Anchor_Damp")

# --- INTERAZIONE ROBOT (Invariato) ---
links_robot = ["fr3_link4", "fr3_link5", "fr3_link6", "fr3_link7", "fr3_link8", "fr3_hand"]
for lnk in links_robot
    add_coordinate!(robot, FrameOrigin(lnk); id="$(lnk)_origin")
    for (k, obs) in enumerate(OBSTACLES_LIST)
        obs_name = "ObstaclePosition$(k)"
        add_coordinate!(vms, CoordDifference(".robot.$(lnk)_origin", ".virtual_mechanism.$(obs_name)"); id="$(lnk)_ObsErr$(k)")
        add_component!(vms, GaussianSpring("$(lnk)_ObsErr$(k)"; stiffness=-350.0, width=0.08); id="$(lnk)_ObsSpr$(k)")
    end
end

# Controllo Slider e Path (Standard)
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

# Orientamento
add_coordinate!(vms, CoordDifference(".virtual_mechanism.P_target_1_ref", ".robot.P_grip_2"); id="SErr1")
add_coordinate!(vms, CoordDifference(".virtual_mechanism.P_target_2_ref", ".robot.P_grip_1"); id="SErr2")
add_coordinate!(vms, CoordDifference(".virtual_mechanism.P_target_3_ref", ".robot.P_grip_3"); id="SErr3")
for i in 1:3
    add_component!(vms, TanhSpring("SErr$i"; max_force=5.0, stiffness=150.0); id="SSpr$i")
    add_component!(vms, LinearDamper(20.0, "SErr$i"); id="SDmp$i") 
end

# =======================================================================================
# --- 4. LOGICA DI CONTROLLO (Aggiornata per N segmenti) ---
# =======================================================================================
function f_setup(cache)
    swp_ref = cache[get_compiled_coordID(cache, ".virtual_mechanism.SliderWorldPos_Ref")].coord_data.val
    p1_ref = cache[get_compiled_coordID(cache, ".virtual_mechanism.P_target_1_ref")].coord_data.val
    p2_ref = cache[get_compiled_coordID(cache, ".virtual_mechanism.P_target_2_ref")].coord_data.val
    p3_ref = cache[get_compiled_coordID(cache, ".virtual_mechanism.P_target_3_ref")].coord_data.val
    
    slider_s_id = get_compiled_coordID(cache, ".virtual_mechanism.SliderCoord_s")
    pivot_id = get_compiled_coordID(cache, ".virtual_mechanism.Pivot_Pos")
    
    # Lista dinamica ID basata su num_generated_segments (Globale o passata)
    seg_ids_compiled = [get_compiled_coordID(cache, ".virtual_mechanism.Seg$(i)_Pos") for i in 1:num_generated_segments]
    
    rob_prog_ref = cache[get_compiled_coordID(cache, ".virtual_mechanism.RobotProgress_s_Ref")].coord_data.val
    tcp_id = get_compiled_coordID(cache, ".robot.TCP") 
    
    return (swp_ref, p1_ref, p2_ref, p3_ref, slider_s_id, pivot_id, seg_ids_compiled, rob_prog_ref, tcp_id)
end

function f_control(cache, t, args, extra)
    (swp_ref, p1_ref, p2_ref, p3_ref, slider_s_id, pivot_id, seg_ids_compiled, rob_prog_ref, tcp_id) = args

    p_pivot = configuration(cache, pivot_id)
    # Recupera posizioni di TUTTI i segmenti
    p_segs = [configuration(cache, id) for id in seg_ids_compiled]
    points_path = pushfirst!(copy(p_segs), p_pivot)

    # Proiezione
    p_robot = configuration(cache, tcp_id)
    best_dist_sq = Inf
    s_robot_projected = 0.0
    current_s_proj = 0.0
    
    # Loop su tutti i segmenti (la lista points_path ora ha dimensione dinamica corretta)
    for k in 1:(length(points_path)-1)
        len_k = gen_results_lengths[k] 

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
            s_robot_projected = current_s_proj + t_clamped * len_k
        end
        current_s_proj += len_k
    end
    rob_prog_ref[] = SVector(s_robot_projected)

    # Slider Target
    s_val = configuration(cache, slider_s_id)[1]
    s_clamped = clamp(s_val, 0.0, BAR_LENGTH_TOTAL) 
    
    current_s = 0.0
    seg_index = 1
    ratio = 0.0 
    
    for i in 1:num_generated_segments
        len_i = gen_results_lengths[i]
        
        if s_clamped <= (current_s + len_i) || i == num_generated_segments
            seg_index = i
            s_local = s_clamped - current_s
            ratio = clamp(s_local / len_i, 0.0, 1.0) 
            break
        end
        current_s += len_i
    end

    p_start = points_path[seg_index]
    p_end = points_path[seg_index+1]
    pos_3D = p_start + (p_end - p_start) * ratio 
    swp_ref[] = pos_3D
    
    # Orientamento fisso
    TARGET_SIDE = SVector(0., 1., 0.)
    TARGET_UP = SVector(1., 0., 0.)
    p1_ref[] = pos_3D + TARGET_SIDE * GRIP_BAR_LENGTH_HALF
    p2_ref[] = pos_3D - TARGET_SIDE * GRIP_BAR_LENGTH_HALF
    p3_ref[] = pos_3D + TARGET_UP * GRIP_TRIANGLE_HEIGHT
    
    return false
end

# =======================================================================================
# --- 5. ASSEMBLAGGIO STATO INIZIALE & SIMULAZIONE --- 
# =======================================================================================
println("5. Assemblaggio stato e Simulazione...")

q_vm_components = Float64[]

# Base (Seg 1)
push!(q_vm_components, gen_results_angles[1][1]) 
push!(q_vm_components, 0.0)                      
push!(q_vm_components, gen_results_angles[1][2]) 
push!(q_vm_components, gen_results_lengths[1])   
# Segmenti successivi
for i in 2:num_generated_segments
    push!(q_vm_components, gen_results_angles[i][1]) 
    push!(q_vm_components, gen_results_angles[i][2]) 
    push!(q_vm_components, gen_results_lengths[i])   
end

push!(q_vm_components, BAR_LENGTH_TOTAL) 

q_vm_initial = q_vm_components
println("   Gradi di libertà totali VMS: $(length(q_vm_initial))")

tspan = (0.0, 15.0)
q_initial = (q_robot_full, q_vm_initial)
q̇_initial = (zeros(n_dof_rob), zeros(length(q_vm_initial)))
prob = get_ode_problem(new_dynamics_cache(compile(vms)), VMRobotControl.DEFAULT_GRAVITY, q_initial, q̇_initial, tspan; f_setup, f_control) 
sol = solve(prob, Tsit5(); maxiters=3e5, abstol=1e-6, reltol=1e-6)


# =======================================================================================
# --- 7. ANALISI SISTEMATICA E DATA LOGGING ---
# =======================================================================================
println("\n========================================")
println("7. Analisi Metriche e Salvataggio Dati")
println("========================================")

# --- A. Setup Variabili per Metriche ---
min_obstacle_distance = Inf
max_path_error = 0.0
cumulative_jerk = 0.0
robot_completion_time = NaN
is_completed = false
completion_threshold = 0.05 # 5 cm di tolleranza per considerare il target raggiunto
start_pos_error_check = 0.5 # Inizia a controllare l'errore dopo 0.5s (per evitare transitori iniziali)

# Cache per calcoli cinematici nel loop di analisi
analysis_cache = new_kinematics_cache(compile(vms))
tcp_id_compiled = get_compiled_coordID(compile(vms), ".robot.TCP")
seg_ids_compiled = [get_compiled_coordID(compile(vms), ".virtual_mechanism.Seg$(i)_Pos") for i in 1:num_generated_segments]
pivot_id_compiled = get_compiled_coordID(compile(vms), ".virtual_mechanism.Pivot_Pos")

# --- B. Loop di Analisi sulla Soluzione ---
# Campioniamo la soluzione a 100Hz per avere metriche accurate
t_eval = range(0.0, sol.t[end], length=1500) 
dt_eval = step(t_eval)

for (idx, t) in enumerate(t_eval)
    # 1. Estrazione Stato e Aggiornamento Cinematica (con fix della Tupla)
    state = sol(t)
    state_deriv = sol(t, Val{1}) 
    
    n_total_dof = length(state) ÷ 2
    q_current = state[1:n_total_dof]
    q_dot_current = state[n_total_dof+1:end]
    q_ddot_current = state_deriv[n_total_dof+1:end]

    # Spacchettamento Tupla per kinematics!
    n_rob = config_size(compile(robot)) 
    q_r_curr = q_current[1:n_rob]
    q_v_curr = q_current[n_rob+1:end]
    kinematics!(analysis_cache, t, (q_r_curr, q_v_curr)) # Fix MethodError
    
    p_robot = configuration(analysis_cache, tcp_id_compiled)

    # 2. Calcolo Robot Completion Time (CORRETTO)
    # In questa simulazione il robot va da Punta -> Pivot (s=0).
    # Quindi il target finale è il Pivot.
    p_dest = configuration(analysis_cache, pivot_id_compiled) 
    
    dist_to_target = norm(p_robot - p_dest)
    
    if !is_completed && dist_to_target < completion_threshold
        # Controllo velocità robot (deve essere quasi fermo)
        robot_vel_norm = norm(q_dot_current[1:7]) 
        if robot_vel_norm < 0.1
            global robot_completion_time = t
            global is_completed = true
        end
    end

    # 3. Calcolo Distanza Minima Ostacoli
    for obs in OBSTACLES_LIST
        d = norm(p_robot - obs) - 0.04 
        if d < min_obstacle_distance
            global min_obstacle_distance = d
        end
    end

    # 4. Calcolo Smoothness (Jerk)
    if idx > 1
        jerk_vec = (q_ddot_current - prev_q_ddot) / dt_eval
        global cumulative_jerk += sum(abs2, jerk_vec) * dt_eval 
    end
    global prev_q_ddot = q_ddot_current

    # 5. Calcolo Path Error
    if t > start_pos_error_check
        p_pivot_pos = configuration(analysis_cache, pivot_id_compiled)
        p_segs = [configuration(analysis_cache, id) for id in seg_ids_compiled]
        points_path = pushfirst!(copy(p_segs), p_pivot_pos)
        
        best_dist_sq = Inf
        
        for k in 1:(length(points_path)-1)
            p_start = points_path[k]
            p_end = points_path[k+1]
            v_seg = p_end - p_start
            v_point = p_robot - p_start
            seg_len_sq = dot(v_seg, v_seg)
            
            if seg_len_sq > 1e-6
                t_proj = clamp(dot(v_point, v_seg) / seg_len_sq, 0.0, 1.0)
                closest_pt = p_start + v_seg * t_proj
                d_sq = sum(abs2, closest_pt - p_robot)
                if d_sq < best_dist_sq
                    best_dist_sq = d_sq
                end
            end
        end
        current_err = sqrt(best_dist_sq)
        if current_err > max_path_error
            global max_path_error = current_err
        end
    end
end

# Se non ha mai raggiunto il target
if isnan(robot_completion_time)
    robot_completion_time = sol.t[end]
    println("   Attenzione: Robot non ha raggiunto il target entro $(sol.t[end])s")
end

# Normalizzazione Jerk (opzionale, per renderlo indipendente dalla durata)
# Log dimensionless jerk or simple mean jerk
mean_jerk = cumulative_jerk / sol.t[end]

# --- C. Stampa a Video ---
println("\n--- RISULTATI ANALISI (L_SEGMENT = $L_SEGMENT_DEFAULT, N_SEG = $num_generated_segments) ---")
println("1. Computation Time (Path Gen): $(round(t_generation_elapsed, digits=4)) s")
println("2. Path Length Total:           $(round(BAR_LENGTH_TOTAL, digits=3)) m")
println("3. Robot Completion Time:       $(round(robot_completion_time, digits=3)) s")
println("4. Max Path Error:              $(round(max_path_error * 1000, digits=2)) mm")
println("5. Min Dist to Obstacles:       $(round(min_obstacle_distance * 1000, digits=2)) mm")
println("6. Smoothness (Int Sq Jerk):    $(round(mean_jerk, digits=2))")
println("---------------------------------------------------------------")

# --- D. Salvataggio su File ---
filename = "vmc_simulation_results.txt"
file_exists = isfile(filename)

open(filename, "a") do f
    # Scrivi header se il file è nuovo
    if !file_exists
        write(f, "L_Segment\tN_Segments\tPath_Length\tComp_Time_Gen\tComp_Time_Robot\tMax_Path_Err_m\tMin_Obs_Dist_m\tJerk_Index\n")
    end
    # Scrivi riga dati
    write(f, "$L_SEGMENT_DEFAULT\t$num_generated_segments\t$BAR_LENGTH_TOTAL\t$t_generation_elapsed\t$robot_completion_time\t$max_path_error\t$min_obstacle_distance\t$mean_jerk\n")
end
println("   Dati salvati in $filename")

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

# Visualizzazione Statica
mesh!(ls, Sphere(Point3f(PIVOT_POINT), 0.03), color=:green, label="Pivot")
for (k, obs) in enumerate(OBSTACLES_LIST)
    mesh!(ls, Sphere(Point3f(obs), 0.04), color=:orange, label="Ostacolo $k")
end
mesh!(ls, Rect3f(Vec3f(-1.0, -1.0, -0.001), Vec3f(2.0, 2.0, 0.002)), color=(:blue, 0.1), transparency=true)
mesh!(ls, Sphere(Point3f(P_EE_INITIAL), 0.01), color=:red, label="PEE Initial")

# Visualizzazione Snake
points_obs = []
pivot_id = get_compiled_coordID(compile(vms), ".virtual_mechanism.Pivot_Pos")
push!(points_obs, map(c -> Point3f(configuration(c, pivot_id)), pkcache))

for i in 1:num_generated_segments
    id = get_compiled_coordID(compile(vms), ".virtual_mechanism.Seg$(i)_Pos")
    push!(points_obs, map(c -> Point3f(configuration(c, id)), pkcache))
end

for i in 1:(length(points_obs)-1) 
    lines!(ls, map((p1, p2) -> [p1, p2], points_obs[i], points_obs[i+1]), color=:red, linewidth=5)
end

# Cilindri Telescopici
for i in 1:num_generated_segments
    base_id = get_compiled_coordID(compile(vms), ".virtual_mechanism.Seg$(i)_Base_Pos")
    tip_id  = get_compiled_coordID(compile(vms), ".virtual_mechanism.Seg$(i)_Pos")
    
    p_base = map(c -> Point3f(configuration(c, base_id)), pkcache)
    p_tip  = map(c -> Point3f(configuration(c, tip_id)), pkcache)
    
    L_nom = Float32(L_SEGMENT_DEFAULT)
    
    p_nominal_tip = map((b, t) -> begin
        vec = t - b
        dist = norm(vec)
        if dist < 1e-6
            return b + Point3f(0, 0, L_nom)
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

cursore_id = get_compiled_coordID(compile(vms), ".virtual_mechanism.SliderWorldPos_Ref")
scatter!(ls, map(c -> Point3f(configuration(c, cursore_id)), pkcache), color=:blue, markersize=20, label="Target")

axislegend(ls, position=:rt)

savepath = joinpath(@__DIR__, "franka_recursive_snake.mp4")
println("Rendering in corso...")
animate_robot_odesolution(fig, sol, pkcache, savepath; f_setup, f_control) 
@info "Animazione salvata: $savepath"


