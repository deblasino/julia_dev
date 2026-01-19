# =======================================================================================
# SIMULAZIONE VMC IBRIDA - SBARRA "SNAKE" A LUNGHEZZA VARIABILE (RICORSIVA)
# =======================================================================================

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

# =======================================================================================
# --- 0. PARAMETRI DI SIMULAZIONE ---
# =======================================================================================

const PIVOT_POINT = SVector(0.6, 0.0, 0.05)
const OBSTACLE_POSITION_1 = SVector(0.5, 0.1, 0.2)
const OBSTACLE_POSITION_2 = SVector(0.4, -0.1, 0.3)
const OBSTACLES_LIST = [OBSTACLE_POSITION_1, OBSTACLE_POSITION_2]

const GRIP_BAR_LENGTH_HALF = 0.20
const GRIP_TRIANGLE_HEIGHT = 0.20

# --- NUOVI PARAMETRI PER GENERAZIONE RICORSIVA ---
const MAX_SEGMENTS = 7           # Numero massimo di segmenti
const SEGMENT_LENGTH = 0.15      # Lunghezza fissa di ogni segmento (ridotta per maggiore flessibilità) 
const TARGET_TOLERANCE = 0.10    # 10 cm di tolleranza per smettere di aggiungere segmenti

global NUM_ACTUAL_SEGMENTS = MAX_SEGMENTS

# =======================================================================================
# --- 1. ROBOT MODEL CONFIGURATION (Invariato) ---
# =======================================================================================
println("1. Setup del robot...") 
cfg = URDFParserConfig(; suppress_warnings=true)
module_path = joinpath(splitpath(splitdir(pathof(VMRobotControl))[1])[1:end-1])
robot = parseURDF(joinpath(module_path, "URDFs/franka_description/urdfs/fr3_franka_hand(copy).urdf"), cfg)
add_gravity_compensation!(robot, VMRobotControl.DEFAULT_GRAVITY)

# Recuperiamo i limiti dei giunti dalla configurazione URDF 
joint_limits = cfg.joint_limits

println("INFO: Applicazione Safety Springs e TanhDampers...")

# Definiamo coppie di Coulomb specifiche per i giunti (più alti alla base, più bassi al polso)
# Attrito molto più alto sui primi giunti, scende verso il polso
for (i, τ_coulomb) in zip(1:7, [2.5, 2.5, 2.5, 2.5, 1.5, 1.5, 1.5])
    joint_name = "fr3_joint$i"
    coord_id = "J$i" # ID breve per la coordinata del giunto
    
    # 1. Aggiunta Coordinata del Giunto
    # JointSubspace rappresenta la configurazione q del giunto specifico
    add_coordinate!(robot, JointSubspace(joint_name); id=coord_id)
    
    # Recupero limiti dal dizionario URDF
    limits = joint_limits[joint_name]
    
    # 2. Applicazione Molle di Sicurezza (Deadzone Springs)
    # Se i limiti esistono nel file URDF, aggiungiamo molle "buffer" a 0.1 rad dal fine corsa.
    # Questo è analogo all'esempio sciurus_reaching.jl 
    if !isnothing(limits) && !isnothing(limits.lower) && !isnothing(limits.upper)
        add_deadzone_springs!(robot, 50.0, (limits.lower + 0.1, limits.upper - 0.1), coord_id)
    end

    # 3. Applicazione Attrito di Coulomb (TanhDamper)
    # Il TanhDamper applica una forza di smorzamento che satura (simile all'attrito secco/Coulomb).
    # β (width) determina quanto è "netta" la transizione a velocità zero.
    β = 1e-1 
    add_component!(robot, TanhDamper(τ_coulomb, β, coord_id); id="JointDamper$i")
end

add_coordinate!(robot, FramePoint("fr3_hand_tcp", SVector(0.0, GRIP_BAR_LENGTH_HALF, 0.0)); id="P_grip_1")
add_coordinate!(robot, FramePoint("fr3_hand_tcp", SVector(0.0, -GRIP_BAR_LENGTH_HALF, 0.0)); id="P_grip_2")
add_coordinate!(robot, FramePoint("fr3_hand_tcp", SVector(GRIP_TRIANGLE_HEIGHT, 0.0, 0.0)); id="P_grip_3")
add_coordinate!(robot, FrameOrigin("fr3_hand_tcp"); id="TCP")

# =======================================================================================
# --- 1.5 PRE-CALCOLO TOPOLOGIA (Corretto e Funzionante) ---
# =======================================================================================
println("1.5. Generazione Topologia tramite Simulazione Fisica Iterativa...")

# --- A. Setup Target e Parametri (Invariato) ---
# Necessario per calcolare P_EE_INITIAL
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

# Funzione helper
function vector_to_yaw_pitch(v)
    v_norm = normalize(v)
    
    # MODIFICA PROFESSORE:
    # Poiché i segmenti sono definiti fisicamente lungo -Y (SVector(0, -L, 0)),
    # dobbiamo calcolare lo Yaw rispetto all'asse -Y, non rispetto a +X.
    # atan(x, -y) fa sì che se il vettore è (0, -1, 0), l'angolo sia 0.
    
    yaw = atan(v_norm[1], -v_norm[2]) 
    
    horiz = sqrt(v_norm[1]^2 + v_norm[2]^2)
    
    # Anche il pitch va gestito coerentemente (in 2.5 era negativo per l'asse Z)
    pitch = -atan(v_norm[3], horiz)
    
    return yaw, pitch 
end

# --- B. Costruzione Meccanismo "Generatore" ---
vm_gen = Mechanism{Float64}("SnakeGenerator")
gen_pivot = add_frame!(vm_gen, "gen_pivot")
gen_ry = add_frame!(vm_gen, "gen_ry")

add_joint!(vm_gen, Rigid(Transform(PIVOT_POINT)); parent=root_frame(vm_gen), child=gen_pivot, id="Gen_Pivot_Joint")
add_joint!(vm_gen, Revolute(SVector(0., 0., 1.)); parent=gen_pivot, child=gen_ry, id="Gen_Base_Yaw")

add_coordinate!(vm_gen, FrameOrigin(gen_pivot); id="gen_pivot")
add_coordinate!(vm_gen, ConstCoord(P_EE_INITIAL); id="Gen_Target")

# Variabili di stato per il loop
prev_frame = gen_ry
segment_frames = []

# Parametri ottimizzati per stabilità
const K_ATTR_GEN = 50.0   
const K_REPEL_GEN = -20.0  
const DAMPING_MOVING = 5.0 
const DAMPING_FIXED = 100.0 

const SEG_MASS = 0.5 
const SEG_INERTIA_VAL = (1/12) * SEG_MASS * SEGMENT_LENGTH^2
const SEG_INERTIA_TENSOR = SMatrix{3,3}(SEG_INERTIA_VAL, 0, 0, 0, 1e-4, 0, 0, 0, SEG_INERTIA_VAL)

# Variabile globale per mantenere la "memoria" della forma tra un'iterazione e l'altra
global q_prev = Float64[] 

# Array globali risultati
global segment_vectors = SVector{3, Float64}[]
global segment_angles = Tuple{Float64, Float64}[]

for i in 1:MAX_SEGMENTS
    # Dichiarazione globale necessaria per aggiornare le variabili dentro il loop
    global prev_frame
    global q_prev 
    
    # --- 1. COSTRUZIONE MECCANISMO (Incremental) ---
    j_frame = add_frame!(vm_gen, "gen_j$(i)")
    s_frame = add_frame!(vm_gen, "gen_seg$(i)")
    push!(segment_frames, s_frame)
    
    # Logica Giunti
    axis = (i == 1) ? SVector(1., 0., 0.) : SVector(1., 0., 0.) 
    parent_j = (i == 1) ? prev_frame : add_frame!(vm_gen, "gen_int$(i)")
    
    if i > 1
        add_joint!(vm_gen, Revolute(SVector(0., 0., 1.)); parent=prev_frame, child=parent_j, id="Gen_J$(i)_Yaw")
    end
    
    add_joint!(vm_gen, Revolute(axis); parent=parent_j, child=j_frame, id="Gen_J$(i)_Pitch")
    add_joint!(vm_gen, Rigid(Transform(SVector(0.0, -SEGMENT_LENGTH, 0.0))); parent=j_frame, child=s_frame, id="Gen_Seg_Rigid$(i)")

    # Componenti Inerziali
    add_coordinate!(vm_gen, FrameAngularVelocity(j_frame); id="Gen_Vel$(i)")
    add_component!(vm_gen, Inertia(SEG_INERTIA_TENSOR, "Gen_Vel$(i)"); id="Gen_Inertia$(i)")
    add_component!(vm_gen, LinearDamper(DAMPING_MOVING * I(3), "Gen_Vel$(i)"); id="Gen_Damper$(i)")

    # Coordinate Posizione
    add_coordinate!(vm_gen, FrameOrigin(s_frame); id="Gen_Pos$(i)")
    
    # Molle Repulsive
    for (k, obs) in enumerate(OBSTACLES_LIST)
        obs_coord_name = "Gen_Obs$(i)_$(k)"
        if !haskey(vm_gen.coordinates, obs_coord_name) 
             add_coordinate!(vm_gen, ConstCoord(obs); id=obs_coord_name)
        end
        add_coordinate!(vm_gen, CoordDifference("Gen_Pos$(i)", obs_coord_name); id="Gen_Err_Obs$(i)_$(k)")
        add_component!(vm_gen, GaussianSpring("Gen_Err_Obs$(i)_$(k)"; stiffness=K_REPEL_GEN, width=0.15); id="Gen_Repel$(i)_$(k)")
    end

    # Gestione Molla Attrattiva e Smorzamento
    if i > 1
        # Rimuove attrazione dal segmento precedente
        if haskey(vm_gen.components, "Gen_Attract$(i-1)")
            delete!(vm_gen.components, "Gen_Attract$(i-1)")
        end
        # Blocca il segmento precedente (aumenta smorzamento)
        if haskey(vm_gen.components, "Gen_Damper$(i-1)")
            delete!(vm_gen.components, "Gen_Damper$(i-1)")
        end
        add_component!(vm_gen, LinearDamper(DAMPING_FIXED * I(3), "Gen_Vel$(i-1)"); id="Gen_Damper$(i-1)")
    end
    
    # Attrae la NUOVA punta al target
    add_coordinate!(vm_gen, CoordDifference("Gen_Pos$(i)", "Gen_Target"); id="Gen_Err_Attr$(i)")
    add_component!(vm_gen, LinearSpring(K_ATTR_GEN, "Gen_Err_Attr$(i)"); id="Gen_Attract$(i)")

    prev_frame = s_frame

    # --- 2. COMPILAZIONE E GESTIONE STATO ---
    m_step = compile(vm_gen)
    
    # COSTRUZIONE STATO INIZIALE (Corrected syntax)
    local q_start
    if i == 1
        q_start = 1e-4 * ones(config_size(m_step))
    else
        # Calcolo nuovi gradi di libertà
        n_prev = length(q_prev)
        n_curr = config_size(m_step)
        n_new = n_curr - n_prev
        
        # Concatenazione corretta (senza 'global' dentro le quadre)
        q_start = [q_prev; zeros(n_new)]
    end

    println("   -> Ottimizzazione Segmento $i (Simulazione)...")
    
    # --- 3. SIMULAZIONE ---
    cache_step = new_dynamics_cache(m_step)
    
    # Tempo sufficiente per assestamento
    dt_local = 3.0 
    prob_step = get_ode_problem(cache_step, VMRobotControl.DEFAULT_GRAVITY, q_start, zero_q̇(m_step), (0.0, dt_local))
    
    sol_step = solve(prob_step, Tsit5(); abstol=1e-4, reltol=1e-4)
    
    # Aggiornamento dello stato globale per la prossima iterazione
    q_prev = sol_step.u[end][1:config_size(m_step)]
    
    # --- 4. ESTRAZIONE DATI ---
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
# --- 2. VIRTUAL MECHANISM DEFINITION (Generazione Dinamica) ---
# =======================================================================================
println("2. Creazione della 'Snake-Bar' dinamica ($NUM_ACTUAL_SEGMENTS segmenti)...")

segment_inertia_val = (1 / 12) * 1.0 * SEGMENT_LENGTH^2 # Massa unitaria approx
segment_inertia_tensor = SMatrix{3,3}(segment_inertia_val, 0, 0, 0, 1e-3, 0, 0, 0, segment_inertia_val)

vm = Mechanism{Float64}("VirtualSnakeRecursive")

# Base Pivot
pivot_frame = add_frame!(vm, "pivot_frame")
frame_rz = add_frame!(vm, "frame_rz")
frame_ry = add_frame!(vm, "frame_ry") # Questo funge da "radice" per la catena
add_joint!(vm, Rigid(Transform(PIVOT_POINT)); parent=root_frame(vm), child=pivot_frame, id="PivotAnchor") 
add_joint!(vm, Revolute(SVector(0., 0., 1.)); parent=pivot_frame, child=frame_rz, id="J_base_yaw")
add_joint!(vm, Revolute(SVector(0., 1., 0.)); parent=frame_rz, child=frame_ry, id="J_base_pitch")
# Nota: Il "roll" lo mettiamo nel primo segmento come in 2_4.jl originale


add_coordinate!(vm, FrameOrigin(pivot_frame); id="Pivot_Pos")

# GENERAZIONE DINAMICA SEGMENTI
# Struttura: Base -> Seg 1 -> Seg 2 -> ... -> Seg N
# Nota: Nel codice originale Seg3 era la base. Qui usiamo indici 1..N dove 1 è base.

previous_frame = frame_ry
seg_ids = String[]

for i in 1:NUM_ACTUAL_SEGMENTS
    # Creazione Frames Obbligatori
    joint_frame = add_frame!(vm, "joint$(i)_frame")
    seg_frame = add_frame!(vm, "seg$(i)_frame")
    
    push!(seg_ids, "seg$(i)_frame") # Salviamo ID per dopo
    
    if i == 1
        # Primo segmento: Connesso DIRETTAMENTE alla base rotante
        # Qui NON serve int_frame_1, quindi NON lo creiamo con add_frame! 
        # Roll (X) relativo a frame_ry
        add_joint!(vm, Revolute(SVector(1., 0., 0.)); parent=previous_frame, child=joint_frame, id="J_roll_base")
        # Link rigido
        add_joint!(vm, Rigid(Transform(SVector(0.0, -SEGMENT_LENGTH, 0.0))); parent=joint_frame, child=seg_frame, id="Link$(i)_Rigid")
    else
        # Segmenti successivi: Hanno bisogno del frame intermedio per i 2 DOF
        int_frame = add_frame!(vm, "int_frame_$(i)") 
        
        # Yaw (Z) -> connette previous a int_frame
        add_joint!(vm, Revolute(SVector(0., 0., 1.)); parent=previous_frame, child=int_frame, id="J_seg$(i)_yaw")
        # Pitch (X) -> connette int_frame a joint_frame
        add_joint!(vm, Revolute(SVector(1., 0., 0.)); parent=int_frame, child=joint_frame, id="J_seg$(i)_pitch")
        # Link rigido
        add_joint!(vm, Rigid(Transform(SVector(0.0, -SEGMENT_LENGTH, 0.0))); parent=joint_frame, child=seg_frame, id="Link$(i)_Rigid")
    end
    
    # Dinamica Segmento (Invariata)
    add_coordinate!(vm, FrameAngularVelocity(seg_frame); id="Seg$(i)_AngVel") 
    add_component!(vm, Inertia(segment_inertia_tensor, "Seg$(i)_AngVel"); id="Seg$(i)_Inertia")
    add_component!(vm, LinearDamper(0.5 * I(3), "Seg$(i)_AngVel"); id="Seg$(i)_RotDamper")
    
    # Coordinate Posizione per VMS
    add_coordinate!(vm, FrameOrigin(seg_frame); id="Seg$(i)_Pos")
    
    global previous_frame = seg_frame
end

# Motore Slider (Invariato)
slider_s_frame = add_frame!(vm, "slider_s_frame")
add_joint!(vm, Prismatic(SVector(1., 0., 0.)); parent=root_frame(vm), child=slider_s_frame, id="J_slider_progress")
add_coordinate!(vm, JointSubspace("J_slider_progress"); id="SliderCoord_s")
add_component!(vm, LinearInerter(0.1, "SliderCoord_s"); id="SliderProgressInertance")

# Target (Invariato)
initial_slider_pos_world = PIVOT_POINT - SVector(0.0, BAR_LENGTH_TOTAL, 0.0)
add_coordinate!(vm, ReferenceCoord(Ref(initial_slider_pos_world)); id="SliderWorldPos_Ref")
add_coordinate!(vm, ReferenceCoord(Ref(initial_slider_pos_world)); id="P_target_1_ref")
add_coordinate!(vm, ReferenceCoord(Ref(initial_slider_pos_world)); id="P_target_2_ref") 
add_coordinate!(vm, ReferenceCoord(Ref(initial_slider_pos_world)); id="P_target_3_ref")


# =======================================================================================
# --- 3. VMS ASSEMBLY (Ciclo dinamico ostacoli) ---
# =======================================================================================
println("3. Setup VMS...")
vms = VirtualMechanismSystem("SlidingControl", robot, vm)
add_coordinate!(vm, ConstCoord(OBSTACLE_POSITION_1); id="ObstaclePosition1")
add_coordinate!(vm, ConstCoord(OBSTACLE_POSITION_2); id="ObstaclePosition2")
ostacoli_vm = ["ObstaclePosition1", "ObstaclePosition2"]

# Repulsione Segmenti (Ciclo dinamico 1..N)
for i in 1:NUM_ACTUAL_SEGMENTS
    seg_pos_id = "Seg$(i)_Pos"
    for (j, ostacolo_id) in enumerate(ostacoli_vm)
        add_coordinate!(vms, CoordDifference(".virtual_mechanism.$(seg_pos_id)", ".virtual_mechanism.$(ostacolo_id)"); id="Seg$(i)_Err$(j)")
        add_component!(vms, GaussianSpring("Seg$(i)_Err$(j)"; stiffness=-80.0, width=0.08); id="Seg$(i)_Spr$(j)") 
        #add_component!(vms, LinearDamper(20.0, "Seg$(i)_Err$(j)"); id="Seg$(i)_Dmp$(j)")
    end
end

# Repulsione Robot (Invariata)
links_robot = ["fr3_link4", "fr3_link5", "fr3_link6", "fr3_link7", "fr3_link8", "fr3_hand"]
ostacoli_robot_cfg = [(id=1, c=".virtual_mechanism.ObstaclePosition1"), (id=2, c=".virtual_mechanism.ObstaclePosition2")]
for lnk in links_robot
    add_coordinate!(robot, FrameOrigin(lnk); id="$(lnk)_origin")
    for obs in ostacoli_robot_cfg
        add_coordinate!(vms, CoordDifference(".robot.$(lnk)_origin", obs.c); id="$(lnk)_ObsErr$(obs.id)")
        add_component!(vms, GaussianSpring("$(lnk)_ObsErr$(obs.id)"; stiffness=-350.0, width=0.11); id="$(lnk)_ObsSpr$(obs.id)")
    end
end



println("3.X. Applicazione vincolo rigido sulla punta della Snake-Bar...")

# 1. Identifichiamo il frame dell'ultimo segmento generato
#    In fase di costruzione (Sez 2), i frame sono stati chiamati "seg$(i)_frame"
last_seg_frame_name = "seg$(NUM_ACTUAL_SEGMENTS)_frame"

# 2. Definiamo la coordinata della posizione attuale della punta (FrameOrigin)
#    Questa coordinata rappresenta la posizione Cartesiana dell'ultimo link virtuale.
add_coordinate!(vm, FrameOrigin(last_seg_frame_name); id="Snake_Tip_Pos")

# 3. Definiamo il punto di ancoraggio fisso (ConstCoord)
#    Usiamo P_EE_INITIAL, calcolato nella Sezione 1.5, come punto di ancoraggio fisso nello spazio.
add_coordinate!(vm, ConstCoord(P_EE_INITIAL); id="Snake_Tip_Anchor_Ref")

# 4. Calcoliamo l'errore (CoordDifference)
#    Differenza vettoriale tra la punta attuale e l'ancoraggio.
add_coordinate!(vms, CoordDifference(".virtual_mechanism.Snake_Tip_Pos", ".virtual_mechanism.Snake_Tip_Anchor_Ref");
                id="Snake_Tip_Anchor_Error")

# 5. Applichiamo Molla e Smorzatore (Components)
#    - LinearSpring: Stiffness 200.0 N/m (o superiore) per mantenere la punta vicina all'ancoraggio.commen
#    - LinearDamper: Smorzamento 20.0 Ns/m per dissipare energia ed evitare oscillazioni.
add_component!(vms, LinearSpring(200.0, "Snake_Tip_Anchor_Error"); id="Snake_Tip_Constraint_Spring")
add_component!(vms, LinearDamper(20.0, "Snake_Tip_Anchor_Error"); id="Snake_Tip_Constraint_Damper")




# Controllo Motore Slider (DRIVE)
add_coordinate!(vm, ConstCoord(SVector(0.0)); id="SliderTarget_s")
add_coordinate!(vms, CoordDifference(".virtual_mechanism.SliderCoord_s", ".virtual_mechanism.SliderTarget_s"); id="SliderDrivingError_s")
add_component!(vms, TanhSpring("SliderDrivingError_s"; max_force=15.0, stiffness=350.0); id="SliderDrivingSpring") 
add_component!(vms, LinearDamper(10.0, "SliderDrivingError_s"); id="SliderDrivingDamper")

# --- MODIFICA PROFESSORE: Feedback Posizione Robot su Slider ---
# 1. Definiamo una coordinata di riferimento che conterrà il valore 's' (progresso) 
#    corrente del robot lungo la catena (calcolato in f_control). 
#    Usiamo ReferenceCoord poiché il suo valore può essere modificato dinamicamente (mutabile) 
#    all'interno di `f_control`.
add_coordinate!(vm, ReferenceCoord(Ref(SVector(0.0))); id="RobotProgress_s_Ref")

# 2. Creiamo la differenza tra la posizione dello slider (s) e quella del robot (s_robot)
#    Questa è la "distanza di errore" nello spazio operativo dello slider (1D).
add_coordinate!(vms, CoordDifference(".virtual_mechanism.SliderCoord_s", ".virtual_mechanism.RobotProgress_s_Ref"); id="SliderFeedbackError_s")

# 3. Aggiungiamo la MOLLA DI FEEDBACK e lo SMORZAMENTO.
#    Questa molla funge da accoppiamento dinamico (complaintness) tra il target (definito 
#    dallo slider) e il robot (attraverso la sua posizione proiettata). 
add_component!(vms, TanhSpring("SliderFeedbackError_s"; max_force = 15.0, stiffness=350.0); id="SliderFeedbackSpring")
add_component!(vms, LinearDamper(10.0, "SliderFeedbackError_s"); id="SliderFeedbackDamper")
# -------------------------------------------------------------------------------------

# Accoppiamento Robot-Cursore (Invariato)
add_coordinate!(vms, CoordDifference(".virtual_mechanism.SliderWorldPos_Ref", ".robot.TCP"); id="PathError")
add_component!(vms, TanhSpring("PathError"; max_force=15.0, stiffness=350.0); id="PathSpring")

add_component!(vms, LinearDamper(10.0, "PathError"); id="PathDamper")

# Orientamento Treppiede (Invariato)
add_coordinate!(vms, CoordDifference(".virtual_mechanism.P_target_1_ref", ".robot.P_grip_2"); id="SErr1")
add_coordinate!(vms, CoordDifference(".virtual_mechanism.P_target_2_ref", ".robot.P_grip_1"); id="SErr2")
add_coordinate!(vms, CoordDifference(".virtual_mechanism.P_target_3_ref", ".robot.P_grip_3"); id="SErr3")

for i in 1:3
    add_component!(vms, TanhSpring("SErr$i"; max_force=5.0, stiffness=150.0); id="SSpr$i")
    add_component!(vms, LinearDamper(20.0, "SErr$i"); id="SDmp$i")
end

# =======================================================================================
# --- 4. LOGICA DI CONTROLLO (Generalizzata per N segmenti) ---
# =======================================================================================
println("4. Definizione f_control generalizzata...")

function f_setup(cache)
    # Refs (Invariato)
    swp_ref = cache[get_compiled_coordID(cache, ".virtual_mechanism.SliderWorldPos_Ref")].coord_data.val
    p1_ref = cache[get_compiled_coordID(cache, ".virtual_mechanism.P_target_1_ref")].coord_data.val
    p2_ref = cache[get_compiled_coordID(cache, ".virtual_mechanism.P_target_2_ref")].coord_data.val
    p3_ref = cache[get_compiled_coordID(cache, ".virtual_mechanism.P_target_3_ref")].coord_data.val
    
    # IDs (Invariato)
    slider_s_id = get_compiled_coordID(cache, ".virtual_mechanism.SliderCoord_s")
    pivot_id = get_compiled_coordID(cache, ".virtual_mechanism.Pivot_Pos")
    
    # Recuperiamo IDs segmenti in ordine (Seg1 -> SegN)
    seg_ids_compiled = [get_compiled_coordID(cache, ".virtual_mechanism.Seg$(i)_Pos") for i in 1:NUM_ACTUAL_SEGMENTS]

    # --- MODIFICA PROFESSORE: Recupero ID e Reference ---
    # Reference al valore s_robot da aggiornare (ReferenceCoord)
    rob_prog_ref = cache[get_compiled_coordID(cache, ".virtual_mechanism.RobotProgress_s_Ref")].coord_data.val
    
    # ID della posizione cartesiana del robot (TCP) per calcolare dove si trova
    tcp_id = get_compiled_coordID(cache, ".robot.TCP") 
    
    # Ritorna la tupla estesa 
    return (swp_ref, p1_ref, p2_ref, p3_ref, slider_s_id, pivot_id, seg_ids_compiled, rob_prog_ref, tcp_id)
end

function f_control(cache, t, args, extra)
    # De-pack degli argomenti aggiornati
    (swp_ref, p1_ref, p2_ref, p3_ref, slider_s_id, pivot_id, seg_ids_compiled, rob_prog_ref, tcp_id) = args

    # 1. Leggere Posizioni Punti Chiave della catena (Come prima)
    p_pivot = configuration(cache, pivot_id)
    p_segs = [configuration(cache, id) for id in seg_ids_compiled]
    points_path = pushfirst!(copy(p_segs), p_pivot) # [Pivot, Seg1, Seg2...] 

    # --- MODIFICA PROFESSORE: Calcolo Proiezione Robot su Catena ---
    # Leggiamo la posizione attuale del robot
    p_robot = configuration(cache, tcp_id)
    
    best_dist_sq = Inf
    s_robot_projected = 0.0
    
    # Iteriamo su ogni segmento della catena per trovare la proiezione minima
    # K va da 1 a NUM_ACTUAL_SEGMENTS
    for k in 1:(length(points_path)-1)
        p_start = points_path[k]
        p_end = points_path[k+1]
        
        # Vettore segmento e vettore verso il robot
        v_seg = p_end - p_start
        v_point = p_robot - p_start
        
        # Lunghezza segmento al quadrato
        seg_len_sq = dot(v_seg, v_seg)
        
        if seg_len_sq > 1e-6
            # Proiezione scalare normalizzata t in [0, 1]
            t_proj = dot(v_point, v_seg) / seg_len_sq
            t_clamped = clamp(t_proj, 0.0, 1.0)
        else
            t_clamped = 0.0
        end
        
        # Punto più vicino sul segmento k
        closest_pt = p_start + v_seg * t_clamped
        
        # Distanza al quadrato
        diff = closest_pt - p_robot
        dist_sq = dot(diff, diff)
        
        # Se è il punto più vicino trovato finora, salviamo il valore s
        if dist_sq < best_dist_sq
            best_dist_sq = dist_sq
            # Calcolo coordinata s globale: (segmenti precedenti) + (frazione corrente)
            s_base = (k - 1) * SEGMENT_LENGTH
            s_robot_projected = s_base + t_clamped * SEGMENT_LENGTH
        end
    end
    
    # AGGIORNAMENTO VMS: Scriviamo s_robot nella ReferenceCoord.
    # Questo valore sarà il "target" per la molla di feedback dello slider.
    # Deve essere un SVector anche se 1D.
    rob_prog_ref[] = SVector(s_robot_projected)
    # -----------------------------------------------------------

    # 3. Progresso Slider 's'
    s_val = configuration(cache, slider_s_id)[1]
    s_clamped = clamp(s_val, 0.0, BAR_LENGTH_TOTAL) 
    
    # 4. Interpolazione lungo la catena di segmenti
    # Troviamo in quale segmento ci troviamo
    seg_index = Int(floor(s_clamped / SEGMENT_LENGTH)) + 1
    if seg_index > NUM_ACTUAL_SEGMENTS seg_index = NUM_ACTUAL_SEGMENTS end
    
    # Posizione relativa nel segmento
    s_local = s_clamped - (seg_index - 1) * SEGMENT_LENGTH
    ratio = s_local / SEGMENT_LENGTH
    
    # Segmento k è definito tra points_path[k] (inizio) e points_path[k+1] (fine)
    p_start = points_path[seg_index]
    p_end = points_path[seg_index+1]
    
    pos_3D = p_start + (p_end - p_start) * ratio 

    # 5. Aggiornamento Target
    swp_ref[] = pos_3D
    
    # Orientamento fisso (invariato)
    TARGET_SIDE = SVector(0., 1., 0.)
    TARGET_UP = SVector(1., 0., 0.)
    p1_ref[] = pos_3D + TARGET_SIDE * GRIP_BAR_LENGTH_HALF
    p2_ref[] = pos_3D - TARGET_SIDE * GRIP_BAR_LENGTH_HALF
    p3_ref[] = pos_3D + TARGET_UP * GRIP_TRIANGLE_HEIGHT
    
    return false
end

# =======================================================================================
# --- 4.5 CALCOLO STATO INIZIALE VM (CORRETTO CON MATRICI DI ROTAZIONE) --- 
# =======================================================================================
println("4.5. Assemblaggio vettore stato iniziale (Inverse Kinematics Ricorsiva)...")

# Funzione helper per creare matrici di rotazione dai giunti
rot_z(θ) = @SMatrix [cos(θ) -sin(θ) 0; sin(θ) cos(θ) 0; 0 0 1]
rot_y(θ) = @SMatrix [cos(θ) 0 sin(θ); 0 1 0; -sin(θ) 0 cos(θ)]
rot_x(θ) = @SMatrix [1 0 0; 0 cos(θ) -sin(θ); 0 sin(θ) cos(θ)]

# 1. Configurazione Base (Seg 1)
# Il primo segmento è connesso al mondo tramite la sequenza J_base_yaw(Z) -> J_base_pitch(Y) -> J_roll_base(X)
# Usiamo gli angoli globali calcolati in 1.5 per orientare la base.
y1, p1 = segment_angles[1]

q_yaw_base   = y1
q_pitch_base = 0.0  # Come in 2.3, assumiamo che l'elevazione sia gestita dal Roll(X) 
q_roll_base  = p1   # L'elevazione (Pitch globale) diventa Roll nel giunto locale X

# Inizializziamo il vettore q
q_vm_components = Float64[q_yaw_base, q_pitch_base, q_roll_base]

# Teniamo traccia dell'orientamento accumulato (Matrice di Rotazione Globale) del segmento precedente
# Ordine rotazioni base: Z(yaw) -> Y(0) -> X(roll)
R_prev = rot_z(q_yaw_base) * rot_y(q_pitch_base) * rot_x(q_roll_base)

# 2. Configurazione Segmenti Successivi (Seg 2..N)
for i in 2:NUM_ACTUAL_SEGMENTS
    # Vettore desiderato (Globale) per questo segmento
    v_target_global = segment_vectors[i]
    
    # Proiettiamo il target nel frame locale del segmento precedente
    v_local = transpose(R_prev) * v_target_global 
    
    # Ora dobbiamo trovare q_yaw (Z) e q_pitch (X) tali che:
    # Ruotando il vettore "a riposo" (0, -1, 0) di Z(q_yaw) poi X(q_pitch), otteniamo v_local. 
    # Usiamo le funzioni helper "vector_to_yaw_pitch" ma sul vettore LOCALE
    q_rel_yaw, q_rel_pitch = vector_to_yaw_pitch(v_local) 
    
    push!(q_vm_components, q_rel_yaw)
    push!(q_vm_components, q_rel_pitch)
    
    # Aggiorniamo R_prev per il prossimo giro
    # La rotazione aggiunta è Z(q_rel_yaw) * X(q_rel_pitch)
    R_curr = R_prev * rot_z(q_rel_yaw) * rot_x(q_rel_pitch)
    global R_prev = R_curr
end

# 3. Slider start
push!(q_vm_components, BAR_LENGTH_TOTAL)

q_vm_initial = q_vm_components
println("   Stato iniziale VM calcolato ($length(q_vm_initial) DOF)")

# =======================================================================================
# --- 5. SIMULAZIONE E VISUALIZZAZIONE (Invariate) ---
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

# Creazione della figura principale
fig = Figure(size=(800, 800))

# --- MODIFICA FONDAMENTALE ---
# Questo comando apre la finestra grafica (backend GLMakie) permettendoti di vedere
# l'animazione in tempo reale mentre viene registrata, esattamente come in 2_3.jl. 
display(fig) 
# -----------------------------

ls = LScene(fig[1, 1]; show_axis=true)

# Configurazione Camera
cam = cam3d!(ls, camera=:perspective, center=false)
cam.lookat[] = [0.4, 0.1, 0.5]
cam.eyeposition[] = [1.7, 0.8, 1.0]

# Creazione dell'Observable per la cinematica (Reactive Programming)
pkcache = Observable(new_kinematics_cache(compile(vms))) 

# Visualizzazione Robot
robotvisualize!(ls, pkcache)

# Visualizzazione elementi statici (Pivot e Ostacoli)
mesh!(ls, Sphere(Point3f(PIVOT_POINT), 0.03), color=:green, label="Pivot")
#mesh!(ls, Sphere(Point3f(OBSTACLE_POSITION_1), 0.04), color=:orange, label="Ostacolo 1")
#mesh!(ls, Sphere(Point3f(OBSTACLE_POSITION_2), 0.04), color=:magenta, label="Ostacolo 2")
mesh!(ls, Rect3f(Point3f(OBSTACLE_POSITION_1) - Vec3f(0.04, 0.04, 0.2), Vec3f(0.08, 0.08, 0.2)), color=:orange, label="Ostacolo 1")
mesh!(ls, Rect3f(Point3f(OBSTACLE_POSITION_2) - Vec3f(0.04, 0.04, 0.2), Vec3f(0.08, 0.08, 0.2)), color=:magenta, label="Ostacolo 2")

# Visualizzazione Piano Z=0
mesh!(ls, Rect3f(Vec3f(-1.0, -1.0, -0.001), Vec3f(2.0, 2.0, 0.002)), color=(:blue, 0.1), transparency=true)

# --- Visualizzazione Snake Dinamica (Ricorsiva) ---
# Raccogliamo i punti della catena in una lista di Observable per tracciare le linee
points_obs = []

# 1. Pivot (Base)
pivot_id = get_compiled_coordID(compile(vms), ".virtual_mechanism.Pivot_Pos")
push!(points_obs, map(c -> Point3f(configuration(c, pivot_id)), pkcache))

# 2. Segmenti (1..N)
for i in 1:NUM_ACTUAL_SEGMENTS
    id = get_compiled_coordID(compile(vms), ".virtual_mechanism.Seg$(i)_Pos")
    push!(points_obs, map(c -> Point3f(configuration(c, id)), pkcache))
end

# Disegna linee connettendo i punti osservabili
for i in 1:(length(points_obs)-1) 
    # Segmento i-esimo (dal punto i al punto i+1)
    lines!(ls, map((p1, p2) -> [p1, p2], points_obs[i], points_obs[i+1]), 
           color=:red, linewidth=5)
end

# --- Visualizzazione Cursore ---
cursore_id = get_compiled_coordID(compile(vms), ".virtual_mechanism.SliderWorldPos_Ref")
scatter!(ls, map(c -> Point3f(configuration(c, cursore_id)), pkcache), 
         color=:blue, markersize=20, label="Target (Cursore)")

# Legenda (opzionale ma utile)
axislegend(ls, position=:rt)

# --- Esecuzione e Registrazione ---
savepath = joinpath(@__DIR__, "franka_recursive_snake.mp4")
println("Rendering in corso su finestra e salvataggio su file...")

animate_robot_odesolution(fig, sol, pkcache, savepath; f_setup, f_control) 

@info "Animazione completata e salvata: $savepath"