"""
user_guidance.py
----------------
This is the main file you will analyse and change for the SGN Guidance Challenge.

You are expected to work on two sections only:

  1. which_mission()  – Set the initial conditions for the scenario
                        (drone and target starting positions, velocities, target mode).

  2. my_guidance()    – Implement your guidance law for the seeker drone.
                        This is the core of the challenge.

Do NOT modify any other function. The target behaviour and the rest of the
software pipeline are managed by the organisers.

Available sensor readings (import core_data_access and call the functions below):

  CDA.get_drone_navdata()    →  [time, pos_x, pos_y, vel_x, vel_y, acc_x, acc_y, yaw]
  CDA.get_target_navdata()   →  [time, pos_x, pos_y, vel_x, vel_y, acc_x, acc_y]
  CDA.get_seeker_data()      →  [time, r, sigma]
  CDA.get_seeker_ext_data()  →  [time, r, sigma, r_dot, sigma_dot]

  where:
    r         – range between seeker and target [m]
    sigma     – Line-Of-Sight (LOS) angle [rad]
    r_dot     – range rate [m/s]
    sigma_dot – LOS angle rate [rad/s]
    yaw       – drone heading angle [rad]

Coordinate frame: Vicon Navigation Frame (planar, z = 0 for both drone and target).
"""

import math
import core_data_access as CDA
from logging_script import *
import numpy as np
from persistent_var import PersistentVariable 


# ---------------------------------------------------------------------------
# Logger setup — do not modify
# ---------------------------------------------------------------------------

# Number of custom variables you want to log (max 10)
private_guidance_data_length = 10

ff_time_old = PersistentVariable(0.0)
sigma_old = PersistentVariable(0.0)
ff_r = PersistentVariable(0.0)
ff_r_dot = PersistentVariable(0.0)
ff_sigma_dot = PersistentVariable(0.0)
ff_sigma = PersistentVariable(0.0)
ff_r_dot_raw = PersistentVariable(0.0)
ff_sigma_dot_raw = PersistentVariable(0.0)
ff_tan_acc_est = PersistentVariable(0.0)
ff_tan_acc_old = PersistentVariable(0.0)
ff_initialized = PersistentVariable(False)
ff_aT_perp_est = PersistentVariable(0.0)
ff_sigma_dot_old = PersistentVariable(0.0)
guidance_mode = PersistentVariable(2) # 0 = hybrid, 1 = pursuit, 2 = png, 3 = apng (change this initializaton to select a fixed guidance mode instead of the hybrid one)


# Persistent variables for target PNG-killer guidance law
target_last_switch = PersistentVariable(-1e9)
target_chosen_sign = PersistentVariable(1.0)
target_initialized = PersistentVariable(False)
 

# Storage array and async logger for your private data
private_guidance_data = np.zeros((private_guidance_data_length, 1))
private_logger = AsyncMatlabPrint(flag=9, num_data=private_guidance_data_length)



# ---------------------------------------------------------------------------
# SECTION 1 — Initial Conditions
# ---------------------------------------------------------------------------

def which_mission():
    """
    Define the initial conditions for the guidance scenario.

    Both drone and target move in the XY plane (z = 0) inside the Vicon room.
    The framework will automatically stop the simulation if the drone exits the
    allowed region or exceeds the velocity limit.

    Constraints
    -----------
    - Seeker (drone) speed  : ||vel|| ≤ 0.5 m/s
    - Tracking area (X)     : [-1.4,  1.3] m
    - Tracking area (Y)     : [-1.4,  1.7] m

    Target modes
    ------------
    Set `target_mode` to one of the following integers:

      0 – Stationary / uniformly moving target (no lateral acceleration)
      1 – Switching acceleration  →  edit acceleration_switcher() below
      2 – Intelligent target      →  edit target_guidance_law() below

    What to modify
    --------------
    - init_pos    : drone starting position [x, y] in metres
    - heading_error : initial heading offset from the desired angle [degrees]
    - v_norm      : drone axial speed magnitude [m/s]  (must stay ≤ 0.5)
    - in_p        : target initial position [x, y, 0] in metres
    - v_angle_t   : target initial heading [rad]
    - v_norm_t    : target axial speed magnitude [m/s]
    - target_mode : integer 0 / 1 / 2  (see above)

    Returns
    -------
    tuple
        (init_pos, init_vel, in_p, in_v, target_mode)
    """

    # --- Drone initial conditions -------------------------------------------

    init_pos = [-1.0 , -1.0]         # Starting position [x, y]  (metres)

    heading_error = 0               # Offset from the nominal heading angle [degrees]
                                    # Increase this to introduce an initial pointing error

    v_angle = ((45 - heading_error) * math.pi / 180.0)   # Heading angle [rad]
    v_norm  = 0.45                                        # Speed [m/s] — must be ≤ 0.5

    init_vel = np.array([v_norm * math.cos(v_angle),
                         v_norm * math.sin(v_angle)])

    # --- Target initial conditions -------------------------------------------

    in_p = np.array([1.0, 1.0, 0])                        # Position [x, y, 0]  (metres)

    v_angle_t = (180 * math.pi / 180.0)                     # Target heading [rad]
    v_norm_t  = 0.25                                        # Target speed [m/s]

    in_v = np.array([v_norm_t * math.cos(v_angle_t),
                     v_norm_t * math.sin(v_angle_t), 0])

    # --- Target mode ---------------------------------------------------------
    # 0: constant / no lateral acceleration
    # 1: switching acceleration  →  see acceleration_switcher()
    # 2: intelligent pursuit     →  see target_guidance_law()

    target_mode = 2          # ← CHANGE THIS to select the target behaviour

    return init_pos, init_vel, in_p, in_v, target_mode


# ---------------------------------------------------------------------------
# SECTION 2 — Seeker Guidance Law
# ---------------------------------------------------------------------------

def my_guidance(data: list):

    """
    Compute the lateral acceleration command for the seeker drone.

    This function is called at every simulation timestep by the framework.
    Your task is to implement a guidance law that drives the drone to intercept
    the target.

    Input argument
    --------------
    data : list
        Fading-filter estimates  [r, sigma, r_dot, sigma_dot]
        These are the same quantities available through get_seeker_ext_data(),
        but pre-filtered. You may use either source.

    Sensor readings available
    -------------------------
    Use the CDA functions at the top of this file.  Quick reference:

      ddata = CDA.get_drone_navdata()    # index  →  0:time  1:px  2:py  3:vx  4:vy  5:ax  6:ay  7:yaw
      tdata = CDA.get_target_navdata()   # index  →  0:time  1:px  2:py  3:vx  4:vy  5:ax  6:ay
      sdata = CDA.get_seeker_data()      # index  →  0:time  1:r   2:sigma
      edata = CDA.get_seeker_ext_data()  # index  →  0:time  1:r   2:sigma  3:r_dot  4:sigma_dot

    Acceleration limits
    -------------------
    The commanded acceleration must stay within [-0.75, +0.75] m/s².
    Values outside this range will be saturated.  ← CHANGE the saturation
    limits below if you want a tighter constraint for your strategy.

    Sign convention
    ---------------
    Positive acceleration  →  turn LEFT  (counter-clockwise)
    Negative acceleration  →  turn RIGHT (clockwise)

    Logger (optional)
    -----------------
    You can log up to 10 custom variables for post-run plotting.
    Example:

        global private_guidance_data, private_logger
        private_guidance_data[0] = CDA.get_drone_navdata()[0]   # time
        private_guidance_data[1] = CDA.get_seeker_data()[1]     # range r
        private_logger.append(private_guidance_data)

    Returns
    -------
    float
        Lateral acceleration command perpendicular to the drone velocity [m/s²]
    """
    
    # ------------------DA QUI INIZIA IL CODICE STANDARD DEL PROF----------------------
    # Declare globals to enable logging
    # global private_logger, private_guidance_data

    # Read sensor data
    ddata = CDA.get_drone_navdata()
    sdata = CDA.get_seeker_data()
    edata = CDA.get_seeker_ext_data()

    time_now   = float(sdata[0])
    r_meas     = float(sdata[1])
    sigma_meas = wrap(float(sdata[2]))
    yaw        = wrap(float(ddata[7]))

    # ------------------------------------------------------------------
    # 2nd-order fading filter on r and sigma
    # States:
    #   r, r_dot
    #   sigma, sigma_dot
    #
    # Important:
    #   sigma residual is wrapped to avoid jumps near +/-pi.
    # ------------------------------------------------------------------
    try:
        ff_initialized.get()
    except NameError:
        ff_initialized.set(False)

    if not ff_initialized.get():
        ff_r.set(max(r_meas, 1e-3))
        ff_sigma.set(sigma_meas)
        ff_initialized.set(True)

    dt = time_now - ff_time_old.get()
   
    if dt > 1e-5:
        beta_memory = 0.55
        G = 1.0 - beta_memory**2
        H = (1.0 - beta_memory)**2
   
        # --- r filter
        r_pred = ff_r.get() + dt * ff_r_dot.get()
        r_dot_pred = ff_r_dot.get()
        e_r = r_meas - r_pred
        ff_r.set(max(r_pred + G * e_r, 1e-3))
        ff_r_dot.set(r_dot_pred + H * e_r / dt)

        # --- sigma filter
        sigma_pred = wrap(ff_sigma.get() + dt * ff_sigma_dot.get())
        sigma_dot_pred = ff_sigma_dot.get()
        e_sigma = wrap(sigma_meas - sigma_pred)

        ff_sigma.set(wrap(sigma_pred + G * e_sigma))
        ff_sigma_dot.set(sigma_dot_pred + H * e_sigma / dt)

        sigma_ddot_raw = (ff_sigma_dot.get() - ff_sigma_dot_old.get())/dt
        a_rel_perp_raw = ff_r.get()*sigma_ddot_raw + 2.0 * ff_r_dot.get() * ff_sigma_dot.get()
        
        # derived and low pass filtered target acceleration
        tau_a = 0.35
        alpha_a = tau_a/(tau_a + dt)
        ff_aT_perp_est.set((alpha_a * ff_aT_perp_est.get() + (1.0 - alpha_a)*a_rel_perp_raw))
        
        ff_time_old.set(time_now)
        ff_sigma_dot_old.set(ff_sigma_dot.get())
    
    r = ff_r.get()
    r_dot = ff_r_dot.get()
    sigma = ff_sigma.get()
    sigma_dot = ff_sigma_dot.get()
    Vc = max(-r_dot, 0.0)

    # ------------------------------------------------------------------
    # EXAMPLE GUIDANCE LAW — Pure Pursuit
    # ------------------------------------------------------------------
    # The drone aligns its heading with the Line-Of-Sight (LOS) angle sigma.
    #
    # sigma_relative = yaw - sigma
    #   > 0  →  target is to the RIGHT  →  turn right  (negative acc)
    #   < 0  →  target is to the LEFT   →  turn left   (positive acc)
    #
    # Feel free to replace this block entirely with your own law.
    # ------------------------------------------------------------------

    sigma_r = wrap(ddata[7] - sdata[2])   # Heading error w.r.t. LOS [rad]
    

    # pure pursuit LOS alignment
    kp_pursuit = 1.8                              # Proportional gain — tune this value
    kd_pursuit = 0.1                              # Derivative gain
    acc_pursuit = -kp_pursuit * sigma_r - kd_pursuit * sigma_dot


    # PNG with bias
    N_png = 4.7
    acc_png = N_png * Vc * sigma_dot - 0.35 * sigma_r    

    # APNG with bias
    N_apng = 4.7
    aT_perp = ff_aT_perp_est.get()    
    acc_apng = N_apng * Vc * sigma_dot + 0.5 * N_apng * aT_perp - 0.35 * sigma_r

    # adaptive guidance
    # mode_IDs
    # 1 = pursuit 
    # 2 = PNG
    # 3 = APNG
    # 4 = blended PNG_APNG

    R_TERMINAL = 0.2
    APNG_ON = 0.1

    maneuver_level = abs(aT_perp)
    target_maneuvering = maneuver_level > APNG_ON

    
    if r < R_TERMINAL:
        if target_maneuvering:
            acc_hybrid = acc_apng
            print("using apng near to the target")
        else: 
            acc_hybrid = acc_png
            print("using png near to the target if it's not maneuvering")
    else: 
       acc_hybrid = acc_png
       print("using png far from the target")
    
    if guidance_mode.get() == 1:
        acc = acc_pursuit
    elif guidance_mode.get() == 2:
        acc = acc_png
    elif guidance_mode.get() == 3:
        acc = acc_apng
    else:
        acc = acc_hybrid
        
    
    acc_sat = 0.5
    if acc >  acc_sat:
        acc =  acc_sat
    if acc < -acc_sat:
        acc = -acc_sat
  

    # store private_guidance_data
    # EXAMPLE
    private_guidance_data[0] = ddata[0] #time
    private_guidance_data[1] = ddata[7] #yaw
    private_guidance_data[2] = sdata[2] #sigma
    private_guidance_data[3] = sigma_r  #sigma_r
    private_guidance_data[4] = acc #acc
    private_guidance_data[5] = sigma_dot
    private_guidance_data[6] = r_dot
    private_logger.append(private_guidance_data)

    return acc
    

    


# ---------------------------------------------------------------------------
# SECTION 3 — Target Behaviour  (modes 1 and 2)
# !! READ ONLY — provided for reference. Do NOT modify. !!
# ---------------------------------------------------------------------------

def target_guidance_function(mode, t_sim, yaw):
    """
    Dispatcher that selects the target acceleration based on the chosen mode.

    Parameters
    ----------
    mode  : int    Target mode (0 / 1 / 2) — set in which_mission()
    t_sim : float  Current simulation time [s]
    yaw   : float  Current target heading angle [rad]

    Returns
    -------
    float
        Target lateral acceleration command [m/s²]
    """
    if mode == 0:
        # Mode 0: no lateral acceleration — target moves straight
        acc = 0
        return acc

    elif mode == 1:
        # Mode 1: pre-programmed switching acceleration — see acceleration_switcher()
        acc = acceleration_switcher(t_sim)
        return acc

    elif mode == 2:
        # Mode 2: intelligent target using a guidance law — see target_guidance_law()
        acc = target_guidance_law(t_sim, yaw)
        return acc
    
    elif mode == 3:
        # Mode 3: target law based on miss distance (ZEM) — see target_miss_distance_escape()
        acc = target_miss_distance_escape(t_sim, yaw)
        return acc
    
    elif mode == 4:
        # Mode 3: PNG-killer intelligent target — see target_png_killer_law()
        acc = target_png_killer_law(t_sim)
        return acc
    
    # elif mode == 5:
    #     # Mode 3: PNG-killer intelligent target — see target_png_killer_law()
    #     acc = target_miss_distance_escape2(t_sim)
    #     return acc


def acceleration_switcher(t_sim):
    """
    Define a time-based switching lateral acceleration profile for the target (mode 1).

    The target applies different constant accelerations across three time windows.

    Parameters
    ----------
    t_sim : float   Current simulation time [s]

    Returns
    -------
    float
        Target lateral acceleration [m/s²]
    """

    t1_switch = 1    # End of first phase [s]
    t2_switch = 60   # End of second phase [s]

    if 0 < t_sim <= t1_switch:
        acc = 0.0          # Phase 1 acceleration
    elif t1_switch < t_sim <= t2_switch:
        acc =  -0.1            # Phase 2 acceleration
    else:
        acc = -0.2           # Phase 3 acceleration

    return acc


def target_guidance_law(t_sim, yaw):
    """
    Intelligent target guidance law (mode 2).

    The target actively tries to evade the seeker using a pursuit-based strategy
    combined with an Artificial Potential Field (APF) component that keeps the
    target away from the room boundaries.

    Parameters
    ----------
    t_sim : float   Current simulation time [s]
    yaw   : float   Current target heading angle [rad]

    Returns
    -------
    float
        Target lateral acceleration command [m/s²]  (saturated to [-0.5, +0.5])
    """

    # Read navigation data
    ddata = CDA.get_drone_navdata()
    tdata = CDA.get_target_navdata()
    sdata = CDA.get_seeker_data()

    # Target and drone positions
    t_pose_x   = tdata[1]
    t_pose_y   = tdata[2]
    drone_pose_x = ddata[1]
    drone_pose_y = ddata[2]

    # --- Pursuit component --------------------------------------------------
    # The target steers away from the seeker by reversing a pursuit law:
    # sigma (from target's perspective) is shifted by π to point away from drone.

    yaw   = wrap(yaw)
    sigma = wrap(math.pi + sdata[2])   # LOS from target toward drone [rad]
    sigma_r = wrap(yaw - sigma)        # Target heading error w.r.t. anti-LOS

    kp  = 1.5
    acc = kp * sigma_r

    # --- Last-ditch maneuver ------------------------------------------------
    # If the drone gets very close, apply a fixed evasive jink.

    target_pos = np.array([t_pose_x, t_pose_y])
    drone_pos  = np.array([drone_pose_x, drone_pose_y])
    rel_pos    = np.linalg.norm(drone_pos - target_pos, 2)   # Relative distance [m]

    if rel_pos < 0.2:
        acc = 0.3

    # Acceleration saturation
    if acc >  0.5:
        acc =  0.5
    elif acc < -0.5:
        acc = -0.5

    # --- Artificial Potential Field (APF) component -------------------------
    # If the target is close to the room boundary, override with a fixed
    # repulsive acceleration to prevent it from exiting the tracking area.

    x_max = 1.4   # Half-width of tracking area  [m]
    y_max = 1.4   # Half-height of tracking area [m]

    threshold = 0.7   # APF activates when |pos| > threshold * x/y_max
                      # (e.g. 0.7 means the APF kicks in at 70 % of the boundary)

    if (math.fabs(t_pose_x) > x_max * threshold or
            math.fabs(t_pose_y) > y_max * threshold):
        acc = 0.35

    return acc


# TARGET LAW BASED ON MISS DISTANCE

def target_miss_distance_escape(t_sim, yaw):

    # Read navigation data
    ddata = CDA.get_drone_navdata()
    tdata = CDA.get_target_navdata()
    sdata = CDA.get_seeker_data()
    edata = CDA.get_seeker_ext_data()

    # Target positions
    t_pose_x   = tdata[1]
    t_pose_y   = tdata[2]
    
    # 1. Miss distance (ZEM)
    r         = float(edata[1])
    r_dot     = float(edata[3])
    sigma_dot = float(edata[4])
    
    Vc = max(-r_dot, 0.001) 
    miss_distance = (r**2 * sigma_dot) / Vc

    yaw   = wrap(yaw)
    sigma = wrap(math.pi + sdata[2])
    drone_relative_angle = wrap(sigma - yaw) 
    # Se > 0: il drone è a SINISTRA del target
    # Se < 0: il drone è a DESTRA del target
    
    # 3. LOGICA EVASIVA BASATA SU MISS DISTANCE E RANGE
    if abs(miss_distance) < 0.3 and Vc > 0.1 and r < 1.2:
        # Se ZEM è critico e il drone è abbastanza vicino, 
        # vira con la MASSIMA accelerazione VERSO il drone per forzare l'overshoot.
        if drone_relative_angle > 0:
            acc = 0.5  # Drone a sx -> Vira tutto a sx
        else:
            acc = -0.5 # Drone a dx -> Vira tutto a dx
        
    elif r < 0.5:
        # Fase 2 (Last-ditch): Il drone è letteralmente addosso.
        # INVERTI brutalmente la virata per fare uno scarto all'ultimo istante.
        if drone_relative_angle > 0:
            acc = -0.5  # Drone a sx -> scarta a dx
        else:
            acc = 0.5   # Drone a dx -> scarta a sx
        
    else:
        # Navigazione tattica: invece di dare le spalle, cerca di tenere 
        # il drone a 90 gradi (di fianco). Questo costringe la ProNav del drone
        # a faticare costantemente per chiudere la curva.
        desired_relative = math.pi/2 if drone_relative_angle > 0 else -math.pi/2
        error = wrap(drone_relative_angle - desired_relative)
        kp  = 1.5
        acc = kp * error

    # Acceleration saturation
    if acc >  0.5:
        acc =  0.5
    elif acc < -0.5:
        acc = -0.5

    # --- Artificial Potential Field (APF) component -------------------------
    # If the target is close to the room boundary, override with a fixed
    # repulsive acceleration to prevent it from exiting the tracking area.

    x_max = 1.4   # Half-width of tracking area  [m]
    y_max = 1.4   # Half-height of tracking area [m]

    threshold = 0.7   # APF activates when |pos| > threshold * x/y_max
                      # (e.g. 0.7 means the APF kicks in at 70 % of the boundary)

    if (math.fabs(t_pose_x) > x_max * threshold or
            math.fabs(t_pose_y) > y_max * threshold):
        acc = 0.35

    return acc

def target_png_killer_law(t_sim):
    """
    Intelligent target guidance law translated from the MATLAB function
    target_intelligente_png_killer.

    This target tries to make PNG difficult by:
      - activating only when the seeker is close / dangerous;
      - estimating closing velocity and time-to-go;
      - comparing two candidate target accelerations: +ACC_MAX and -ACC_MAX;
      - choosing the maneuver that maximizes a score based on future range
        and future LOS-rate magnitude;
      - adding a dwell-time logic to avoid changing maneuver too fast.

    Parameters
    ----------
    t_sim : float
        Current simulation time [s]

    Returns
    -------
    float
        Target lateral acceleration command [m/s²]
    """

    global target_last_switch, target_chosen_sign, target_initialized

    # ============================================================
    # Main parameters
    # ============================================================

    ACC_MAX = 0.2       # [m/s^2] target maximum lateral acceleration

    R_ON = 0.65         # [m] start intelligent evasion
    R_PANIC = 0.30      # [m] aggressive evasion
    TGO_ON = 2.0        # [s] evade if estimated intercept is within 2 s

    DWELL_TIME = 0.45   # [s] minimum time between maneuver sign changes

    TAU_MIN = 0.25      # [s] minimum prediction horizon
    TAU_MAX = 0.80      # [s] maximum prediction horizon

    K_SIGMA = 0.04      # weight to also disturb sigma_dot

    EPS_R = 1e-6
    EPS_VC = 1e-6
    EPS_VT = 1e-6

    # ============================================================
    # Persistent initialization
    # ============================================================

    if not target_initialized.get():
        target_last_switch.set(-1e9)
        target_chosen_sign.set(1.0)
        target_initialized.set(True)

    # Default: no maneuver
    acc = 0.0

    # ============================================================
    # Read navigation data
    # ============================================================

    ddata = CDA.get_drone_navdata()
    tdata = CDA.get_target_navdata()

    # Relative position: R = P_T - P_M
    R_x = float(tdata[1] - ddata[1])
    R_y = float(tdata[2] - ddata[2])

    # Absolute velocities
    VM_x = float(ddata[3])
    VM_y = float(ddata[4])

    VT_x = float(tdata[3])
    VT_y = float(tdata[4])

    # ============================================================
    # Relative geometry
    # ============================================================

    R = math.sqrt(R_x**2 + R_y**2)

    if R < EPS_R:
        acc = ACC_MAX
        return acc

    # Relative velocity coherent with R = P_T - P_M
    V_rel_x = VT_x - VM_x
    V_rel_y = VT_y - VM_y

    # Range derivative
    R_dot = (R_x * V_rel_x + R_y * V_rel_y) / R

    # Closing velocity: positive if the missile/drone is approaching
    Vc = -R_dot

    if Vc > EPS_VC:
        t_go = R / Vc
    else:
        t_go = 1e6

    # Instantaneous LOS rate from relative geometry
    sigma_dot = (R_x * V_rel_y - R_y * V_rel_x) / (R**2)

    # ============================================================
    # Threat evaluation
    # ============================================================

    threat = False

    if Vc > EPS_VC:
        if R < R_ON:
            threat = True

        if t_go < TGO_ON:
            threat = True

    if not threat:
        acc = 0.0
        return acc

    # ============================================================
    # Compute beta from target velocity components
    # ============================================================

    VT_norm = math.sqrt(VT_x**2 + VT_y**2)

    if VT_norm < EPS_VT:
        beta = 0.0
    else:
        beta = math.atan2(VT_y, VT_x)

    # Left normal to target velocity.
    # If acc > 0:
    # a_T = acc * [-sin(beta); cos(beta)]
    nTx = -math.sin(beta)
    nTy = math.cos(beta)

    # Prediction horizon
    tau = 0.35 * t_go

    if tau < TAU_MIN:
        tau = TAU_MIN
    elif tau > TAU_MAX:
        tau = TAU_MAX

    # ============================================================
    # Candidate 1: acc = +ACC_MAX
    # ============================================================

    acc1 = ACC_MAX

    aT1_x = acc1 * nTx
    aT1_y = acc1 * nTy

    R1_x = R_x + V_rel_x * tau + 0.5 * aT1_x * tau**2
    R1_y = R_y + V_rel_y * tau + 0.5 * aT1_y * tau**2

    V1_x = V_rel_x + aT1_x * tau
    V1_y = V_rel_y + aT1_y * tau

    R1 = math.sqrt(R1_x**2 + R1_y**2) + EPS_R

    sigma_dot_1 = (R1_x * V1_y - R1_y * V1_x) / (R1**2)

    score1 = R1 + K_SIGMA * math.fabs(sigma_dot_1)

    # ============================================================
    # Candidate 2: acc = -ACC_MAX
    # ============================================================

    acc2 = -ACC_MAX

    aT2_x = acc2 * nTx
    aT2_y = acc2 * nTy

    R2_x = R_x + V_rel_x * tau + 0.5 * aT2_x * tau**2
    R2_y = R_y + V_rel_y * tau + 0.5 * aT2_y * tau**2

    V2_x = V_rel_x + aT2_x * tau
    V2_y = V_rel_y + aT2_y * tau

    R2 = math.sqrt(R2_x**2 + R2_y**2) + EPS_R

    sigma_dot_2 = (R2_x * V2_y - R2_y * V2_x) / (R2**2)

    score2 = R2 + K_SIGMA * math.fabs(sigma_dot_2)

    # ============================================================
    # Choose best maneuver
    # ============================================================

    if score1 >= score2:
        best_sign = 1.0
        second_sign = -1.0
        best_score = score1
        second_score = score2
    else:
        best_sign = -1.0
        second_sign = 1.0
        best_score = score2
        second_score = score1

    # ============================================================
    # Jinking: sign changes to disturb sigma_dot estimation
    # ============================================================

    chosen_sign = target_chosen_sign.get()
    last_switch = target_last_switch.get()

    if R < R_PANIC:
        chosen_sign = best_sign
        last_switch = t_sim
    else:
        if (t_sim - last_switch) >= DWELL_TIME:

            if second_score > 0.88 * best_score:
                chosen_sign = second_sign
            else:
                chosen_sign = best_sign

            last_switch = t_sim

    # If sigma_dot is almost zero, PNG is doing well.
    # Change sign to break the collision triangle.
    if math.fabs(sigma_dot) < 0.02 and R < R_ON and Vc > EPS_VC:
        if (t_sim - last_switch) >= 0.5 * DWELL_TIME:
            chosen_sign = -chosen_sign
            last_switch = t_sim

    target_chosen_sign.set(chosen_sign)
    target_last_switch.set(last_switch)

    acc = ACC_MAX * chosen_sign

    return acc


# ---------------------------------------------------------------------------
# Utility
# ---------------------------------------------------------------------------

def wrap(angle):
    """
    Wrap an angle to the interval (-π, π].

    Parameters
    ----------
    angle : float   Input angle [rad]

    Returns
    -------
    float
        Equivalent angle in (-π, π]  [rad]
    """
    while angle >  math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle
