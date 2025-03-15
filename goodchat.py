import math
import matplotlib.pyplot as plt
import numpy as np

# --- Constants and Configurations ---
# Simulation time settings
TIME_STEP = 1          # seconds
SESSION_LENGTH = 3600  # total simulation duration in seconds

# Vehicle parameters
CAR_WEIGHT = 65        # kg
DRIVER_WEIGHT = 65     # kg
TOTAL_WEIGHT = CAR_WEIGHT + DRIVER_WEIGHT
G = 9.81               # m/s^2

# Aerodynamic parameters
RHO = 1.225            # Air density (kg/m^3)
CSA = 0.1535           # Cross-sectional area (m^2)
C_D = 1.07             # Drag coefficient
C_L = 0.89             # Lift coefficient
C_S = 0.011            # Skin friction coefficient

# Tyre parameters
TYRE_PRESSURE_PSI = 40
TYRE_PRESSURE = TYRE_PRESSURE_PSI / 14.504   # convert psi to bar
TYRE_DIAMETER = 0.5    # m

# Battery parameters
V_MAX = 24             # Maximum voltage
V_MIN = 18             # Minimum voltage
H = 20                 # Rated discharge time in hours
C_CAP = 36             # Battery capacity in Ah
K_PEUKERT = 1.2        # Peukert's constant
BATTERY_WH = 1000      # Battery capacity in Wh

# Motor parameters
MOTOR_EFFICIENCY = 0.9
MOTOR_INITIAL_RESISTANCE = 0.063  # Ohms
NO_LOAD_CURRENT = 0.5
ORIGINAL_TORQUE_CONSTANT = 0.104
BACK_EMF_CONSTANT = 1.15
I_STALL_MAX = 130
RPM_DESIRED = 1650  # Target motor rpm (from data/graph)
# Initial gear ratio will be calibrated later
DIA_MOTOR_GEAR = 1
DIA_AXLE_GEAR = 2.75
GEAR_RATIO = DIA_AXLE_GEAR / DIA_MOTOR_GEAR

# Elevation map: Each entry is [lap_progress, angle (radians), direction factor]
ELEVATION_MAP = [
    [10, 0, 1],
    [35, math.pi/700, 1],
    [45, math.pi/800, 1],
    [70, 0, 1],
    [100, math.pi/600, -1],
    [110, math.pi/500, -1],
    [135, math.pi/550, -1],
    [160, 0, 1],
    [200, math.pi/1000, 1],
    [220, math.pi/1000, -1],
    [250, math.pi/800, -1],
    [275, math.pi/750, -1]
]

# --- Functions ---
def aero_forces(velocity):
    """
    Calculate aerodynamic forces: skin friction, drag, and lift.
    
    Args:
        velocity (float): Vehicle speed in m/s.
    
    Returns:
        tuple: (skin_friction, drag, lift) in Newtons.
    """
    dynamic_pressure = 0.5 * RHO * velocity**2
    skin_friction = C_S * dynamic_pressure * CSA
    drag = C_D * dynamic_pressure * CSA
    lift = C_L * dynamic_pressure * CSA
    return skin_friction, drag, lift

def rolling_resistance(lift, velocity):
    """
    Calculate rolling resistance force.
    
    Args:
        lift (float): Lift force (N).
        velocity (float): Vehicle speed (m/s).
    
    Returns:
        float: Rolling resistance force in N.
    """
    c_rr = 0.005 + 1 / TYRE_PRESSURE * (0.01 + 0.0095 * ((velocity * 3.6 / 100) ** 2))
    F_rr = c_rr * (TOTAL_WEIGHT * G - lift)
    return F_rr

def track_elevation(lap_progress):
    """
    Determine the downhill force based on track elevation.
    
    Args:
        lap_progress (float): Position along the track.
    
    Returns:
        float: Downhill force (N) due to track gradient.
    """
    pointer = 0
    while pointer + 1 < len(ELEVATION_MAP) and lap_progress > ELEVATION_MAP[pointer + 1][0]:
        pointer += 1
    angle = ELEVATION_MAP[pointer][1]
    direction = ELEVATION_MAP[pointer + 1][2]
    downhill_force = TOTAL_WEIGHT * G * math.sin(angle) * direction
    return downhill_force

def calculate_motor_force_from_torque(motor_torque, motor_rpm):
    """
    Calculate force at the wheels given a motor torque.
    
    Args:
        motor_torque (float): Motor torque in Nm.
        motor_rpm (float): Motor speed in rpm.
    
    Returns:
        tuple: (wheel_rpm, wheel_torque, motor_force, max_motor_velocity)
            where: wheel_rpm (rpm), wheel_torque (Nm), motor_force is the force exerted by the motor (N),
            max_motor_velocity is the maximum velocity that the car can achieve in this configuration (m/s).
    """
    wheel_rpm = motor_rpm / GEAR_RATIO
    wheel_torque = GEAR_RATIO * motor_torque 
    motor_force = wheel_torque / (TYRE_DIAMETER/2) # Force = moment/distance
    max_motor_velocity = wheel_rpm * (math.pi * TYRE_DIAMETER) / 60 # Max velocity achievable by the car
    return wheel_rpm, wheel_torque, motor_force, max_motor_velocity


def calc_motor_parameters(temperature, voltage):
    """
    Adjust motor parameters based on temperature.
    
    Args:
        temperature (float): Motor temperature in °C.
        voltage (float): Battery voltage (V).
    
    Returns:
        tuple: (P_max, no_load, T_stall, I_stall)
            where P_max is maximum power (W), no_load is a parameter (rpm-equivalent),
            T_stall is stall torque (Nm), and I_stall is stall current (A).
    """
    motor_resistance = MOTOR_INITIAL_RESISTANCE * (1 + 0.004 * (temperature - 20))
    I_stall = voltage / motor_resistance  # simplified stall current estimate
    torque_constant = ORIGINAL_TORQUE_CONSTANT * (1 - 0.0012 * (temperature - 20))
    T_stall = I_stall * torque_constant
    no_load = 9.5493 * (voltage - (NO_LOAD_CURRENT * motor_resistance)) / torque_constant
    omega = voltage / torque_constant
    P_max = 0.25 * omega * T_stall  # rough estimation of maximum power
    return P_max, no_load, T_stall, I_stall

def motor_graph_conversion(battery_power, P_max, no_load, T_stall, I_stall):
    """
    Estimate motor torque, current, and rpm from battery power using a simplified conversion.
    
    Args:
        battery_power (float): Available battery power (W).
        P_max (float): Maximum motor power (W).
        no_load (float): No-load parameter.
        T_stall (float): Stall torque (Nm).
        I_stall (float): Stall current (A).
    
    Returns:
        tuple: (motor_torque (Nm), current (A), motor_rpm)
    """
    # Adjust battery power to account for losses (assumed 70% effective)
    motor_power = battery_power * 0.7
    motor_torque = -math.sqrt(max(0, (-(motor_power / (P_max / ((T_stall / 2) ** 2))) - (T_stall / 2) ** 2))) + T_stall / 2

    current = motor_torque * I_stall / T_stall + NO_LOAD_CURRENT
    motor_rpm = (-no_load / T_stall) * motor_torque + no_load
    return motor_torque, current, motor_rpm

def battery_update(SoC, voltage, current, battery_power, t):
    """
    Update battery state (SoC and voltage) using a simplified Peukert model and voltage decay curve.
    
    Args:
        SoC (float): State of Charge (%).
        voltage (float): Battery voltage (V).
        current (float): Battery current (A).
        battery_power (float): Battery power (W).
        t (int): Current simulation time in seconds.
    
    Returns:
        tuple: Updated (current, SoC, voltage).
    """
    t_discharge = H * (C_CAP / (current * H)) ** K_PEUKERT
    SoC -= SoC * TIME_STEP / (t_discharge * 3600)
    
    # Voltage model: exponential decay then a quadratic transition for smoothness
    if t < 3500:
        voltage = 18 + 6 * np.exp(-np.log(2) * (t / 3500) ** 2)
    else:
        A = -3.84e-6
        B = -0.00257
        voltage = 21 + B * (t - 3500) + A * (t - 3500) ** 2
    return current, SoC, voltage

def update_velocity_distance(velocity, acceleration, distance, max_velocity, moving):
    """
    Update vehicle velocity and distance using Euler integration.
    
    Args:
        velocity (float): Current velocity (m/s).
        acceleration (float): Acceleration (m/s^2).
        distance (float): Distance traveled (m).
        max_velocity (float): Speed limit based on available power (m/s).
    
    Returns:
        tuple: Updated (velocity, distance).
    """
    u = velocity
    new_velocity = u + acceleration * TIME_STEP
    new_velocity = min(new_velocity, max_velocity)
    if new_velocity == 0:
        moving  = False  
    distance += TIME_STEP * (u + new_velocity) / 2  # s = (u + v)/2 * t
    return new_velocity, distance, moving

def find_max_speed(max_power, velocity, lap_progress):
    """
    Iteratively determine the maximum achievable speed for the given available power.
    
    Args:
        max_power (float): Available power (W).
        velocity (float): Starting velocity (m/s).
        lap_progress (float): Track progress (for elevation).
    
    Returns:
        tuple: (skin_friction, drag, lift, F_rr, downhill_force, total_force, power_required, max_velocity)
    """
    while True:
        skin_friction, drag, lift = aero_forces(velocity)
        F_rr = rolling_resistance(lift, velocity)
        downhill_force = track_elevation(lap_progress)
        total_force = skin_friction + drag + F_rr + downhill_force
        power_required = total_force * velocity
        if power_required >= max_power:
            break
        velocity += 0.01
    return skin_friction, drag, lift, F_rr, downhill_force, total_force, power_required, velocity

def calc_wheel_rpm(velocity):
    """
    Calculate the wheel RPM given vehicle speed.
    
    Args:
        velocity (float): Vehicle speed (m/s).
    
    Returns:
        float: Wheel RPM.
    """
    return velocity * 60 / (math.pi * TYRE_DIAMETER)

# --- Simulation Initialization ---
time = 0
distance = 0
lap_progress = 0
moving = True
velocity = 0
motor_temperature = 20
SoC = 100
voltage = V_MAX
current = 0
battery_power = BATTERY_WH  # initial battery power available

# Lists for storing simulation results
times = []
velocities = []
distances = []
socs = []
voltages = []
currents = []
powers = []

# Calibrate gear ratio based on a target max speed from available power
skin_friction, drag, lift, F_rr, downhill_force, total_force, power_req, velocity_max = find_max_speed(450, velocity, lap_progress)
print("Calculated max speed:", velocity_max, "m/s with power requirement:", power_req)
wheel_rpm = calc_wheel_rpm(velocity_max)
gear_ratio = RPM_DESIRED / wheel_rpm
GEAR_RATIO = gear_ratio
print("Optimised gear ratio:", GEAR_RATIO)

# --- Main Simulation Loop ---
for t in range(SESSION_LENGTH):
    # Update battery power degradation over time using a placeholder function
    if t < 3000:
        battery_power = -1/15 * t + 700
    else:
        battery_power = -0.00056 * t**2 + 3.33 * t - 4450 - 0.1
    
    # Update motor temperature (linear increase over time)
    motor_temperature += 60 / 3600  # increases by 60°C per hour
    
    # Update motor parameters based on current temperature and voltage
    P_max, no_load, T_stall, I_stall = calc_motor_parameters(motor_temperature, voltage)
    
    # Convert battery power to motor torque, current, and rpm using a simplified conversion
    motor_torque, current, motor_rpm = motor_graph_conversion(battery_power, P_max, no_load, T_stall, I_stall)
    
    # Update battery state (SoC and voltage)
    current, SoC, voltage = battery_update(SoC, voltage, current, battery_power, t)
    
    # Calculate wheel parameters and forces
    wheel_rpm = calc_wheel_rpm(velocity)
    wheel_rpm, wheel_torque, motor_force, max_motor_velocity = calculate_motor_force_from_torque(motor_torque, motor_rpm)
    
    # Determine the maximum allowable speed for available power (accounting for losses)
    skin_friction, drag, lift, F_rr, downhill_force, total_force, power_required, max_velocity = find_max_speed(battery_power * 0.7, velocity, lap_progress)
    
    # Update lap progress (simulate looping track; here 300 units per lap)
    lap_progress = (lap_progress + 1) % 300
    
    # Calculate net force (motor force minus resistive forces) and acceleration
    net_force = motor_force - (skin_friction + drag + F_rr + downhill_force)
    acceleration = net_force / TOTAL_WEIGHT
    
    # Update vehicle velocity and traveled distance
    velocity, distance, moving = update_velocity_distance(velocity, acceleration, distance, max_velocity, moving)
    
    # Log simulation data
    times.append(t)
    velocities.append(velocity)
    distances.append(distance)
    socs.append(SoC)
    voltages.append(voltage)
    currents.append(current)
    powers.append(battery_power)
    
    # Break the loop if the vehicle stops
    if velocity <= 0:
        moving = False
        break

print("Final velocity:", velocity, "m/s")
print("Final SoC:", SoC)
print("Total distance traveled:", distance, "m")
print("Final acceleration:", acceleration, "m/s^2")
print("Calibrated gear ratio:", GEAR_RATIO)

# --- Plotting Results ---
fig, axs = plt.subplots(2, 3, figsize=(14, 8))
axs[0, 0].plot(times, currents)
axs[0, 0].set_title('Current (A)')
axs[0, 1].plot(times, velocities, 'tab:orange')
axs[0, 1].set_title('Velocity (m/s)')
axs[1, 0].plot(times, voltages, 'tab:green')
axs[1, 0].set_title('Voltage (V)')
axs[1, 1].plot(times, powers, 'tab:red')
axs[1, 1].set_title('Battery Power (W)')
axs[1, 2].plot(times, socs, 'tab:blue')
axs[1, 2].set_title('State of Charge (%)')
plt.tight_layout()
plt.show()
