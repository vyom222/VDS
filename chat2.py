import math
import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import UnivariateSpline

# --- Constants & Configuration ---
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

# Battery parameters (simplified)
V_MAX = 24             # Maximum voltage (V)
V_MIN = 18             # Minimum voltage (V)
H = 20                 # Rated discharge time (hours)
C_CAP = 36             # Battery capacity (Ah)
K_PEUKERT = 1.2        # Peukert's constant
BATTERY_WH = 1000      # Battery capacity in Wh

# Motor parameters (simplified)
MOTOR_EFFICIENCY = 0.9
MOTOR_INITIAL_RESISTANCE = 0.063  # Ohm
NO_LOAD_CURRENT = 0.5
ORIGINAL_TORQUE_CONSTANT = 0.104
RPM_DESIRED = 1650
# Initial gear ratio (calibrated later)
DIA_MOTOR_GEAR = 1
DIA_AXLE_GEAR = 2.75
GEAR_RATIO = DIA_AXLE_GEAR / DIA_MOTOR_GEAR

# --- Interpolated Elevation Profile ---
# Define lap progress (0 to 300 units) and corresponding gradient angles (in radians)
lap_positions = np.array([0, 10, 35, 45, 70, 100, 110, 135, 160, 200, 220, 250, 275, 300])
# Gradient angles from original data (positive for uphill, negative for downhill)
gradient_angles = np.array([
    0,
    0,
    math.pi/700,   # ~0.00449 rad uphill
    math.pi/800,   # ~0.00393 rad uphill
    0,
    -math.pi/600,  # ~-0.00524 rad downhill
    -math.pi/500,  # ~-0.00628 rad downhill
    -math.pi/550,  # ~-0.00571 rad downhill
    0,
    math.pi/1000,  # ~0.00314 rad uphill
    -math.pi/1000, # ~-0.00314 rad downhill
    -math.pi/800,  # ~-0.00393 rad downhill
    -math.pi/750,  # ~-0.00419 rad downhill
    0
])
# Create a cubic spline interpolation (no smoothing for exact interpolation)
elevation_spline = UnivariateSpline(lap_positions, gradient_angles, k=3, s=0)

def get_gradient(lap_progress):
    """
    Returns a smooth gradient (slope angle in radians) at the given lap progress.
    """
    progress = lap_progress % 300  # loop the lap progress over 300 units
    return elevation_spline(progress)

# --- Dynamic Wind Model ---
def dynamic_wind(t):
    """
    Returns a time-varying wind speed (m/s) and fixed wind angle (radians).
    For demonstration, wind speed oscillates (gusts) with a period of 60 seconds.
    """
    base_wind_speed = 5  # m/s base wind speed
    gust_amplitude = 3   # m/s amplitude of gusts
    period = 60          # seconds
    wind_speed = base_wind_speed + gust_amplitude * math.sin(2 * math.pi * t / period)
    wind_angle = math.pi / 4  # 45° relative to vehicle's forward direction
    return wind_speed, wind_angle

# --- Aerodynamic Forces with Wind ---
def aero_forces(velocity, wind_speed=0, wind_angle=0):
    """
    Calculate aerodynamic forces (skin friction, drag, lift) accounting for wind.
    The effective relative velocity is computed as the vector difference between
    the vehicle's speed and the wind's components.
    """
    # Assume vehicle moves along x-axis; vehicle velocity = [velocity, 0]
    # Wind components:
    wind_vx = wind_speed * math.cos(wind_angle)
    wind_vy = wind_speed * math.sin(wind_angle)
    # Relative velocity components:
    rel_vx = velocity - wind_vx
    rel_vy = 0 - wind_vy
    effective_velocity = math.sqrt(rel_vx**2 + rel_vy**2)
    
    dynamic_pressure = 0.5 * RHO * effective_velocity**2
    skin_friction = C_S * dynamic_pressure * CSA
    drag = C_D * dynamic_pressure * CSA
    lift = C_L * dynamic_pressure * CSA
    return skin_friction, drag, lift

# --- Other Helper Functions (similar to previous examples) ---
def rolling_resistance(lift, velocity):
    c_rr = 0.005 + 1 / TYRE_PRESSURE * (0.01 + 0.0095 * ((velocity * 3.6 / 100) ** 2))
    F_rr = c_rr * (TOTAL_WEIGHT * G - lift)
    return F_rr

def calc_wheel_rpm(velocity):
    return velocity * 60 / (math.pi * TYRE_DIAMETER)

def update_velocity_distance(velocity, acceleration, distance, max_velocity):
    u = velocity
    new_velocity = u + acceleration * TIME_STEP
    new_velocity = min(new_velocity, max_velocity)
    new_velocity = max(new_velocity, 0)
    distance += TIME_STEP * (u + new_velocity) / 2
    return new_velocity, distance

def battery_update(SoC, voltage, battery_current, t):
    t_discharge = H * (C_CAP / (battery_current * H)) ** K_PEUKERT
    SoC -= SoC * TIME_STEP / (t_discharge * 3600)
    # Simplified voltage decay
    if t < 3500:
        voltage = 18 + 6 * np.exp(-np.log(2) * (t / 3500) ** 2)
    else:
        A_fixed = -3.84e-6
        B_fixed = -0.00257
        voltage = 21 + B_fixed * (t - 3500) + A_fixed * (t - 3500) ** 2
    return battery_current, SoC, voltage

def calc_motor_parameters(temperature, voltage):
    motor_resistance = MOTOR_INITIAL_RESISTANCE * (1 + 0.004 * (temperature - 20))
    I_stall = voltage / motor_resistance
    torque_constant = ORIGINAL_TORQUE_CONSTANT * (1 - 0.0012 * (temperature - 20))
    T_stall = I_stall * torque_constant
    no_load = 9.5493 * (voltage - (NO_LOAD_CURRENT * motor_resistance)) / torque_constant
    omega = voltage / torque_constant
    P_max = 0.25 * omega * T_stall
    return P_max, no_load, T_stall, I_stall

def motor_graph_conversion(input_power, P_max, no_load, T_stall, I_stall):
    effective_power = input_power * 0.7
    try:
        motor_torque = -math.sqrt(max(0, (-(effective_power / (P_max / ((T_stall / 2) ** 2))) - (T_stall / 2) ** 2))) + T_stall / 2
    except ValueError:
        motor_torque = 0
    current = motor_torque * I_stall / T_stall + NO_LOAD_CURRENT
    motor_rpm = (-no_load / T_stall) * motor_torque + no_load
    return motor_torque, current, motor_rpm

# --- Simulation Initialization ---
time = 0
distance = 0
lap_progress = 0
velocity = 0
motor_temperature = 20
SoC = 100
voltage = V_MAX

# Lists for storing simulation data
times, velocities, distances, socs, voltages, currents, powers = [], [], [], [], [], [], []

# Calibrate gear ratio (simple calibration using a small nonzero velocity)
wheel_rpm = calc_wheel_rpm(velocity + 1e-3)
calibrated_gear_ratio = RPM_DESIRED / (wheel_rpm if wheel_rpm else 1)
GEAR_RATIO = calibrated_gear_ratio

# --- Main Simulation Loop with Spline Elevation & Dynamic Wind ---
for t in range(SESSION_LENGTH):
    # Get dynamic wind conditions
    wind_speed, wind_angle = dynamic_wind(t)
    
    # Calculate aerodynamic forces with wind influence
    skin_friction, drag, lift = aero_forces(velocity, wind_speed, wind_angle)
    
    # Rolling resistance remains as before
    F_rr = rolling_resistance(lift, velocity)
    
    # Compute downhill/uphill force using the smooth elevation profile
    gradient = get_gradient(lap_progress)
    downhill_force = TOTAL_WEIGHT * G * math.sin(gradient)
    
    # Total resistive force
    F_resist = skin_friction + drag + F_rr + downhill_force
    
    # Estimate required tractive force: add a target acceleration (e.g., 0.2 m/s²)
    target_acceleration = 0.2
    F_required = TOTAL_WEIGHT * target_acceleration + F_resist
    required_power = F_required * velocity if velocity > 0.1 else 100
    motor_input_power = required_power / MOTOR_EFFICIENCY
    
    # Battery current draw from motor demand
    battery_current = motor_input_power / voltage if voltage > 0 else 0.1
    battery_current = max(battery_current, 0.1)
    
    # Update battery state
    battery_current, SoC, voltage = battery_update(SoC, voltage, battery_current, t)
    
    # Update motor performance based on new voltage and temperature
    P_max, no_load, T_stall, I_stall = calc_motor_parameters(motor_temperature, voltage)
    motor_torque, motor_current, motor_rpm = motor_graph_conversion(motor_input_power, P_max, no_load, T_stall, I_stall)
    
    # Motor force at the wheels
    motor_force = (GEAR_RATIO * motor_torque) / (TYRE_DIAMETER / 2)
    
    # Net force and acceleration
    net_force = motor_force - F_resist
    acceleration = net_force / TOTAL_WEIGHT
    
    # Update vehicle state
    max_velocity = velocity * 1.2 + 1  # Arbitrary max velocity function
    velocity, distance = update_velocity_distance(velocity, acceleration, distance, max_velocity)
    
    # Update lap progress (simulate a looping track over 300 units)
    lap_progress = (lap_progress + 1) % 300
    
    # Update motor temperature (simplified linear increase)
    motor_temperature += 60 / 3600
    
    # Log data for plotting
    times.append(t)
    velocities.append(velocity)
    distances.append(distance)
    socs.append(SoC)
    voltages.append(voltage)
    currents.append(battery_current)
    powers.append(motor_input_power)
    
    if velocity <= 0:
        break

print("Final velocity:", velocity, "m/s")
print("Final SoC:", SoC)
print("Total distance traveled:", distance, "m")
print("Final acceleration:", acceleration, "m/s^2")

# --- Plotting Results ---
fig, axs = plt.subplots(2, 3, figsize=(14, 8))
axs[0, 0].plot(times, currents)
axs[0, 0].set_title('Battery Current (A)')
axs[0, 1].plot(times, velocities, 'tab:orange')
axs[0, 1].set_title('Velocity (m/s)')
axs[1, 0].plot(times, voltages, 'tab:green')
axs[1, 0].set_title('Voltage (V)')
axs[1, 1].plot(times, powers, 'tab:red')
axs[1, 1].set_title('Motor Input Power (W)')
axs[1, 2].plot(times, socs, 'tab:blue')
axs[1, 2].set_title('State of Charge (%)')
plt.tight_layout()
plt.show()
