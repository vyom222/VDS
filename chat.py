import math
import matplotlib.pyplot as plt
import numpy as np

# --- Constants ---
G = 9.81  # Gravity [m/s^2]
SESSION_LENGTH = 3600  # seconds
TIME_STEP = 1

# Vehicle Parameters
CAR_WEIGHT = 65  # kg
DRIVER_WEIGHT = 65  # kg
TOTAL_WEIGHT = CAR_WEIGHT + DRIVER_WEIGHT

# Aerodynamic Parameters
RHO = 1.225      # Air density [kg/m^3]
CSA = 0.1535     # Cross-sectional area [m^2]
C_D = 1.07       # Drag coefficient
C_L = 0.89       # Lift coefficient
C_S = 0.011      # Skin friction coefficient

# Tyre Parameters
TYRE_PRESSURE_PSI = 40
TYRE_PRESSURE = TYRE_PRESSURE_PSI / 14.504  # convert psi to bar
TYRE_DIAMETER = 0.5  # metres

# Battery Parameters
V_MAX = 24  # Maximum voltage
V_MIN = 18  # Minimum voltage
H = 20      # Battery rated discharge time in Hours
C = 36      # Battery capacity [Ah]
K = 1.2     # Peukert's constant
BATTERY_WH = 1000  # Watt-hours

# Motor Parameters
MOTOR_EFFICIENCY = 0.9
MOTOR_INITIAL_RESISTANCE = 0.063  # Ohm
GEAR_RATIO = 2.75  # Initial gear ratio guess
NO_LOAD_CURRENT = 0.5

# Elevation Map: [lap_progress, angle (radians), direction factor]
ELEVATION_MAP = [
    [10, 0, 1], [35, math.pi/700, 1], [45, math.pi/800, 1],
    [70, 0, 1], [100, math.pi/600, -1], [110, math.pi/500, -1],
    [135, math.pi/550, -1], [160, 0, 1], [200, math.pi/1000, 1],
    [220, math.pi/1000, -1], [250, math.pi/800, -1], [275, math.pi/750, -1]
]

# --- Classes ---
class Battery:
    """Simulates the battery behavior using a simplified voltage and SoC model."""
    def __init__(self, wh, V_max, V_min, H, C, K):
        self.wh = wh
        self.V_max = V_max
        self.V_min = V_min
        self.H = H
        self.C = C
        self.K = K
        self.SoC = 100  # Start with 100% State of Charge
        self.voltage = V_max

    def update(self, current, time, time_step):
        # Use Peukert's Law to update SoC
        t_discharge = self.H * (self.C / (current * self.H)) ** self.K
        self.SoC -= self.SoC * time_step / (t_discharge * 3600)

        # Voltage curve model
        if time < 3500:
            self.voltage = 18 + 6 * np.exp(-np.log(2) * (time / 3500) ** 2)
        else:
            A_fixed = -3.84e-6
            B_fixed = -0.00257
            self.voltage = 21 + B_fixed * (time - 3500) + A_fixed * (time - 3500) ** 2
        return self.voltage, self.SoC

class Motor:
    """Simulates motor behavior and calculates forces."""
    def __init__(self, initial_resistance, gear_ratio, tyre_diameter):
        self.initial_resistance = initial_resistance
        self.gear_ratio = gear_ratio
        self.tyre_diameter = tyre_diameter

    def calculate_wheel_force(self, motor_torque):
        """Calculate the force at the wheel given the motor torque."""
        wheel_torque = self.gear_ratio * motor_torque 
        motor_force = wheel_torque / (self.tyre_diameter / 2)
        return motor_force

    def motor_temperature_effects(self, temperature):
        """Adjust motor parameters based on temperature."""
        Motor_Resistance = self.initial_resistance * (1 + 0.004 * (temperature - 20))
        I_stall = self.voltage / Motor_Resistance  # Simplified estimate
        # Further calculations can be added here
        return Motor_Resistance, I_stall

class Track:
    """Provides track elevation data and calculates the downhill force."""
    def __init__(self, elevation_map):
        self.elevation_map = elevation_map

    def get_downhill_force(self, lap_progress, total_weight):
        pointer = 0
        while pointer + 1 < len(self.elevation_map) and lap_progress > self.elevation_map[pointer+1][0]:
            pointer += 1
        angle = self.elevation_map[pointer][1]
        direction = self.elevation_map[pointer+1][2] if pointer + 1 < len(self.elevation_map) else 1
        downhill_force = total_weight * G * math.sin(angle) * direction
        return downhill_force

# --- Helper Functions ---
def aero_forces(velocity):
    """Calculate aerodynamic forces: skin friction, drag, and lift."""
    dynamic_pressure = 0.5 * RHO * velocity**2
    skin_friction = C_S * dynamic_pressure * CSA
    drag = C_D * dynamic_pressure * CSA
    lift = C_L * dynamic_pressure * CSA
    return skin_friction, drag, lift

def rolling_resistance(lift, velocity):
    """Calculate the rolling resistance force."""
    c_rr = 0.005 + 1 / TYRE_PRESSURE * (0.01 + 0.0095 * ((velocity * 3.6 / 100) ** 2))
    F_rr = c_rr * (TOTAL_WEIGHT * G - lift)
    return F_rr

def update_velocity_distance(velocity, acceleration, time_step, distance, max_velocity):
    """Update the vehicle velocity and distance using basic kinematics."""
    new_velocity = velocity + acceleration * time_step
    new_velocity = min(new_velocity, max_velocity)
    if new_velocity < 0:
        new_velocity = 0
    distance += time_step * (velocity + new_velocity) / 2  # s = (u+v)/2 * t
    return new_velocity, distance

def find_max_speed(max_power, init_velocity, lap_progress, track):
    """
    Iteratively find the max speed where the required power equals the available max power.
    Returns the aerodynamic forces, forces on the vehicle, and the speed.
    """
    velocity = init_velocity
    while True:
        skin_friction, drag, lift = aero_forces(velocity)
        F_rr = rolling_resistance(lift, velocity)
        downhill_force = track.get_downhill_force(lap_progress, TOTAL_WEIGHT)
        total_force = skin_friction + drag + F_rr + downhill_force
        power_required = total_force * velocity
        if power_required >= max_power:
            break
        velocity += 0.01
    return skin_friction, drag, lift, F_rr, downhill_force, total_force, power_required, velocity

# --- Main Simulation Function ---
def run_simulation():
    # Initialize state variables
    time = 0
    distance = 0
    lap_progress = 0
    velocity = 0
    motor_temperature = 20

    # Initialize simulation components
    battery = Battery(BATTERY_WH, V_MAX, V_MIN, H, C, K)
    motor = Motor(MOTOR_INITIAL_RESISTANCE, GEAR_RATIO, TYRE_DIAMETER)
    track = Track(ELEVATION_MAP)

    # Lists to store simulation results
    times, distances, velocities, socs, voltages = [], [], [], [], []

    # Main simulation loop (1 second time-step)
    for t in range(SESSION_LENGTH):
        # Update battery state (using an assumed current; adjust as needed)
        current_assumed = 20  # Placeholder value; in a real model, calculate from demand
        voltage, soc = battery.update(current_assumed, t, TIME_STEP)

        # Example motor simulation (you could add more detailed calculations here)
        # For now, assume a constant motor torque based on available power:
        available_power = voltage * current_assumed
        motor_torque = 2.9  # Placeholder value
        motor_force = motor.calculate_wheel_force(motor_torque)

        # Calculate aerodynamic and resistive forces
        skin_friction, drag, lift = aero_forces(velocity)
        F_rr = rolling_resistance(lift, velocity)
        downhill_force = track.get_downhill_force(lap_progress, TOTAL_WEIGHT)

        # Total resistance force acting on the vehicle
        total_resistance = skin_friction + drag + F_rr + downhill_force

        # Determine acceleration (simplified net force / mass)
        net_force = motor_force - total_resistance
        acceleration = net_force / TOTAL_WEIGHT

        # Update velocity and distance
        # Use a max speed determined by available power (simplified here)
        _, _, _, _, _, _, _, max_velocity = find_max_speed(available_power * 0.7, velocity, lap_progress, track)
        velocity, distance = update_velocity_distance(velocity, acceleration, TIME_STEP, distance, max_velocity)

        # Update lap progress (simulate track looping)
        lap_progress = (lap_progress + 1) % 300

        # Save results
        times.append(t)
        distances.append(distance)
        velocities.append(velocity)
        socs.append(soc)
        voltages.append(voltage)

    return times, distances, velocities, socs, voltages

# --- Run Simulation and Plot ---
if __name__ == '__main__':
    times, distances, velocities, socs, voltages = run_simulation()

    # Plotting results
    fig, axs = plt.subplots(2, 3, figsize=(12, 8))
    axs[0, 0].plot(times, velocities)
    axs[0, 0].set_title('Velocity (m/s)')
    axs[0, 1].plot(times, voltages, 'tab:green')
    axs[0, 1].set_title('Voltage (V)')
    axs[0, 2].plot(times, socs, 'tab:blue')
    axs[0, 2].set_title('State of Charge (%)')
    axs[1, 0].plot(times, distances, 'tab:red')
    axs[1, 0].set_title('Distance (m)')
    # Additional plots can be added as needed
    fig.tight_layout()
    plt.show()
print(distances[-1])