import math
import matplotlib.pyplot as plt
import numpy as np
import time

def VDS(test):
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
    RHO = 1.225            # Air density (NASA) (kg/m^3)
    CSA = 0.466            # Cross-sectional area (m^2)
    C_D = 1.166            # Drag coefficient
    C_L = -0.2             # Lift coefficient
    C_S = 0.016            # Skin friction coefficient

    # Tyre parameters
    TYRE_PRESSURE_PSI = 50
    TYRE_PRESSURE = TYRE_PRESSURE_PSI / 14.504   # convert psi to bar
    TYRE_DIAMETER = 0.5    # m

    # Battery parameters
    V_MAX = 24             # Maximum voltage
    V_MIN = 18             # Minimum voltage
    H = 20                 # Rated discharge time in hours
    C_CAP = 36             # Battery capacity in Ah
    K_PEUKERT = 1.2        # Peukert's constant
    BATTERY_WH = 650       # Battery capacity in Wh

    # Motor parameters
    MOTOR_EFFICIENCY = 0.7
    MOTOR_INITIAL_RESISTANCE = 0.063  # Ohms 
    NO_LOAD_CURRENT = 0.5
    ORIGINAL_TORQUE_CONSTANT = 0.104
    BACK_EMF_CONSTANT = 1.15
    I_STALL_MAX = 130
    RPM_DESIRED = 1650     # Target motor rpm from motor graph

    # Initial gear ratio will be calculated later
    # But I could create an override for the future
    DIA_MOTOR_GEAR = 1
    DIA_AXLE_GEAR = 2.75
    # GEAR_RATIO = DIA_AXLE_GEAR / DIA_MOTOR_GEAR

    # Each entry is [lap_progress, angle (radians), direction factor]
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
        [250, math.pi/800, -1],
        [275, math.pi/750, -1]
    ]
    # Map which sums to 0 elevation change
    ELEVATION_MAP = [
        [10, 0, 1],
        [35, math.pi/700, 1],
        [45, math.pi/800, 1],
        [70, 0, 1],
        [100, 0.00028548081085095285, -1],  # Adjusted
        [110, 0.002441535705601295, -1],  # Adjusted
        [135, 0.002441535705601295, -1],  # Adjusted
        [160, 0, 1],
        [200, math.pi/1000, 1],
        [250, 0.002441535705601295, -1],  # Adjusted
        [275, 0.002441535705601295, -1]
    ]

    # --- Functions ---
    def aero_forces(velocity):
        """
        Calculate skin friction, drag, and lift.
        """
        dynamic_pressure = 0.5 * RHO * velocity**2
        skin_friction = C_S * dynamic_pressure * CSA # Eq. 21 in research section 3.2.3
        drag = C_D * dynamic_pressure * CSA # Eq. 22 in research section 3.2.3
        lift = C_L * dynamic_pressure * CSA # Eq. 23 in research section 3.2.3
        return skin_friction, drag, lift

    def rolling_resistance(lift, velocity):
        """
        Calculate rolling resistance.
        """
        # Eq. 28 in research section 3.2.4
        c_rr = 0.005 + 1 / TYRE_PRESSURE * (0.01 + 0.0095 * ((velocity * 3.6 / 100) ** 2))
        F_rr = c_rr * (TOTAL_WEIGHT * G - lift) # Eq. 26 in research section 3.2.4
        return F_rr

    def track_elevation(lap_progress):
        """
        Calculate the force induced by the track elevation.
        """
        pointer = 0
        while lap_progress > ELEVATION_MAP[pointer+1][0] and pointer+2 < len(ELEVATION_MAP):
            pointer += 1
        angle = ELEVATION_MAP[pointer][1]
        direction = ELEVATION_MAP[pointer][2]
        elevation_force = TOTAL_WEIGHT * G * math.sin(angle) * direction
        return elevation_force * test

    def motor(motor_torque, motor_rpm):
        """
        Calculate force at the wheels given a motor torque.
        """
        wheel_rpm = motor_rpm / GEAR_RATIO
        wheel_torque = GEAR_RATIO * motor_torque 
        motor_force = wheel_torque / (TYRE_DIAMETER/2) # Force = moment/distance
        max_motor_velocity = wheel_rpm * (math.pi * TYRE_DIAMETER) / 60 # Max velocity achievable by the car
        return wheel_rpm, wheel_torque, motor_force, max_motor_velocity

    def calc_motor_temp_changes(temperature):
        """
        Recalculate motor graph due to temperature changes.
        """
        voltage = 24 # Keep constant for graph
        # Eq 12 in research section 3.2.2
        motor_resistance = MOTOR_INITIAL_RESISTANCE * (1 + 0.004 * (temperature - 20))
        # Eq 13 in research section 3.2.2    
        I_Stall = voltage / motor_resistance 
        # Eq 14 in research section 3.2.2
        torque_constant = ORIGINAL_TORQUE_CONSTANT * (1 - 0.0012 * (temperature - 20))
        # Eq 15 in research section 3.2.2
        T_stall = I_Stall * torque_constant
        # Eq 16 in research section 3.2.2
        no_load = 9.5493 * (voltage - (NO_LOAD_CURRENT * motor_resistance)) / torque_constant
        # Eq 17 in research section 3.2.2
        omega = voltage / torque_constant
        P_max = 0.25 * omega * T_stall  # estimation of maximum power
        return P_max, no_load, T_stall, I_Stall

    def motor_graph_conversion(battery_power, P_max, no_load, T_stall, I_Stall):
        """
        Use the motor graph to get the values of key variables.
        """
        motor_power = battery_power * 0.7
        motor_torque = -1 * math.sqrt(-1 * ((motor_power / (P_max / (T_stall/2)**2)) - (T_stall/2)**2)) + T_stall/2

        current = motor_torque * I_Stall / T_stall + NO_LOAD_CURRENT
        motor_rpm = (-no_load / T_stall) * motor_torque + no_load
        return motor_torque, current, motor_rpm

    def battery_update(SoC, voltage, current, t, watthours):
        """
        Update battery SoC and voltage using a Peukert model and voltage discharge curve from the team's battery dyno.
        """
        # t_discharge = H * (C_CAP / (current * H)) ** K_PEUKERT # Peukert's Law Eq. 4 in research section 3.2.1 
        # SoC -= SoC * TIME_STEP / (t_discharge * 3600)
        
        SoC = (1 - (watthours / BATTERY_WH)) * 100
        
        # Voltage model
        if t < 3500:
            voltage = 18 + 6 * np.exp(-np.log(2) * (t / 3500) ** 2)
        else:
            A = -9.6235e-6 # For smooth transition
            B = -0.0011883 # For continuity
            voltage = 21 + B * (t - 3500) + A * (t - 3500) ** 2
        return current, SoC, voltage


    def temperature_and_cooling(temperature):
        """
        Model the changes in motor temperature.
        More research needs to be done into how the motor temp changes during the race.
        """
        # return temperature + test/3600
        return temperature

    def power_degradation(power, time):
        """
        Approximated graph for power degredation over time. Modelled as a straight line and then a quadratic.
        This can be refined in the future.
        """
        # Power model
        if time<3000:
            power= (BATTERY_WH/571.1) * (-1/15 *time +700)
        else:
            power = (BATTERY_WH/571.1) * (-0.000444 * (time - 3000) **2 -0.0667 * (time - 3000) + 500) 
            
        return power

    def update_velocity_distance(velocity, acceleration, distance, max_velocity, moving, power, watthours):
        """
        Update vehicle velocity and distance using SUVAT.
        """
        u = velocity
        new_velocity = u + acceleration * TIME_STEP # v = u + at
        new_velocity = min(new_velocity, max_velocity)
        if new_velocity <= 0:
            moving  = False  
            new_velocity = 0
        distance += TIME_STEP * (u + new_velocity) / 2  # s = (u + v)/2 * t
        watthours += power*TIME_STEP/3600
        return new_velocity, distance, moving, watthours

    def find_max_speed(max_power, velocity, lap_progress):
        """
        Iteratively determine the maximum achievable speed for the given available power.
        """
        while True:
            skin_friction, drag, lift = aero_forces(velocity)
            F_rr = rolling_resistance(lift, velocity)
            elevation_force = track_elevation(lap_progress)
            total_force = skin_friction + drag + F_rr + elevation_force
            resistive_power = total_force * velocity
            if resistive_power >= max_power:  
                break
            velocity += 0.01
        return skin_friction, drag, lift, F_rr, elevation_force, total_force, resistive_power, velocity   

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
    watthours = 0
    battery_power = BATTERY_WH  # initial battery power available

    # Lists for storing simulation results
    times = []
    velocities = []
    distances = []
    socs = []
    voltages = []
    currents = []
    powers = []
    watthours_log = []

    # ----- Find opitmum gear ratio -----

    # Find max spped for a given power. 
    #  450 taken from graph - find power and divide by efficiency to get battery power = 650
    skin_friction, drag, lift, F_rr, elevation_force, total_force, power_req, max_velocity = find_max_speed(450, velocity, lap_progress)
    wheel_rpm =  max_velocity * 60 / (math.pi * TYRE_DIAMETER)
    gear_ratio = RPM_DESIRED / wheel_rpm
    GEAR_RATIO = gear_ratio
    # print("Optimised gear ratio:", GEAR_RATIO)
    GEAR_RATIO = test

    # --- Main Simulation Loop ---
    for t in range(SESSION_LENGTH):
            # Update battery power degradation over time
        battery_power = power_degradation(battery_power, t)

        # Update motor temperature (a cooling algorithm can be added at a later stage)
        motor_temperature = temperature_and_cooling(motor_temperature)

        # Update motor characteristics graph based on motor temperature
        P_max, no_load, T_stall, I_Stall = calc_motor_temp_changes(motor_temperature)

        # Use new graph to get key variables
        motor_torque, current, motor_rpm = motor_graph_conversion(battery_power, P_max, no_load, T_stall, I_Stall)

        # Update battery SoC and voltage
        current, SoC, voltage = battery_update(SoC, voltage, current, t, watthours)
        
        # Calculate wheel parameters and forces
        wheel_rpm, wheel_torque, motor_force, max_motor_velocity = motor(motor_torque, motor_rpm)

        # Determine the maximum speed for available power  
        skin_friction, drag, lift, F_rr, elevation_force, totalForce, resistive_power, velocity = find_max_speed(battery_power*0.7,velocity, lap_progress)

        # Update lap progress - assume 5 min lap
        lap_progress = (lap_progress + 1) % 300

        # Calculate net force (motor force minus resistive forces) and acceleration
        net_force = motor_force - (skin_friction + drag + F_rr + elevation_force)
        acceleration = net_force / TOTAL_WEIGHT

        # Update vehicle velocity and traveled distance
        velocity, distance, moving, watthours = update_velocity_distance(velocity, acceleration, distance, max_motor_velocity, moving, battery_power, watthours)

        # Log simulation data
        times.append(t)
        velocities.append(velocity)
        distances.append(distance)
        socs.append(SoC)
        voltages.append(voltage)
        currents.append(current)
        powers.append(battery_power)
        watthours_log.append(watthours)

        # Break the loop if the car stops
        if not moving:
            break
    
    ### For use when testing individual runs or making changes ###

    # print("Final velocity:", velocity, "m/s")
    # print("Final SoC:", SoC)
    # print("Total distance traveled:", distance+10000, "m")
    # print("Final acceleration:", acceleration, "m/s^2")
    # print("Optimised gear ratio:", GEAR_RATIO)

    # # --- Plotting Results ---
    # fig, axs = plt.subplots(2, 3)
    # axs[0, 0].plot(times, currents)
    # axs[0, 0].set_title('Current (A)')
    # axs[0, 1].plot(times, velocities, 'tab:orange')
    # axs[0, 1].set_title('Velocity (m/s)')
    # axs[1, 0].plot(times, voltages, 'tab:green')
    # axs[1, 0].set_title('Voltage (V)')
    # axs[1, 1].plot(times, powers, 'tab:red')
    # axs[1, 1].set_title('Battery Power (W)')
    # axs[1, 2].plot(times, socs, 'tab:blue')
    # axs[1, 2].set_title('State of Charge (%)')
    # axs[0, 2].plot(times, watthours_log, 'tab:orange')
    # axs[0, 2].set_title('Watthours (Wh)')
    # plt.tight_layout()
    # plt.show()

    return distance

start = time.time()
gear_ratios = []
distances = []
for i in range(10, 1000, 10):
    dist = VDS(i/100)
    gear_ratios.append(i/100)
    distances.append(dist)

print(f"Time taken for 100 runs: {time.time() - start}")
plt.plot(gear_ratios, distances)
plt.title('Gear Ratios')
plt.show()