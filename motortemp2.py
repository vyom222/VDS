import numpy as np
import matplotlib.pyplot as plt

# Motor constants
ORIGINAL_TORQUE_CONSTANT = 0.104
NO_LOAD_CURRENT = 0.5
voltage = 24
MOTOR_INITIAL_RESISTANCE = 0.184

# Temperatures to sweep (in °C)
temperatures = [20, 40, 60, 80, 100]
colors = ['b', 'r', 'g', 'm', 'c']

def MotorTemperature(temperature):
    """
    Returns key motor parameters for a given temperature:
      - P_max: theoretical max mechanical power (unused in this sweep)
      - no_load: no-load speed in RPM
      - T_stall: stall torque in Nm
      - I_stall: stall current in A
    """
    Motor_Resistance = MOTOR_INITIAL_RESISTANCE * (1 + 0.004 * (temperature - 20))
    I_stall = voltage / Motor_Resistance
    Torque_constant = ORIGINAL_TORQUE_CONSTANT * (1 - 0.0012 * (temperature - 20))
    T_stall = I_stall * Torque_constant
    no_load = 9.5493 * (voltage - (NO_LOAD_CURRENT * Motor_Resistance)) / Torque_constant
    omega = voltage / Torque_constant
    P_max = 0.25 * omega * T_stall
    return P_max, no_load, T_stall, I_stall

def MotorPerformanceArrays(no_load, T_stall, I_stall, num_points=50):
    """
    Sweeps torque from 0 up to T_stall.
    Returns arrays for:
      - torque (Nm)
      - speed (RPM)
      - current (A)
      - power (W)
      - efficiency (%)
    """
    torque_array = np.linspace(0, T_stall, num_points)
    speed_array = np.zeros(num_points)
    current_array = np.zeros(num_points)
    power_array = np.zeros(num_points)
    efficiency_array = np.zeros(num_points)
    
    for i, tau in enumerate(torque_array):
        # Linear approximation: speed decreases linearly from no_load to 0 at stall torque.
        rpm = (-no_load / T_stall) * tau + no_load
        
        # Current increases linearly from NO_LOAD_CURRENT to I_stall.
        i_motor = tau * (I_stall / T_stall) + NO_LOAD_CURRENT
        
        # Mechanical power (W) = torque (Nm) * angular speed (rad/s)
        # Convert rpm to rad/s: (rpm * 2π) / 60
        mech_power = tau * (rpm * 2.0 * np.pi / 60.0)
        
        # Electrical input power (W) = current * voltage
        input_power = i_motor * voltage
        
        # Efficiency (%)
        eff = 100.0 * (mech_power / input_power) if input_power > 0 else 0
        
        speed_array[i] = rpm
        current_array[i] = i_motor
        power_array[i] = mech_power
        efficiency_array[i] = eff
    
    return torque_array, speed_array, current_array, power_array, efficiency_array

# Create the plot with two y-axes.
fig, ax_left = plt.subplots(figsize=(10, 6))
ax_right = ax_left.twinx()

for i, temp in enumerate(temperatures):
    color = colors[i]
    
    # Get motor parameters for this temperature
    P_max, no_load, T_stall, I_stall = MotorTemperature(temp)
    
    # Get performance arrays over a torque sweep from 0 to T_stall.
    torque, speed, current, power, eff = MotorPerformanceArrays(no_load, T_stall, I_stall, num_points=50)
    
    # Plot on left y-axis: speed (solid) and power (dashed)
    ax_left.plot(torque, speed, linestyle='-', color=color, label=f"Speed @ {temp}°C")
    ax_left.plot(torque, power, linestyle='--', color=color, label=f"Power @ {temp}°C")
    
    # Plot on right y-axis: current (dash-dot) and efficiency (dotted)
    ax_right.plot(torque, current, linestyle='-.', color=color, label=f"Current @ {temp}°C")
    ax_right.plot(torque, eff, linestyle=':', color=color, label=f"Efficiency @ {temp}°C")

# Set labels and title
ax_left.set_xlabel("Torque [Nm]")
ax_left.set_ylabel("Speed [RPM] / Power [W]", color='b')
ax_right.set_ylabel("Current [A] / Efficiency [%]", color='g')
ax_left.set_title("Motor Performance vs. Torque at Different Temperatures")
ax_left.grid(True)

# Combine legends from both axes
lines_left, labels_left = ax_left.get_legend_handles_labels()
lines_right, labels_right = ax_right.get_legend_handles_labels()
ax_left.legend(lines_left + lines_right, labels_left + labels_right, loc="upper right", fontsize='small')

plt.show()
