import numpy as np
import matplotlib.pyplot as plt

# Motor constants
ORIGINAL_TORQUE_CONSTANT = 0.104
NO_LOAD_CURRENT = 0.5
voltage = 24
MOTOR_INITIAL_RESISTANCE = 0.063

# Temperatures to sweep (in °C)
temperatures = [20, 40, 60, 80, 100]
colours = ['b', 'r', 'g', 'm', 'c']

def calc_motor_temp_changes(temperature):
    """
    Recalculate motor graph due to temperature changes.
    Same subprogram in the VDS
    """
    voltage = 24 # Keep constant for graph
    # Eq 12 in research section 3.2.2
    motor_resistance = MOTOR_INITIAL_RESISTANCE * (1 + 0.004 * (temperature - 20))
    # Eq 13 in research section 3.2.2    
    I_Stall = voltage / motor_resistance 
    I_Stall = 150
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

def MotorPerformanceArrays(no_load, T_stall, I_stall, num_points=50):
    """
    Sweeps torque from 0 up to T_stall.
    """
    torque_array = np.linspace(0, T_stall, num_points)
    speed_array = np.zeros(num_points)
    current_array = np.zeros(num_points)
    power_array = np.zeros(num_points)
    efficiency_array = np.zeros(num_points)
    
    for i, torque in enumerate(torque_array):

        motor_rpm = (-no_load / T_stall) * torque + no_load
        current = torque * ((I_stall - NO_LOAD_CURRENT )/ T_stall) + NO_LOAD_CURRENT

        # Convert rpm to rad/s: (rpm * 2π) / 60
        mech_power = torque * (motor_rpm * 2.0 * np.pi / 60.0)
        
        input_power = current * voltage
        if input_power > 0:
            eff = 100.0 * (mech_power / input_power)
        else:
            eff = 0

        speed_array[i] = motor_rpm
        current_array[i] = current
        power_array[i] = mech_power
        efficiency_array[i] = eff
    
    return torque_array, speed_array, current_array, power_array, efficiency_array

# Create the plot with two y-axes.
fig, ax_left = plt.subplots(figsize=(10, 6))
ax_right = ax_left.twinx()

for i, temp in enumerate(temperatures):
    colour = colours[i]
    
    # Get motor parameters for this temperature
    P_max, no_load, T_stall, I_stall = calc_motor_temp_changes(temp)
    
    # Get arrays over a torque sweep from 0 to T_stall.
    torque, speed, current, power, eff = MotorPerformanceArrays(no_load, T_stall, I_stall, num_points=50)
    
    # Plot on left y-axis: speed and power
    ax_left.plot(torque, speed, linestyle='-', color=colour, label=f"Speed @ {temp}°C")
    ax_left.plot(torque, power, linestyle='--', color=colour, label=f"Power @ {temp}°C")
    
    # Plot on right y-axis: current and efficiency
    ax_right.plot(torque, current, linestyle='-.', color=colour, label=f"Current @ {temp}°C")
    ax_right.plot(torque, eff, linestyle=':', color=colour, label=f"Efficiency @ {temp}°C")

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
