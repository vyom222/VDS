import numpy as np
import matplotlib.pyplot as plt

def MotorTemperature(temperature):
    Motor_Resistance = MOTOR_INITIAL_RESISTANCE * (1 + 0.004 * (temperature - 20))  # Eq.12
    I_Stall = voltage / Motor_Resistance
    Torque_constant = ORIGINAL_TORQUE_CONSTANT * (1 - 0.0012 * (temperature - 20))
    T_stall = I_Stall * Torque_constant
    no_load = 9.5493 * (voltage - (NO_LOAD_CURRENT * Motor_Resistance)) / Torque_constant
    omega = voltage / Torque_constant
    P_max = 0.25 * omega * T_stall  # Maximum output power

    # Efficiency estimation at max power
    Input_Power = voltage * I_Stall
    Efficiency = (P_max / Input_Power) * 100  # Convert to percentage

    return P_max, no_load, T_stall, I_Stall, Efficiency

# Constants
ORIGINAL_TORQUE_CONSTANT = 0.104
MOTOR_INITIAL_RESISTANCE = 0.184
NO_LOAD_CURRENT = 0.5
voltage = 24

# Test different temperatures
temperatures = np.linspace(20, 100, 10)  # Test from 20°C to 100°C
efficiencies = []

for temp in temperatures:
    P_max, no_load, T_stall, I_Stall, Efficiency = MotorTemperature(temp)
    efficiencies.append(Efficiency)
    print(f"Temp: {temp:.1f}°C | P_max: {P_max:.2f} W | Efficiency: {Efficiency:.2f}%")

# Plot the efficiency vs temperature
plt.figure(figsize=(8,5))
plt.plot(temperatures, efficiencies, marker='o', linestyle='-', color='r', label="Efficiency (%)")
plt.xlabel("Temperature (°C)")
plt.ylabel("Efficiency (%)")
plt.title("Motor Efficiency vs Temperature")
plt.legend()
plt.grid(True)
plt.show()
