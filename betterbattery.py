import matplotlib as plt
import numpy as np

# Battery Parameters
C_rated = 36  # Rated capacity in Ah
V_ocv_full = 12.8  # Open Circuit Voltage when fully charged
V_ocv_empty = 9  # Open Circuit Voltage when empty
I_discharge = 20  # Constant discharge current in A
Peukert_exponent = 1.2  # Peukert's exponent (typical for lead-acid batteries)
R_internal = 0.02  # Internal resistance in ohms
R_polarization = 0.05  # Polarization resistance in ohms
C_polarization = 500  # Polarization capacitance in Farads

# Peukert's Law - Adjusted Capacity
I_ref = C_rated / 20  # Reference current (20-hour rate)
C_effective = C_rated * (I_ref / I_discharge) ** (Peukert_exponent - 1)

# Time Calculation
T_total = C_effective / I_discharge  # Total discharge time in hours
T_steps = np.linspace(0, T_total, 1000)  # Time steps in hours

# State of Charge (SoC) over time
SoC = 1 - (I_discharge * T_steps / C_effective)
SoC[SoC < 0] = 0  # Ensure SoC never goes negative

# Nonlinear Open Circuit Voltage Model to simulate voltage cliff
V_ocv = V_ocv_empty + (V_ocv_full - V_ocv_empty) * (1 - np.exp(-5 * SoC))

# Internal Resistance Voltage Drop
V_ir = I_discharge * R_internal

# Polarization Voltage Drop (RC Model Simulation)
V_polarization = (I_discharge * R_polarization) * (1 - np.exp(-T_steps / (R_polarization * C_polarization)))

# Total Terminal Voltage
V_t = V_ocv - V_ir - V_polarization

# Plot Results
plt.figure(figsize=(8, 5))
plt.plot(T_steps, V_t, label='Battery Voltage (V)', color='b')
plt.axhline(y=V_ocv_empty, color='r', linestyle='--', label='Cutoff Voltage')
plt.xlabel('Time (hours)')
plt.ylabel('Voltage (V)')
plt.title("Lead-Acid Battery Discharge Curve with Peukert's Effect and Voltage Cliff")
plt.legend()
plt.grid()
plt.show()
