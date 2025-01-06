import math

# Initialise variables
time = 0
time_step = 1
distance = 0

CAR_WEIGHT = 40
DRIVER_WEIGHT = 60
TOTAL_WEIGHT = CAR_WEIGHT + DRIVER_WEIGHT
G = 9.81

c_rr = 0 # coeff of rolling resistance
TYRE_PRESSURE_PSI = 30 # psi
TYRE_PRESSURE = TYRE_PRESSURE_PSI / 14.504

C_D = 0.55 # coeff of drag
C_L = -0.3 # coeff of lift
C_F = 0.12 # coeff of skin friction
RHO = 1.125 # fluid density
CSA = 0.32 # cross-sectional area
WIND_SPEED = 0
wind_direction = 0 # angle in degrees

V_MAX = 24 # 100% SoC
V_MIN = 18 # 0% SoC
SoC = 100 
Voltage = V_MAX 
current = 0 
BAT_R = 0.12 # Battery internal resistance

rpm = 0
mot_r = 0 # motor internal resistnace

resultant_force = 0
acceleration = 0
velocity = 0 # m/s

def aero():
    skin_friction = C_F * 0.5 * RHO * velocity ** 2 * CSA # Eq. 21 in research section 3.2.3
    drag = C_D * 0.5 * RHO * velocity ** 2 * CSA # Eq. 22 in research section 3.2.3
    lift = C_L * 0.5 * RHO * velocity ** 2 * CSA # Eq. 23 in research section 3.2.3

    # wind speed not added yet
    return skin_friction,drag,lift

def rolling_resistance(lift):
    c_rr = 0.005 + 1 / TYRE_PRESSURE * (0.01 + 0.0095 * (velocity /100)**2) # Eq. 28 in research section 3.2.4
    F_rr = c_rr * (TOTAL_WEIGHT * G - lift)
    return F_rr
