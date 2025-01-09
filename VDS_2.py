### VERSION TO USE POWER AS THE BASE COMPARED TO CURRENT
import math

# Initialise variables
time = 0
time_step = 1
distance = 0

CAR_WEIGHT = 40
DRIVER_WEIGHT = 65
TOTAL_WEIGHT = CAR_WEIGHT + DRIVER_WEIGHT
G = 9.81

c_rr = 0 # coeff of rolling resistance
TYRE_PRESSURE_PSI = 40 # psi
TYRE_PRESSURE = TYRE_PRESSURE_PSI / 14.504 # convert to bar

C_D = 0.49 # coeff of drag
C_L = -0.58 # coeff of lift
C_S = 0.057 # coeff of skin friction
RHO = 1.225 # fluid density
CSA = 0.1535 # cross-sectional area
WIND_SPEED = 0
wind_direction = 0 # angle in degrees

V_MAX = 26 # 100% SoC
V_MIN = 18 # 0% SoC
SoC = 100 
voltage = V_MAX 
current = 0 
BAT_R = 0.12 # Battery internal resistance
H = 20 # Battery Rated Dishcharge time in Hours
C = 36 # #Battery Rated Capacity at discharge rate in Ah
K = 1.2 # Estimation for Peukert's constant
t = 0 # discharge time

rpm = 0
mot_r = 1 # motor internal resistnace
TYRE_DIAMETER = 0.5 # metres
MOTOR_EFFICIENCY = 0.9
DIA_MOTOR_GEAR = 1
DIA_AXLE_GEAR = 1.5
GEAR_RATIO = DIA_AXLE_GEAR/ DIA_MOTOR_GEAR
TRANSMISSION_EFFICIENCY = 0.9
DISTANCE_FROM_MOTOR = 0.5

resultant_force = 0
acceleration = 0
u = 0 # inital velocity
velocity = 10 # m/s

def aero(velocity):
    skin_friction = C_S * 0.5 * RHO * velocity ** 2 * CSA # Eq. 21 in research section 3.2.3
    drag = C_D * 0.5 * RHO * velocity ** 2 * CSA # Eq. 22 in research section 3.2.3
    lift = C_L * 0.5 * RHO * velocity ** 2 * CSA # Eq. 23 in research section 3.2.3

    P_SkinF = skin_friction * velocity
    P_Drag = drag * velocity
    P_Lift = lift * velocity

    # wind speed not added yet
    return skin_friction,drag,lift, P_SkinF, P_Drag, P_Lift

def rolling_resistance(lift, velocity):
    # Eq. 28 in research section 3.2.4 and conversion to km/h from m/s
    c_rr = 0.005 + 1 / TYRE_PRESSURE * (0.01 + 0.0095 * (velocity*3.6 /100)**2) 

    F_rr = c_rr * (TOTAL_WEIGHT * G - lift) # Eq. 26 in research sectino 3.2.4
    P_rr = F_rr * velocity # Power loss from rr
    return F_rr, P_rr



skin_friction,drag,lift, P_SkinF, P_Drag, P_Lift = aero(velocity)
F_rr, P_rr = rolling_resistance(lift, velocity)

# print(skin_friction,drag,lift,F_rr)
# print(P_SkinF, P_Lift, P_Drag, P_rr)

