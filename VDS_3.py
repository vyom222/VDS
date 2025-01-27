### VERSION TO USE POWER AS THE BASE COMPARED TO CURRENT
# I_min = 19
# linear power down and then just map that to get current

import math

# Initialise variables
time = 0
time_step = 1
distance = 0
session_length = 3600 

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
MIN_CURRENT = 19.5 # From telemetry data Goodwood finals 2024
MIN_POWER = MIN_CURRENT * V_MIN  # This is equivalent zero power left
BAT_R = 0.12 # Battery internal resistance
H = 20 # Battery Rated Dishcharge time in Hours
C = 36 # #Battery Rated Capacity at discharge rate in Ah
K = 1.2 # Estimation for Peukert's constant
t = 0 # discharge time
BATTERY_WH = 650 # Watthours of battery as calc by battery dyno
P_Battery = 650/(session_length/3600) # number of watts available
POWER_STEP = (BATTERY_WH - MIN_POWER)/3600 # power per second

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


def Battery(SoC, voltage, P_Battery):
    P_Battery = P_Battery - MIN_POWER/3600
    current = P_Battery/voltage
    t = H * (C/ current*H) ** K # Peukert's Law Eq. 4 in research section 3.2.1 
    SoC = SoC - SoC * time_step/t
    voltage = V_MIN + (V_MAX - V_MIN) * SoC /100 # Eq. 3 in research section 3.2.1
    return current, SoC, voltage, P_Battery


data = dict()
for i in range(3600):
    current, SoC, voltage, P_Battery = Battery(SoC, voltage, P_Battery)
    data[voltage] = P_Battery
    # print(current, SoC, voltage, P_Battery)

# 16.216551016114057 5.4715212163591165 18.437721697308728 298.9999999999904
for i in data:
    print (i, data[i])