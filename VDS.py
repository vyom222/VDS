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
velocity = 0.01 # m/s

def aero(velocity):
    skin_friction = C_S * 0.5 * RHO * velocity ** 2 * CSA # Eq. 21 in research section 3.2.3
    drag = C_D * 0.5 * RHO * velocity ** 2 * CSA # Eq. 22 in research section 3.2.3
    lift = C_L * 0.5 * RHO * velocity ** 2 * CSA # Eq. 23 in research section 3.2.3

    # wind speed not added yet
    return skin_friction,drag,lift

def rolling_resistance(lift):
    # Eq. 28 in research section 3.2.4 and conversion to km/h from m/s
    c_rr = 0.005 + 1 / TYRE_PRESSURE * (0.01 + 0.0095 * (velocity*3.6 /100)**2) 

    F_rr = c_rr * (TOTAL_WEIGHT * G - lift) # Eq. 26 in research sectino 3.2.4
    return F_rr

def Battery(SoC, voltage):
    current = voltage / ( mot_r + 2 * BAT_R) # Ohm's Law in research section 3.2.2
    t = H * (C/ current*H) ** K # Peukert's Law Eq. 4 in research section 3.2.1 
    SoC = SoC - SoC * time_step/t
    voltage = V_MIN + (V_MAX - V_MIN) * SoC /100 # Eq. 3 in research section 3.2.1
    return current, SoC, voltage 

def Motor(current, voltage):
    rpm = velocity * 60 / (math.pi * TYRE_DIAMETER) # Eq. 10 in research section 3.2.2
    torque_motor =  (MOTOR_EFFICIENCY * current * voltage * 60) / (rpm * 2*math.pi) # Eq. 9 in research section 3.2.2
    if torque_motor > 12:
        torque_motor = 12 # limit to torque according to datasheet
    motor_force = (GEAR_RATIO * TRANSMISSION_EFFICIENCY * torque_motor) / (TYRE_DIAMETER/2) # Force = moment/ distance
    return rpm, motor_force, torque_motor

def Acceleration(motor_force, F_rr, skin_friction, drag):
    resultant_force = motor_force - F_rr - skin_friction - drag
    acceleration = resultant_force/TOTAL_WEIGHT # F = ma, m per s^2
    return resultant_force, acceleration

def Velocity_Distance(velocity, acceleration, time_step, distance):
    u = velocity 
    velocity = velocity + acceleration * time_step # v = u + at
    distance += time_step * (u + velocity)/2 # s = (u+v)/2 * t
    return velocity, distance


for i in range(3600):
    skin_friction,drag,lift = aero(velocity)
    print('skin, drag, lift:', skin_friction,drag,lift)
    F_rr = rolling_resistance(lift)
    print('F_rr:', F_rr)
    current, SoC, voltage = Battery(SoC, voltage)
    print('current, SoC, voltage:', current, SoC, voltage)
    rpm, motor_force, torque_motor = Motor(current, voltage)
    print('rpm, motor_force, torque: ', rpm, motor_force, torque_motor)
    resultant_force, acceleration = Acceleration(motor_force, F_rr, skin_friction, drag)
    print('Resultant, Acceleration:',resultant_force, acceleration)
    velocity,distance = Velocity_Distance(velocity, acceleration, time_step, distance)
    print('Velocity, distance:', velocity,distance)
