### VERSION TO USE POWER AS THE BASE COMPARED TO CURRENT
# I_min = 19
# linear power down and then just map that to get current

import math
import matplotlib.pyplot as plt
import numpy as np

# Initialise variables
time = 0
time_step = 1
distance = 0
session_length = 3600 
moving = True

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
DIA_AXLE_GEAR = 2.55
GEAR_RATIO = DIA_AXLE_GEAR/ DIA_MOTOR_GEAR
TRANSMISSION_EFFICIENCY = 0.9
DISTANCE_FROM_MOTOR = 0.5

resultant_force = 0
acceleration = 0
u = 0 # inital velocity
velocity = 0.1 # m/s


def aero(velocity):
    skin_friction = C_S * 0.5 * RHO * (velocity ** 2) * CSA # Eq. 21 in research section 3.2.3
    drag = C_D * 0.5 * RHO * (velocity ** 2) * CSA # Eq. 22 in research section 3.2.3
    lift = C_L * 0.5 * RHO * (velocity ** 2) * CSA # Eq. 23 in research section 3.2.3

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

def Battery(SoC, voltage, P_Battery):
    P_Battery = P_Battery - MIN_POWER/3600
    current = P_Battery/voltage
    t = H * (C/ current*H) ** K # Peukert's Law Eq. 4 in research section 3.2.1 
    SoC = SoC - SoC * time_step/t
    voltage = V_MIN + (V_MAX - V_MIN) * SoC /100 # Eq. 3 in research section 3.2.1
    return current, SoC, voltage, P_Battery 

def Motor(current):
    '''
    current = 
    resultant speed
    average
    max spee
    '''
    motor_torque = current/10
    motor_rpm = (-2000/13.5) * motor_torque + 2000
    wheel_rpm = motor_rpm / GEAR_RATIO
    wheel_torque = GEAR_RATIO * motor_torque 
    motor_force = wheel_torque / (TYRE_DIAMETER/2)
    max_motor_velocity = wheel_rpm * (math.pi * TYRE_DIAMETER) / 60
    motor_acceleration = motor_force/ TOTAL_WEIGHT

    return motor_torque, motor_rpm, motor_force, wheel_torque, wheel_rpm, max_motor_velocity, motor_acceleration

def Power(P_motor, P_Battery, P_SkinF, P_Drag, P_rr):
    P_resultant = P_Battery - (P_motor + P_Drag + P_SkinF + P_rr)
    return P_resultant

def Acceleration(F_rr, skin_friction, drag, motor_acceleration):
    negative_force = - F_rr - skin_friction - drag
    negative_acceleration = negative_force/TOTAL_WEIGHT # F = ma, m per s^2
    acceleration = motor_acceleration + negative_acceleration
    return negative_force, negative_acceleration, acceleration

def Velocity_Distance(velocity, acceleration, time_step, distance, max_motor_velocity, moving):
    u = velocity 
    velocity = u + acceleration*time_step # v = u + at
    if velocity > max_motor_velocity:
        velocity = max_motor_velocity
    if velocity<0:
        moving = False
    distance += time_step * (u + velocity)/2 # s = (u+v)/2 * t
    return velocity, distance, moving

times = []
voltages = []
motor_rpms = []
motor_torques = []
velocities = []
currents = []
for i in range(3600):

    skin_friction,drag,lift, P_SkinF, P_Drag, P_Lift = aero(velocity)
    F_rr, P_rr = rolling_resistance(lift, velocity)
    current, SoC, voltage, P_Battery = Battery(SoC, voltage, P_Battery)
    motor_torque, motor_rpm, motor_force, wheel_torque, wheel_rpm, max_motor_velocity, motor_acceleration = Motor(current)
    # P_resultant = Power(P_motor, P_Battery, P_SkinF, P_Drag, P_rr)
    negative_force, negative_acceleration, acceleration = Acceleration(F_rr, skin_friction, drag, motor_acceleration)
    velocity, distance, moving = Velocity_Distance(velocity, acceleration, time_step, distance, max_motor_velocity, moving)
    
    time+=time_step

    # print(skin_friction,drag,lift,F_rr)
    # print(P_SkinF, P_Lift, P_Drag, P_rr)
    print(f"time: {time}")
    print(f"Current: {current}, SoC: {SoC}, Voltage: {voltage}, P_Battery: {P_Battery}")
    print(f"Motor torque: {motor_torque}, Motor rpm: {motor_rpm}, Motor force: {motor_force}, Max_velocity: {max_motor_velocity}")
    print(f"Wheel torque: {wheel_torque}, Wheel rpm: {wheel_rpm}")
    # print(P_resultant)
    print(f"Negative force: {negative_force}, Negative acceleration: {negative_acceleration}, Acceleration: {acceleration}")
    print(f"Velocity: {velocity}, Distance: {distance}")

    times.append(time)
    voltages.append(voltage)
    motor_rpms.append(motor_rpm)
    motor_torques.append(motor_torque)
    velocities.append(velocity)
    currents.append(current)
    if not moving:
        break
# plt.plot(times,currents)
# plt.show()
# plt.plot(times,velocities)
# plt.show()
# plt.plot(times,voltages)
# plt.show()
# plt.plot(times,motor_rpms)
# plt.show()
# plt.plot(times,motor_torques)
# plt.show()

