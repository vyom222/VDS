# add random subprograms for future proffing, motor temp, mgsin theta
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
BATTERY_WH = 1000 # Watthours of battery as calc by battery dyno
P_Battery = BATTERY_WH/(session_length/3600) # number of watts available
POWER_STEP = (BATTERY_WH - MIN_POWER)/3600 # power per second

rpm = 0
mot_r = 1 # motor internal resistnace
TYRE_DIAMETER = 0.5 # metres
MOTOR_EFFICIENCY = 0.9
DIA_MOTOR_GEAR = 1
DIA_AXLE_GEAR = 2.75
GEAR_RATIO = DIA_AXLE_GEAR/ DIA_MOTOR_GEAR
TRANSMISSION_EFFICIENCY = 0.9
DISTANCE_FROM_MOTOR = 0.5

resultant_force = 0
acceleration = 0
u = 0 # inital velocity
velocity = 0 # m/s

velocity_max = 0
power = 0

def aero_force(velocity):
    skin_friction = C_S * 0.5 * RHO * (velocity ** 2) * CSA # Eq. 21 in research section 3.2.3
    drag = C_D * 0.5 * RHO * (velocity ** 2) * CSA # Eq. 22 in research section 3.2.3
    lift = C_L * 0.5 * RHO * (velocity ** 2) * CSA # Eq. 23 in research section 3.2.3

    return skin_friction,drag,lift

def rolling_resistance_force(lift, velocity):
    # Eq. 28 in research section 3.2.4 and conversion to km/h from m/s
    c_rr = 0.005 + 1 / TYRE_PRESSURE * (0.01 + 0.0095 * (velocity*3.6 /100)**2) 
    F_rr = c_rr * (TOTAL_WEIGHT * G - lift) # Eq. 26 in research sectino 3.2.4
    return F_rr

# Find V_max 
found = False
while not found:
    skin_friction,drag,lift = aero_force(velocity_max)
    F_rr = rolling_resistance_force(lift, velocity_max)
    totalForce = F_rr + skin_friction + drag
    # print(totalForce)
    power = totalForce * velocity_max
    if power<450: # taken from graph - find power and divide by efficiency to get battery power = 650
        velocity_max += 0.01
    else:
        found = True
print(velocity_max, power)

rpm = 1650 # Power graph
torque = 2.9 

def Calc_rpm(velocity):

    wheel_rpm = velocity * 60 / (math.pi * TYRE_DIAMETER)

    return wheel_rpm

wheel_rpm = Calc_rpm(velocity_max)
print(rpm/wheel_rpm, wheel_rpm)
GEAR_RATIO = rpm/wheel_rpm

def Motor(motor_torque):
    wheel_torque = GEAR_RATIO * motor_torque 
    motor_force = wheel_torque / (TYRE_DIAMETER/2) 

    return motor_force

current = 27
def Battery(SoC, voltage, current, b_power):
    t = H * (C/ (current*H)) ** K # Peukert's Law Eq. 4 in research section 3.2.1 
    SoC = SoC - SoC * time_step/(t*3600)
    # voltage = V_MIN + (V_MAX - V_MIN) * SoC /100 # Eq. 3 in research section 3.2.1
    voltage = Battery_power/current
    return current, SoC, voltage 

def MotorGraphConversions(b_power):
    b_power = b_power*0.7 #efficiency
    motor_torque = -1*math.sqrt((750-b_power)/(375/22.78)) + 13.5/2 # motor graph quadratic

    current = motor_torque*10
    motor_rpm = (-2000/13.5) * motor_torque + 2000 # relationship from Fig 3.7
    return motor_torque, current, motor_rpm


def PowerDegredation(power, time):
    if time<3000:
        power= -1/15 *time +700
    else:
        power = -0.00056 * time **2 + 3.33 * time -4500  # Random graph I made - on chat
        # avg 577.22 watts.
    return power-0.1


def Motor(motor_rpm, motor_torque):
    wheel_rpm = motor_rpm / GEAR_RATIO
    wheel_torque = GEAR_RATIO * motor_torque 
    motor_force = wheel_torque / (TYRE_DIAMETER/2) # Force = moment/distance
    max_motor_velocity = wheel_rpm * (math.pi * TYRE_DIAMETER) / 60 # Max velocity achievable by the car
    return wheel_rpm, wheel_torque, motor_force, max_motor_velocity

def Velocity_Distance(velocity, acceleration, time_step, distance, max_motor_velocity, moving):
    u = velocity 
    velocity = u + acceleration*time_step # v = u + at
    if velocity > max_motor_velocity:
        velocity = max_motor_velocity
    if velocity<0:
        moving = False
    distance += time_step * (u + velocity)/2 # s = (u+v)/2 * t
    return velocity, distance, moving

Socs = []
voltages = []
times = []
currents = []
powers = []
velocities = []
velocity = 0
Battery_power = 650

for i in range(3600):
    Battery_power = PowerDegredation(Battery_power, i)
    motor_torque, current, motor_rpm = MotorGraphConversions(Battery_power)
    current, SoC, voltage = Battery(SoC, voltage, current, Battery_power)
    wheel_rpm, wheel_torque, motor_force, max_motor_velocity =  Motor(motor_rpm, motor_torque)

    found = False
    while not found:
        skin_friction,drag,lift = aero_force(velocity)
        F_rr = rolling_resistance_force(lift, velocity)
        totalForce = F_rr + skin_friction + drag
        # print(totalForce)
        power = totalForce * velocity
        
        if totalForce<motor_force: # taken from graph - find power and divide by efficiency to get battery power = 650
            velocity += 0.01
        else:
            found = True
        
        # 22.46 when power< b_power
        # 17. 5 when power< b_power * 0.7
        # 16.91 when totalForce<motor_force

    print(Battery_power)

    skin_friction,drag,lift = aero_force(velocity)
    F_rr = rolling_resistance_force(lift, velocity)
    resultant_force = motor_force - skin_friction - drag - F_rr
    print(resultant_force, velocity)
    acceleration = resultant_force / TOTAL_WEIGHT

    velocity, distance, moving = Velocity_Distance(velocity, acceleration, time_step, distance, max_motor_velocity, moving)

    powers.append(Battery_power)
    currents.append(current)
    Socs.append(SoC)
    voltages.append(voltage)
    times.append(i)
    velocities.append(velocity)
print(velocity, SoC, distance, acceleration)
plt.plot(times,voltages)
plt.show()
plt.plot(times,Socs)
plt.show()
plt.plot(times,currents)
plt.show()
plt.plot(times,powers)
plt.show()
plt.plot(times,velocities)
plt.show()

