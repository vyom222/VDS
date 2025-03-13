# add random subprograms for future proffing, motor temp, mgsin theta
# Just model battery curve and then don't recalculate it
import math
import matplotlib.pyplot as plt
import numpy as np

# Initialise variables
time = 0
time_step = 1
distance = 0
session_length = 3600 
moving = True

CAR_WEIGHT = 65
DRIVER_WEIGHT = 65
TOTAL_WEIGHT = CAR_WEIGHT + DRIVER_WEIGHT
G = 9.81

c_rr = 0 # coeff of rolling resistance
TYRE_PRESSURE_PSI = 40 # psi
TYRE_PRESSURE = TYRE_PRESSURE_PSI / 14.504 # convert to bar

C_D = 1.07 # coeff of drag
C_L = 0.89 # coeff of lift
C_S = 0.011 # coeff of skin friction
RHO = 1.225 # fluid density
CSA = 0.1535 # cross-sectional area
WIND_SPEED = 0
wind_direction = 0 # angle in degrees

V_MAX = 24 # 100% SoC
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
MOTOR_INITIAL_RESISTANCE = 0.063 #### CHECK THIS
DIA_MOTOR_GEAR = 1
DIA_AXLE_GEAR = 2.75
GEAR_RATIO = DIA_AXLE_GEAR/ DIA_MOTOR_GEAR
TRANSMISSION_EFFICIENCY = 0.9
DISTANCE_FROM_MOTOR = 0.5

motor_temperature = 20
ORIGINAL_TORQUE_CONSTANT = 0.104
BACK_EMF_CONSTANT = 1.15
I_stall = 130
NO_LOAD_CURRENT = 0.5

resultant_force = 0
acceleration = 0
u = 0 # inital velocity
velocity = 0 # m/s

velocity_max = 0
power = 0
lap_progress = 0
downhill_force = 0

Socs = []
voltages = []
times = []
currents = []
powers = []
velocities = []
velocity = 0
Battery_power = 650

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

def TrackElevation(lap_progress):
    if lap_progress<10:
        downhill_force = 0
    elif lap_progress<20:
        downhill_force = TOTAL_WEIGHT*9.81 * math.sin(math.pi/180)
    elif lap_progress<25:
        downhill_force = TOTAL_WEIGHT*9.81 * math.sin(math.pi/180)
    else:
        downhill_force = 0
    return lap_progress, downhill_force

def Motor(motor_torque):
    wheel_torque = GEAR_RATIO * motor_torque 
    motor_force = wheel_torque / (TYRE_DIAMETER/2) 

    return motor_force

def Battery(SoC, voltage, current, Battery_power):
    t = H * (C/ (current*H)) ** K # Peukert's Law Eq. 4 in research section 3.2.1 
    SoC = SoC - SoC * time_step/(t*3600)
    # voltage = V_MIN + (V_MAX - V_MIN) * SoC /100 # Eq. 3 in research section 3.2.1
    voltage = Battery_power/current
    return current, SoC, voltage 

def MotorGraphConversions(b_power, P_max, no_load, T_stall, I_Stall):
    b_power = b_power*0.7 #efficiency
    # b_power = (-motor_torque**2 + motor_torque * T_stall) * P_max / (-(T_stall/2)**2+ (T_stall/2) * T_stall)
    motor_torque = -1*math.sqrt(-1*((b_power / (P_max / (T_stall/2)**2)) - (T_stall/2)**2)) + T_stall/2
    # motor_torque = -1*math.sqrt((750-b_power)/(375/22.78)) + 13.5/2 # motor graph quadratic
    current = motor_torque*I_Stall/T_stall + NO_LOAD_CURRENT # y = mx+c
    motor_rpm = (-no_load/T_stall) * motor_torque + no_load # y= mx+c
    return motor_torque, current, motor_rpm

def Temperature(temperature):
    #increase to 80˚C
    return temperature + 60/3600

def PowerDegredation(power, time):
    if time<3000:
        power= -1/15 *time +700
    else:
        power = -0.00056 * time **2 + 3.33 * time -4450  # Random graph I made - on chat
        # avg 577.22 watts.
    return power-0.1

def MotorTemperature(temperature):
    Motor_Resistance= MOTOR_INITIAL_RESISTANCE * (1+0.004*(temperature-20)) # Eq.12
    I_Stall = voltage/Motor_Resistance
    Torque_constant = ORIGINAL_TORQUE_CONSTANT * (1-0.0012*(temperature-20))
    T_stall = I_Stall * Torque_constant
    no_load = 9.5493 * (voltage- (NO_LOAD_CURRENT * Motor_Resistance)) /Torque_constant 
    omega = voltage/ Torque_constant 
    P_max = 0.25 * omega * T_stall
    return P_max, no_load, T_stall, I_Stall

def Motor(motor_rpm, motor_torque):
    wheel_rpm = motor_rpm / GEAR_RATIO
    wheel_torque = GEAR_RATIO * motor_torque 
    motor_force = wheel_torque / (TYRE_DIAMETER/2) # Force = moment/distance
    max_motor_velocity = wheel_rpm * (math.pi * TYRE_DIAMETER) / 60 # Max velocity achievable by the car
    return wheel_rpm, wheel_torque, motor_force, max_motor_velocity

def VelocityDistance(velocity, acceleration, time_step, distance, max_motor_velocity, moving):
    u = velocity 
    velocity = u + acceleration*time_step # v = u + at
    if velocity > max_motor_velocity:
        velocity = max_motor_velocity
    if velocity<0:
        moving = False
    distance += time_step * (u + velocity)/2 # s = (u+v)/2 * t
    return velocity, distance, moving

def MaxSpeed(max_power, velocity, lap_progress):
    found = False
    while not found:
        skin_friction,drag,lift = aero_force(velocity)
        F_rr = rolling_resistance_force(lift, velocity)
        lap_progress, downhill_force = TrackElevation(lap_progress)
        totalForce = F_rr + skin_friction + drag + downhill_force
        # print(totalForce)

        power = totalForce * velocity
        if power< max_power: # taken from graph - find power and divide by efficiency to get battery power = 650
            velocity += 0.01
        else:
            found = True
    return skin_friction,drag,lift, F_rr, lap_progress, downhill_force, totalForce, power, velocity
        

########### Find opitmum gear ratio

# Find V_max 
# 450 taken from graph - find power and divide by efficiency to get battery power = 650
skin_friction,drag,lift, F_rr, lap_progress, downhill_force, totalForce, power, velocity_max = MaxSpeed(450,velocity, lap_progress)
print(velocity_max, power)

rpm = 1650 # Power graph
torque = 2.9 

def Calc_rpm(velocity):

    wheel_rpm = velocity * 60 / (math.pi * TYRE_DIAMETER)

    return wheel_rpm

wheel_rpm = Calc_rpm(velocity_max)
print(rpm/wheel_rpm, wheel_rpm)
GEAR_RATIO = rpm/wheel_rpm
##########


for i in range(session_length):
    Battery_power = PowerDegredation(Battery_power, i)
    motor_temperature = Temperature(motor_temperature)
    P_max, no_load, T_stall, I_Stall = MotorTemperature(motor_temperature)
    motor_torque, current, motor_rpm = MotorGraphConversions(Battery_power, P_max, no_load, T_stall, I_Stall)
    current, SoC, voltage = Battery(SoC, voltage, current, Battery_power)
    wheel_rpm, wheel_torque, motor_force, max_motor_velocity =  Motor(motor_rpm, motor_torque)
    voltage = 24
    print(P_max, no_load, T_stall, I_Stall, voltage)
    print(motor_torque, current, motor_rpm)

    if not moving:
        break

    skin_friction,drag,lift, F_rr, lap_progress, downhill_force, totalForce, power, velocity = MaxSpeed(Battery_power*0.7,velocity, lap_progress)
        # 22.46 when power< b_power
        # 17. 5 when power< Battery_power * 0.7
        # 16.91 when totalForce<motor_force

    # track progress around to model elevation changes
    lap_progress+=1
    if lap_progress>100:
        lap_progress = 0

    resultant_force = motor_force - skin_friction - drag - F_rr - downhill_force
    # print(resultant_force, velocity)
    acceleration = resultant_force / TOTAL_WEIGHT

    velocity, distance, moving = VelocityDistance(velocity, acceleration, time_step, distance, max_motor_velocity, moving)

    powers.append(Battery_power)
    currents.append(current)
    Socs.append(SoC)
    voltages.append(voltage)
    times.append(i)
    velocities.append(velocity)

print(velocity, SoC, distance, acceleration)
print('gear ratio', GEAR_RATIO)
fig, axs = plt.subplots(2, 2)
axs[0, 0].plot(times,currents)
axs[0, 0].set_title('Current')
axs[0, 1].plot(times,velocities, 'tab:orange')
axs[0, 1].set_title('Velocity')
axs[1, 0].plot(times,voltages, 'tab:green')
axs[1, 0].set_title('Voltage')
axs[1, 1].plot(times,powers, 'tab:red')
axs[1, 1].set_title('Power')
fig.tight_layout()

plt.show()