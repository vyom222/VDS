import math
def MotorTemperature(temperature):
    Motor_Resistance= MOTOR_INITIAL_RESISTANCE * (1+0.004*(temperature-20)) # Eq.12
    I_Stall = voltage/Motor_Resistance
    Torque_constant = ORIGINAL_TORQUE_CONSTANT * (1-0.0012*(temperature-20))
    T_stall = I_Stall * Torque_constant
    no_load = 9.5493 * (voltage- (NO_LOAD_CURRENT * Motor_Resistance)) /Torque_constant 
    omega = voltage/ Torque_constant 
    P_max = 0.25 * omega * T_stall
    print(temperature)
    return P_max, no_load, T_stall, I_Stall

def MotorGraphConversions(b_power, P_max, no_load, T_stall, I_Stall):
    b_power = b_power*0.7 #efficiency
    # b_power = (-motor_torque**2 + motor_torque * T_stall) * P_max / (-(T_stall/2)**2+ (T_stall/2) * T_stall)
    motor_torque = -1*math.sqrt(-1*((b_power / (P_max / (T_stall/2)**2)) - (T_stall/2)**2)) + T_stall/2
    # motor_torque = -1*math.sqrt((750-b_power)/(375/22.78)) + 13.5/2 # motor graph quadratic
    current = motor_torque*I_Stall/T_stall + NO_LOAD_CURRENT # y = mx+c
    motor_rpm = (-no_load/T_stall) * motor_torque + no_load # y= mx+c
    return motor_torque, current, motor_rpm

motor_temperature = 20
ORIGINAL_TORQUE_CONSTANT = 0.104
BACK_EMF_CONSTANT = 1.15
I_stall = 130
NO_LOAD_CURRENT = 0.5
voltage = 24
MOTOR_INITIAL_RESISTANCE = 0.184
b_power = 650

for i in range(5):
    P_max, no_load, T_stall, I_Stall = MotorTemperature(20+i*10)
    # print(P_max, no_load, T_stall, I_Stall)
    motor_torque, current, motor_rpm = MotorGraphConversions(b_power, P_max, no_load, T_stall, I_Stall)
    print(motor_torque, current, motor_rpm)