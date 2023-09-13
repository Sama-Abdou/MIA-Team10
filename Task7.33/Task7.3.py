import numpy as np
import time
#import pyfirmata as ard

# Function to calculate w1, w2, w3
def calculate_angular(vx, vy, omega, alpha1, alpha2, alpha3):
    R=0.2 
    matrix = np.array([
        [np.cos(alpha1 + np.pi / 2), np.cos(alpha2 + np.pi / 2), np.cos(alpha3 + np.pi / 2)],
        [np.sin(alpha1 + np.pi / 2), np.sin(alpha2 + np.pi / 2), np.sin(alpha3 + np.pi / 2)],
        [1/R, 1/R, 1/R]
    ])
    input_vector = np.array([vx, vy, omega])
    output_vector = np.dot(np.linalg.inv(matrix), input_vector)
    return output_vector

# Function to set motor speeds and directions
def set_motor_speeds(board, motor_pins, direction_pins, motor_min, motor_max, w1, w2, w3):
    pwm1 = int((w1 + 1) / 2 * (motor_max - motor_min) + motor_min)
    pwm2 = int((w2 + 1) / 2 * (motor_max - motor_min) + motor_min)
    pwm3 = int((w3 + 1) / 2 * (motor_max - motor_min) + motor_min)

    direction1 = board.HIGH if w1 >= 0 else board.LOW
    direction2 = board.HIGH if w2 >= 0 else board.LOW
    direction3 = board.HIGH if w3 >= 0 else board.LOW

    board.digital[direction_pins[0]].write(direction1)
    board.analog[motor_pins[0]].write(pwm1 / 255)

    board.digital[direction_pins[1]].write(direction2)
    board.analog[motor_pins[1]].write(pwm2 / 255)

    board.digital[direction_pins[2]].write(direction3)
    board.analog[motor_pins[2]].write(pwm3 / 255)

# Function for PID control
def PIDcalculation(setPointX, setPointY,setPointOmega, robot_x, robot_y):
        prevError_X = 0
        prevError_Y = 0
        prevError_Omega=0
        integrationX = 0
        integrationY = 0
        integrationOmega = 0
#-----------X-Calculations-----------------------
        currErrorX = setPointX - robot_x
        derivativeX = (currErrorX - prevError_X) / dt
        integrationX += currErrorX * dt
#-------------Y-calculation-------------------
        currErrorY = setPointY - robot_y
        derivativeY = (currErrorY - prevError_Y) / dt
        integrationY += currErrorY * dt
 #-----------W-Calculation---------------
        currErrorOmega=setPointOmega-prevError_Omega
        derivativeOmega  = (currErrorOmega- prevError_Omega) / dt
        integrationOmega += currErrorOmega * dt
 #--------------Calculate Outputs----------------------
        vx = Kp * currErrorX + Ki * integrationX + Kd * derivativeX
        vy = Kp * currErrorY + Ki * integrationY + Kd * derivativeY   
        omega = Kp * currErrorOmega + Ki * integrationOmega + Kd * derivativeOmega
      #----------renew points for prevErrors for next iterations--------------
        prevError_Omega=currErrorOmega
        prevError_X=currErrorX
        prevError_Y=currErrorY
    
        return vx, vy, omega


    
   #---------------------------------


board = ard.Arduino('/dev/ttyACM0')
motor_pins = [3, 5, 6]
direction_pins = [2, 4, 7]
motor_min = 0
motor_max = 255

it = ard.util.Iterator(board)
it.start()

for pin in motor_pins:
    board.analog[pin].mode = board.PWM
    
alpha1=30
alpha2=150            # alpha is angle from x-axis to motor axis
alpha3=270
vx = float(input("vx: "))
vy = float(input("vy: "))
omega = float(input("omega: "))


output_vector = calculate_angular(vx, vy, omega,alpha1,alpha2,alpha3)
w1, w2, w3 = output_vector
print("w1 = ", w1)
print("w2 = ", w2)
print("w3 = ", w3)
set_motor_speeds(board, motor_pins, direction_pins, motor_min, motor_max, w1, w2, w3)
Kp = 10
Ki = 0.5
Kd = 0.1
dt=0.1
robot_x = 0
robot_y = 0
setPointX = 5
setPointY = 5
setPointOmega= 5
x_positions = []
y_positions = []
for _ in range(50):
     vx, vy, omega = PIDcalculation(setPointX, setPointY, setPointOmega, robot_x, robot_y)
     robot_x += vx * dt
     robot_y += vy * dt
     x_positions.append(robot_x)
     y_positions.append(robot_y)








