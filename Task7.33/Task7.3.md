# Task7.3

# Libraries

```python
import numpy as np
import time
import pyfirmata as ard
```

`pyfirmata`: Used for Arduino communication.

`numpy`: A library for numerical computations.

`time`: Used for handling time-related operations.

# calculate_angular

```
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
```

This function calculate the angular velocity of each motor
`R` Distance between wheel and center
`vx`the linear velocity in x direction 

`vy` the linear velocity in x direction 

`omega` angular velocity

`alpha1` The angle between x-axis and first wheel

`alpha2` The angle between x-axis and first wheel

`alpha3` The angle between x-axis and first wheel

# set_motor_speeds

```python
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
```

This function sets the motor speeds and directions for a robot with three wheels based on the calculated angular velocities `w1`, `w2`, and `w3`. It interfaces with an Arduino board using the PyFirmata library to control the motors.

`board:` The Arduino board object representing the hardware connection.
`motor_pins:` A list of pins connected to the motors. It should contain three pin numbers.
`direction_pins:` A list of pins used to control the direction of the motors. It should also contain three pin numbers.
`motor_min:` The minimum motor speed value which is 0.
`motor_max`: The maximum motor speed value which is 255
`w1` The angular velocity of the first wheel.
`w2` The angular velocity of the second wheel.
`w3` The angular velocity of the third wheel.

# PIDCalculation

```python
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
```

This function implements a PID controller to calculate linear velocities `vx` and `vy` and `omega` for a robot. 

`setPointX`The desired X-coordinate setpoint for the robot's position.
`setPointY` The desired Y-coordinate setpoint for the robot's position.
`setPointOmega` The desired angular velocity setpoint for the robot
`robot_x` The current X-coordinate of the robot's position.
`robot_y` The current Y-coordinate of the robot's position.

1. Initialize variables for previous errors and integral terms (`prevError_X`, `prevError_Y`, ``prevError_Omega`, `integrationX`, `integrationY`, `integrationOmega``) to maintain control history.

2. Calculate the error in the X-coordinate position (`currErrorX`) as the difference between the desired X-coordinate (`setPointX`) and the current X-coordinate (`robot_x`).

3. Calculate the error in the Y-coordinate position (`currErrorY`) as the difference between the desired Y-coordinate (``setPointY``) and the current Y-coordinate (`robot_y`).

4. Calculate the error in the angular velocity (`currErrorOmega`) as the difference between the desired angular velocity (`setPointOmega`) and the previous angular velocity (`prevError_Omega`).

5. Compute the derivative terms for X (`derivativeX`), Y (`derivativeY`), and angular velocity (`derivativeOmega`) by calculating the rate of change of their respective errors with respect to time.

6. Update the integral terms for X (`integrationX`), Y (`integrationY`), and angular velocity (`integrationOmega`) by accumulating the errors over time.

7. Calculate the control outputs:
   - Linear velocity in the X-direction (``vx``) as a combination of the proportional, integral, and derivative terms.
   - Linear velocity in the Y-direction (`vy`) similarly to`vx`.
   - Angular velocity (`omega`) also as a combination of the proportional, integral, and derivative terms.

8. Update the previous error values for X, Y, and angular velocity to be used in the next iteration.

9. Return the calculated `vx`, `vy`, and `omega`as the control outputs.

# Main code

```python
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
```

Define our variables in `PIDCalculation` and `set_motor_speeds` 

This for loop to continuously compute control commands for the robot's motion based on the desired setpoints (`setPointX`, `setPointY`, `setPointOmega`) and the robot's current position (`robot_x`, `robot_y`).