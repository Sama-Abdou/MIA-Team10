# Task7.2

This code implements a Proportional-Integral-Derivative (PID) controller using an Arduino with the Ticker library to control a feedback system.

```cpp
#include <Ticker.h>

// Define analog input signal of feedback
#define INPUT_FEEDBACK A0
```

- Ticker library is used to create a period delay for PID loop without affect rest of code.
- **`INPUT_FEEDBACK`:Define Analog Pin for feedback read from Pin A0.**

# Define PID constants.

```cpp
// Define PID Constats
double Kp = 1000 ;
double Ki = 500 ;
double Kd= 0.5 ;
```

- **`Kp`: is the Proportional constant.**
- **`Ki`:is the integral constant.**
- **`Kd`: is the derivative constant.**

# Define Global variables.

```cpp
// Define some global variables
double setPoint = 90 ;
double outPut, flowRate , prevError, currError , derivative, integration = 0 ;
unsigned long  prevTime , currTime , dt = 0 ;

```

- `setPoint`:
- `outPut`: The output value controlled by the PID controller.
- `flowRate`: The current feedback value from the analog sensor.
- `prevError`: The previous error in the control loop.
- `currError`: The current error in the control loop.
- `derivative`: The derivative term in the PID controller.
- `integration`: The integral term in the PID controller.
- `prevTime`: The previous time for calculating time intervals.
- `currTime`: The current time for calculating time intervals.
- `dt`: The time interval between control loop iterations.

```cpp
Ticker timer1(delaying, 10) ;
```

A Ticker timer named `timer1` is used to call the `delaying()` function every 10 milliseconds to update the PID control loop.

# Delaying function

```cpp
void delaying() {

// Blocked (delayed) code

flowRate = analogRead(INPUT_FEEDBACK) ;    
// Calculate el error  by subtracting setPoint from the feed back (If estimated point of feedBack under setPoint then error will be +ve and viceversa)
currError = setPoint - flowRate ;

currTime = millis() ; // Calculate changing in time
dt = (currTime - prevTime) ;

derivative = (currError - prevError) / dt ;    // Calculate derivative 
integration += currError * dt  // calculate integration

outPut = Kp * currError + Ki * integration + Kd * derivative ;   // Equation of output for PID controller
//------------------ update previous values---------------
prevError = currError ;
prevTime = currTime ;

}
```

This function performs the PID control calculations every 10ms.

1. Read the analog feedback signal from `INPUT_FEEDBACK` (pin A0).
2. calculate the current error `currError` by subtracting the setpoint from the feedback.
3. Measure the current time (`currTime`) in milliseconds.
4. Calculate the time interval (`dt`) by subtracting the previous time from the current time.
5. Calculate the derivative term using `(currError - prevError) / dt`.
6. Update the integral term by adding `currError * dt` to the integration variable.
7. Calculate the PID output using the PID equation: `outPut = Kp * currError + Ki * integration + Kd * derivative`.
8. Update the previous error and previous time values for the next iteration.

# Setup Function

```cpp
void setup() {
  
timer1.start(); 
Serial.begin(9600);

}
```

1. Start the timer1 Ticker timer with a 10ms interval.
2. Initialize serial communication.

 

# Loop Function

```cpp
void loop() {      
 timer1.update();
// Non blocked code 

}
```

1. Call `timer1.update()` to allow the Ticker timer to execute the `delaying()` function.