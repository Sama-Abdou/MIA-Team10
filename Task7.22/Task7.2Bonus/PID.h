#ifndef PID_H
#define PID_H
#include <Arduino.h>
/**
 * @class PID
 * @brief class for implementing PID controller
 */
class PID {
public:
/**
 * @brief Set the PID parameters and setpoint
 * @param Kp The proportional term.
 * @param Ki The integral term.
 * @param Kd The derivative term.
 * @param setPoint The desired setpoint.
 */  
  void setParameters(double Kp, double Ki, double Kd, double setPoint);
   /**
    * @brief Compute the PID control loop.
    * @param feedBack The feedback value from the analog sensor.
    */
  void computeOutput(double feedBack) ;
   /**
    * @brief Display the outout.
    */   
  void displayOutput();

private:
  double Kp ;
  double Ki ;
  double Kd ;
  double setPoint ;
  double outPut = 0 ;
  double feedBack = 0 ;
  double prevError = 0 ;
  double currError = 0 ;
  double derivative = 0 ;
  double integration = 0 ;
  unsigned long prevTime = 0 ;
  unsigned long currTime = 0 ;
  unsigned long dt = 0 ;
};

#endif