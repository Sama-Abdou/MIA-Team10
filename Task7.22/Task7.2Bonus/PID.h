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
 * @param Kp The proportional gain.
 * @param Ki The integral gain.
 * @param Kd The derivative gain.
 * @param _setPoint The desired setpoint.
*/  
  void setParameters(double Kp, double Ki, double Kd, double setPoint);
  
  void computeOutput(double flowRate) ;

  
  void displayOutput();

private:
  double Kp ;
  double Ki ;
  double Kd ;
  double setPoint ;
  double outPut = 0 ;
  double flowRate = 0 ;
  double prevError = 0 ;
  double currError = 0 ;
  double derivative = 0 ;
  double integration = 0 ;
  unsigned long prevTime = 0 ;
  unsigned long currTime = 0 ;
  unsigned long dt = 0 ;
};

#endif