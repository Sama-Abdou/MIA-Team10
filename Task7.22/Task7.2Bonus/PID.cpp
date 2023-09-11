#include "PID.h"

  void PID::setParameters(double Kp, double Ki, double Kd, double setPoint){
    this->Kp = Kp ;
    this->Ki = Ki ;
    this->Kd = Kd ;
    this->setPoint = setPoint ;
    
  }
  void PID::computeOutput(double flowRate) {
    // Code will run after this delay
    this->flowRate = flowRate ;

    // Calculate el error  by subtracting setPoint from the feed back (If estimated point of feedBack under setPoint then error will be +ve and viceversa)
    currError = setPoint - flowRate;
    
    currTime = millis(); // Calculate change in time
    dt = currTime - prevTime;


    derivative = (currError - prevError) / dt ;    // Calculate derivative 
    integration += currError * dt ; // calculate integration


  outPut = Kp * currError + Ki * integration + Kd * derivative ;   // Equation of output for PID controller

    //------------------ update previous values---------------
    prevError = currError ;
    prevTime = currTime ;
  
  }
  void PID::displayOutput(){
  Serial.print("Output = ");
  Serial.println(outPut);
  }
