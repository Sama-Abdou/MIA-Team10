
// Define analog input signal of feedback
#define INPUT_FEEDBACK A0

// Define PID Constats
double Kp = 1000 ;
double Ki = 500 ;
double Kd= 0.5 ;
 // Define some global variables
double setPoint = 90 ;
double outPut, flowRate , prevError, currError , derivative, integration = 0 ;
unsigned long  prevTime , currTime , dt = 0 ;


//------------------------------------------------------------
void setup() {
pinMode(INPUT_FEEDBACK,INPUT);
Serial.begin(9600);

}
//--------------------------------------------------------------
void loop() {  
  
     flowRate = analogRead(INPUT_FEEDBACK) ; // Read input from analog PIN
      
  // Calculate el error  by subtracting setPoint from the feed back (If estimated point of feedBack under setPoint then error will be +ve and viceversa)
     currError = setPoint - flowRate ;

      currTime = millis() ; // Calculate changing in time
      dt = (currTime - prevTime) ;


      derivative = (currError - prevError) / dt ;    // Calculate derivative 
      integration += currError * dt ; // calculate integration

       outPut = Kp * currError + Ki * integration + Kd * derivative ;   // Equation of output for PID controller
  //------------------ update previous values---------------
       prevError = currError ;
       prevTime = currTime ;



}
