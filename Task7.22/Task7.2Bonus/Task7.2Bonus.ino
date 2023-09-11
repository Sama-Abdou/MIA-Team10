class PID {
public:
   PID(double Kp, double Ki, double Kd, double setPoint) { 
    this->Kp = Kp ;
    this->Ki = Ki ;
    this->Kd = Kd ;
    this->setPoint = setPoint ;
    
  }
  
  void OUTPUT() {
    // Code will run after this delay
    flowRate = analogRead(INPUT_FEEDBACK);

    // Calculate el error  by subtracting setPoint from the feed back (If estimated point of feedBack under setPoint then error will be +ve and viceversa)
    currError = setPoint - flowRate;
    
    currTime = millis(); // Calculate change in time
    dt = currTime - prevTime;


    derivative = (currError - prevError) / dt ;    // Calculate derivative 
    integration += currError * dt  // calculate integration


   outPut = Kp * currError + Ki * integration + Kd * derivative ;   // Equation of output for PID controller

    //------------------ update previous values---------------
    prevError = currError ;
    prevTime = currTime ;

  }
  void DISPLAY( {
     Serial.print(", Output: ");
    Serial.println(outPut);
  })

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

// Define analog input signal of feedback
#define INPUT_FEEDBACK A0

PID PID1;

void setup() {
    Serial.begin(9600);
    Serial.println("Enter Kp, Ki, Kd, setpoint respectively);
    if (Serial.available()) {
    // Read user input from Serial Monitor
        double Kp, Ki, Kd, setPoint;
        int parsed = Serial.parseInt();
      if (parsed == 4) {
            Kp = Serial.parseFloat();
            Ki = Serial.parseFloat();
            Kd = Serial.parseFloat();
            SetPoint = Serial.parseFloat();
            PID1 = PID(Kp, Ki, Kd, setPoint)
            Serial.println("PID parameters are successfully entered.");
            }
      }
 
}

void loop() {
 
  PID1.OUTPUT();
  PID1.DISPLAY();
}
