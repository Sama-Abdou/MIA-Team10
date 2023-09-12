
#include "PID.h"
// Define analog input signal of feedback
#define INPUT_FEEDBACK A0

PID PID1;

void setup() {
     pinMode(INPUT_FEEDBACK,INPUT);
     PID1.setParameters(5000,1000,0.5,90,350);
     Serial.begin(9600);
       
            }
      
void loop() {
  
  
 double feedBack = analogRead(INPUT_FEEDBACK);
  PID1.computeOutput(feedBack);
  PID1.displayOutput();
 
}
