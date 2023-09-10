int flowPin = 2;    //Input Pin on the Arduino
double flowRate;
volatile int count;
const int OUTPUT_PIN = DD3;

double dt, last_time;
double proportional, integral, derivative, previous, output = 0;
double kp, ki, kd;
double setpoint = 90;

void setup(){
  pinMode(flowPin, INPUT);
  attachInterrupt(0, Flow, RISING);
  kp = 0.8;
  ki = 0.20;
  kd = 0.001;
  last_time = 0;
  Serial.begin(9600);
  delay(100);
}

void loop(){
  count = 0;
  interrupts();
  delay (1000);
  noInterrupts();
  flowRate = (count * (2.25/1699010.869));       //Getting flowRate in ml/sec the converting it in CFM (Cubic feet per minute)
  
  double now = millis();
  dt = (now - last_time)/1000.00;
  last_time = now;

  double actual = flowRate;
  double error = setpoint - actual;
  output = pid(error);

  analogWrite(OUTPUT_PIN, output);

  Serial.print(setpoint);
  Serial.print(",");
  Serial.println(actual);

  delay(300);
}

double pid(double error){
  proportional = error;
  integral += error * dt;
  derivative = (error - previous) / dt;
  previous = error;
  double output = (kp * proportional) + (ki * integral) + (kd * derivative);
  return output;
}

void Flow(){
   count++;
}