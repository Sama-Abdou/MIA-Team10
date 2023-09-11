#define pwm1 3 // PWM1 pin connected to digital pin 3
#define dir1 4  // DIR1 pin connected to digital pin 4

float alpha = 0.1; // Alpha constant which is used in the filter
float lastAvg = 0.0; // A variable to store the last average taken

void setup() {
  pinMode(pwm1, OUTPUT); // Setting the mode of the PWM1 pin to OUTPUT
  pinMode(dir1, OUTPUT);  // Setting the mode of the DIR1 pin to OUTPUT
  Serial.begin(9600);
}

void loop() {
  byte currentAvg = 0; // A variable to store the current average between 0-255 to be assigned to the PWM1
  byte currentValue = 250; // The value we want to reach by the end
  currentAvg = (currentValue*alpha) + ((lastAvg)*(1-alpha)); // The exponential smoothing equation
  lastAvg = currentAvg; // Setting the last average which will be used in the next loop
  digitalWrite(dir1, HIGH); // Setting the direction of the motor
  analogWrite(pwm1, currentAvg); // Setting the speed of the motor
  Serial.println(currentAvg);
  delay(100);
}

