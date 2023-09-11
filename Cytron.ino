#define pwm1 3
#define dir1 4

float alpha = 0.2;
float lastAvg = 0.0;

void setup() {
  pinMode(pwm1, OUTPUT);
  pinMode(dir1, OUTPUT);
  Serial.begin(9600);
}

void loop() {
  byte currentAvg = 0;
  byte currentValue = 250;
  currentAvg = (currentValue*alpha) + ((lastAvg)*(1-alpha));
  lastAvg = currentAvg;
  digitalWrite(dir1, HIGH);
  analogWrite(pwm1, currentAvg);
  Serial.println(currentAvg);
  delay(100);
}

