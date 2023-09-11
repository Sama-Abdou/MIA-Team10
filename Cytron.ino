#define pwm1 3
#define dir1 4
void setup() {
  // put your setup code here, to run once:
  pinMode(pwm1, OUTPUT);
  pinMode(dir1, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(dir1, HIGH);
  analogWrite(pwm1, 150);
}
