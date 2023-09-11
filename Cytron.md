# Cytron

---

This code demonstrates PWM control with a soft start using an exponential filter. It uses two digital pins for controlling a motor: **pwm1** for the PWM signal and **dir1** for the direction signal. 

## Pins Declaration and Variables Used

---

- **pwm1**: This pin is used for generating the PWM signal to control the motor speed. It is defined on pin number 3.
- **dir1**: This pin is used to set the direction of the motor rotation. It is defined on pin number 4.
- **alpha**: smoothing factor of data; 0 < α < 1 It is defined as a float with a value of 0.1.
- **lastAvg**: This variable holds the previous filtered value. It is initialized to 0.0.

```arduino
#define pwm1 3
#define dir1 4

float alpha = 0.1;
float lastAvg = 0.0;
```

## Set Up **Function**

---

In this function, the pin modes for **pwm1** and **dir1** are set to OUTPUT. The serial communication is also initialized with a baud rate of 9600.

```arduino
void setup() {
  pinMode(pwm1, OUTPUT);
  pinMode(dir1, OUTPUT);
  Serial.begin(9600);
}
```

## Loop Funtion

---

- Declaring **currentAvg** as a byte variable and initializes it to 0.
- Declares **currentValue** as a byte variable and initializes it to 250.
- Calculates the new filtered value using the exponential filter formula:

**y[n] = α * y[n-1] + (1-α ) * x[n]**. ****The exponential filter gradually adjusts the motor speed by smoothing out sudden changes in the **currentValue**, providing a soft start effect. The **alpha** value determines the rate of change in the filter response. Higher values of **alpha** will result in faster response but less smoothing, while lower values of `alpha` will result in slower response but more smoothing.

- Updates **lastAvg** with the new **currentAvg** value.
- Sets the direction of the motor rotation by setting the **dir1** pin to **HIGH**.
- Sets the motor speed by using the analogWrite() function to output the **currentAvg** value to the **pwm1** pin.
- Prints the **currentAvg** value to the serial monitor.
- Adds a delay of 100 milliseconds to control the loop execution speed.

```arduino
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
```