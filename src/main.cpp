
#include <Arduino.h>
#include <Adafruit_PWMServoDriver.h>
#include <string.h>

//Create object to represent PCA9685 at default I2C address
Adafruit_PWMServoDriver pca9685 = Adafruit_PWMServoDriver(0x40);
Adafruit_PWMServoDriver pca96852 = Adafruit_PWMServoDriver(0x41);

#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates

// Define maximum and minimum number of "ticks" for the servo motors
// Range from 0 to 4095
// This determines the pulse width

#define SERVOMIN 90 // Minimum value
#define SERVOMAX 500 // Maximum value

// Define servo motor connections (expand as required)
#define SERP1 0   // Pan Servo Motor 1 o'clock on connector 1
#define SERP2  1  // Pan Servo motor 2 o'clock on connector 2
#define SERP3  2  // Pan Servo motor 3 o'clock on connector 3
#define SERP4 3   // Pan Servo motor 4 o'clock on connector 4
#define SERP5 4   // Pan Servo motor 5 o'clock on connector 5
#define SERP6 5   // Pan Servo motor 6 o'clock on connector 6
#define SERP7 6   // Pan Servo motor 7 o'clock on connector 7
#define SERP8 7   // Pan Servo motor 8 o'clock on connector 8
#define SERP9 8   // Pan Servo motor 9 o'clock on connector 9
#define SERP10 9   // Pan Servo motor 10 o'clock on connector 10
#define SERP11 10   // Pan Servo motor 11 o'clock on connector 11
#define SERP12 11   // Pan Servo motor 12 o'clock on conncetor 12


#define SERT1 0   // Tilt Servo Motor 1 o'clock on connector 1
#define SERT2  1  // Tilt Servo motor 2 o'clock on connector 2
#define SERT3  2  // Tilt Servo motor 3 o'clock on connector 3
#define SERT4 3   // Tilt Servo motor 4 o'clock on connector 4
#define SERT5 4   // Tilt Servo motor 5 o'clock on connector 5
#define SERT6 5   // Tilt Servo motor 6 o'clock on connector 6
#define SERT7 6   // Tilt Servo motor 7 o'clock on connector 7
#define SERT8 7   // Tilt Servo motor 8 o'clock on connector 8
#define SERT9 8   // Tilt Servo motor 9 o'clock on connector 9
#define SERT10 9   // Tilt Servo motor 10 o'clock on connector 10
#define SERT11 10   // Tilt Servo motor 11 o'clock on connector 11
#define SERT12 11   // Tilt Servo motor 12 o'clock on conncetor 12






long previousMillis = 0;
long interval = 10000;

int posDegrees = 0;

int posDelta = 1;

// timestamp
uint32_t ts = 0;

uint32_t td = 0;  //time delay for motion
  int pwm0 = map(posDegrees, 0, 180, SERVOMIN, SERVOMAX);
void calibrate() {
  for(int i = 0 ; i < 10 ; i++) {
  posDegrees = 0;
  pca9685.setPWM(SERP1, 0, pwm0);
  delay(1000);
  posDegrees = 180;
  pca9685.setPWM(SERP1, 0, pwm0);
  delay(1000);
  }

}

void setup() {

  Serial.begin(115200);
  
  // Print to monitor
  Serial.println("PCA9685 Servo Test");

  // Initialise PCA9685
  pca9685.begin();
  pca96852.begin();

  //set PWM Frequency to 50Hz
  pca9685.setPWMFreq(SERVO_FREQ);
  pca96852.setPWMFreq(SERVO_FREQ);

}

void loop() {
  //calibrate();
  
  // only print every 100 millis
  if(millis()>ts+100) {
    ts = millis();
    // Print to serial monitor
    Serial.print("Pos degrees = ");
    Serial.println(posDegrees);
  }
  int pwm0 = map(posDegrees, 0, 180, SERVOMIN, SERVOMAX);
  int pwm0_rev = map(180-posDegrees, 0, 180, SERVOMIN, SERVOMAX);
  
  //int pwm2 = map(posDegrees, 180, 0, SERVOMIN, SERVOMAX);
  //int pwm2_rev = map(180-posDegrees, 0, 180, SERVOMIN, SERVOMAX);
  
  
  pca9685.setPWM(SERP1, 0, pwm0);
  pca9685.setPWM(SERP2, 0, pwm0_rev);
  pca9685.setPWM(SERP3, 0, pwm0);
  pca9685.setPWM(SERP4, 0, pwm0_rev);
  pca9685.setPWM(SERP5, 0, pwm0);
  pca9685.setPWM(SERP6, 0, pwm0_rev);
  pca9685.setPWM(SERP7, 0, pwm0);
  pca9685.setPWM(SERP8, 0, pwm0_rev);

  //myBrief test
  pca96852.setPWM(SERT1, 0, pwm0);
  pca96852.setPWM(SERT4, 0, 30);
  //end of myBrief Test


   // delay(10);
  if(millis()>td+10) {
    td = millis();
    // 
    
  
  //Attempt to narrow the start postion
  if(posDegrees<1) {
    posDegrees=1;
  }
  
  // increment posDegrees as necessary
  if(posDegrees+posDelta>180) {
    posDelta = -posDelta;
  } else if(posDegrees+posDelta<1) {
    posDelta = -posDelta;
  }
  
  posDegrees += posDelta;
  
  
  }
  //delay(1000);
  
  
  
  
}