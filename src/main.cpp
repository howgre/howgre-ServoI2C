
#include <Arduino.h>
#include <Adafruit_PWMServoDriver.h>


//Create object to represent PCA9685 at default I2C address
Adafruit_PWMServoDriver pwms[] {
  Adafruit_PWMServoDriver(0x40),
  Adafruit_PWMServoDriver(0x41) 
};
const size_t pwms_count = sizeof(pwms)/sizeof(Adafruit_PWMServoDriver);

#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates

// Define maximum and minimum number of "ticks" for the servo motors
// Range from 0 to 4095
// This determines the pulse width

#define SERVOMIN 90 // Minimum value
#define SERVOMAX 500 // Maximum value

int maxPanAngle[] = {180, 90, 45, 135};
const size_t maxPanAngle_count = sizeof(maxPanAngle)/sizeof(maxPanAngle[0]);

int minPanAngle[] = {0, 90, 45, 135};
const size_t minPanAngle_count = sizeof(minPanAngle)/sizeof(minPanAngle[0]);





void servo_fn_up_down(int& i, int& delta) {
  if(i+delta>maxPanAngle[0] || i+delta<minPanAngle[0]) {
    delta=-delta;
  }
  i+=delta;
}
void servo_fn_wobble(int& i, int& delta) {
  if(i+delta<minPanAngle[2] || i+delta>maxPanAngle[3]) {
    delta=-delta;
  }
  i+=delta;
}

typedef void(*servo_fn)(int& i,int& delta);

struct servo_entry {
  // the id of the servo, 0-31
  int id;
  // the function used to update i/position
  servo_fn fn;
  // the period in ms between updating the position
  int period;
  // internal timestamp
  uint32_t ts;
  // the position indicator
  int i;
  // the delta/step for each position movement
  int delta;
};

servo_entry servos[] {
  {0,servo_fn_up_down,10,0,0,1},
  {2,servo_fn_up_down,10,0,180,-1},
  {17,servo_fn_wobble,5,00,60,1},
  {23,servo_fn_wobble,50,0,90,1}
};
const size_t servos_count = sizeof(servos)/sizeof(servo_entry);

uint32_t ts = 0;  //time stamp for changing motions
int state = 0;
void setup() {

  Serial.begin(115200);
  
  // Print to monitor
  Serial.println("PCA9685 Servo Test");

  // Initialise PCA9685s
  for(int i = 0;i<pwms_count;++i) {
    pwms[i].begin();
    pwms[i].setPWMFreq(SERVO_FREQ);
  }
  
}

void loop() {
  if(millis()>ts+10*1000) {
    ts = millis();
    switch(state) {
      case 0:
      Serial.println("First change");
      // first change
      servos[0].fn = servo_fn_wobble;
      servos[0].i = 90;
      ++state;
      break;
      case 1:
      Serial.println("Second change");
      // second change
      servos[0].fn = servo_fn_up_down;
      //servos[0].i = 0;
      ++state;
      break;
      default:
        Serial.println("Reset state");
        state = 0;
        break;
    }
  }
  for(int i = 0;i<servos_count;++i) {
    servo_entry& e = servos[i];
    if(millis()>=e.ts+e.period) {
      e.ts = millis();
      int duty = map(e.i,0,180,SERVOMIN,SERVOMAX);
      pwms[e.id/16].setPWM(e.id%16,0,duty);
      Serial.printf("id: %d, i: %d\n",e.id,e.i);
      e.fn(e.i,e.delta);
    }
  }
  //delay(1000);
}