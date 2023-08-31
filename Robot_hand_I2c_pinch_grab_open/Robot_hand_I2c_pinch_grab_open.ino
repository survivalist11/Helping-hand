/*************************************************** 
  This is an example for our Adafruit 16-channel PWM & Servo driver
  Servo test - this will drive 8 servos, one after the other on the
  first 8 pins of the PCA9685

  Pick one up today in the adafruit shop!
  ------> http://www.adafruit.com/products/815
  
  These drivers use I2C to communicate, 2 pins are required to  
  interface.

  Adafruit invests time and resources providing this open source code, 
  please support Adafruit and open-source hardware by purchasing 
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.  
  BSD license, all text above must be included in any redistribution
 ****************************************************/

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <VL53L0X.h>
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
VL53L0X pinch1;
VL53L0X grab1;
VL53L0X wrist1;
VL53L0X wrist2;
VL53L0X wrist3;
VL53L0X open1;
#define SERVOMINSTOP 100
#define SERVOMIN  (120) // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMID 300
#define SERVOMID4 210
#define SERVOMAX  (400) // This is the 'maximum' pulse length count (out of 4096)
#define SERVOMAXSTOP 210
#define SERVOMAXSTOP1 200
#define SERVOMAXSTOP2 250
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates
int grabstate=0;
int wriststate=0;
int w;
int o;
int g;
int p;
int w2;
int w3;
//unsigned long duration;
//unsigned long duration1;
//unsigned long duration2;
struct servo {
  int id;
   int minPos;
   int midPos;
   int maxPos;
   int maxPos2;
};
//void servoUp(struct servo servoID) {
//  Serial.println(servoID.id);
//    for (uint16_t pulselen = servoID.minPos; pulselen < servoID.maxPos; pulselen++) {
//    pwm.setPWM(servoID.id, 0, pulselen);
//    }
//    Serial.println("Done Up");
//}
//
//void servoDown(struct servo servoID) {
//  Serial.println(servoID.id);
//    for (uint16_t pulselen = servoID.maxPos; pulselen > servoID.minPos; pulselen--) {
//    pwm.setPWM(servoID.id, 0, pulselen);
//    }
//    Serial.println("Done Down");
//}
// our servo # counter
//uint8_t servonum = 0;
struct servo servo0; 
struct servo servo1; 
struct servo servo2;
struct servo servo3;
struct servo servo4;
struct servo servo5;

void setup() {
  servo0.id = 0;
  servo0.minPos = SERVOMINSTOP; 
  servo0.midPos = SERVOMID;
  servo0.maxPos = SERVOMAXSTOP;
  servo1.id = 1;
  servo1.minPos = SERVOMIN;
  servo1.midPos = SERVOMID;
  servo1.maxPos = SERVOMAX;
  servo1.maxPos2 = SERVOMAXSTOP1;
  servo2.id = 2;
  servo2.minPos = SERVOMIN;
  servo2.midPos = SERVOMID;
  servo2.maxPos = SERVOMAX;
  servo2.maxPos2 = SERVOMAXSTOP2;
  servo3.id = 3;
  servo3.minPos = SERVOMIN;
  servo3.midPos = SERVOMID;
  servo3.maxPos = SERVOMAX;
  servo4.id = 4;
  servo4.minPos = SERVOMIN;
  servo4.midPos = SERVOMID4;
  servo4.maxPos = SERVOMAX;
  servo5.id = 5;
  servo5.minPos = SERVOMIN;
  servo5.midPos = SERVOMID4;
  servo5.maxPos = SERVOMAX;
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates
  delay(10);
  pinMode(A3, OUTPUT); 
  pinMode(A2, OUTPUT);  
  pinMode(A1, OUTPUT);
  pinMode(A0, OUTPUT);
  pinMode(13, OUTPUT);
  pinMode(12, OUTPUT);
  pinMode(11, OUTPUT);
  pinMode(10, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(8, OUTPUT);
  pinMode(7, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(3, OUTPUT);
  pinMode(2, OUTPUT);
  digitalWrite(2, LOW);
  digitalWrite(3, LOW);
  digitalWrite(4, LOW);
  digitalWrite(5, LOW);
  digitalWrite(6, LOW);
  digitalWrite(7, LOW);
  digitalWrite(8, LOW);
  digitalWrite(9, LOW);
  digitalWrite(10, LOW);
  digitalWrite(11, LOW);
  digitalWrite(12, LOW);
  digitalWrite(13, LOW);
  digitalWrite(A0, LOW);
  digitalWrite(A1, LOW);
  digitalWrite(A2, LOW);
  digitalWrite(A3, LOW);
  delay(500);
  Wire.begin();
  Serial.begin(9600);
 digitalWrite(6, HIGH);
  delay(150);
  Serial.println("00");
  pinch1.init(true);

  Serial.println("01");
  delay(100);
  pinch1.setAddress((uint8_t)01);
  Serial.println("02");

  digitalWrite(4, HIGH);
    delay(150);
  grab1.init(true);
  Serial.println("03");
  delay(100);
  grab1.setAddress((uint8_t)02);
  Serial.println("04");
  
  digitalWrite(A1, HIGH);
  delay(150);
  Serial.println("10");
  wrist1.init(true);

  Serial.println("11");
  delay(100);
  wrist1.setAddress((uint8_t)03);
  Serial.println("12");
  
  digitalWrite(A0, HIGH);
  delay(150);
  Serial.println("13");
  open1.init(true);

  Serial.println("14");
  delay(100);
  open1.setAddress((uint8_t)05);
  Serial.println("15");

//  digitalWrite(A2, HIGH);
//  delay(150);
//  Serial.println("13");
//  wrist2.init(true);
//
//  Serial.println("14");
//  delay(100);
//  wrist2.setAddress((uint8_t)06);
//  Serial.println("15");
//  Serial.println("addresses set");
  digitalWrite(A3, HIGH);
  delay(150);
  Serial.println("13");
  wrist3.init(true);

  Serial.println("14");
  delay(100);
  wrist3.setAddress((uint8_t)07);
  Serial.println("15");
  Serial.println("addresses set");
  servoMinUpToMax(servo3);
  delay(100);
  servoMaxDownToMin(servo1);
  servoMaxDownToMin(servo2);
  servoMinUpToMax(servo4);  
  servoMaxDownToMin(servo0);
  servoMinUpToMax(servo5);
pinch1.startContinuous();
grab1.startContinuous();
wrist1.startContinuous();
wrist2.startContinuous();
wrist3.startContinuous();
open1.startContinuous();
}
// You can use this function if you'd like to set the pulse length in seconds
// e.g. setServoPulse(0, 0.001) is a ~1 millisecond pulse width. It's not precise!
void setServoPulse(uint8_t n, double pulse) {
  double pulselength;
  
  pulselength = 1000000;   // 1,000,000 us per second
  pulselength /= SERVO_FREQ;   // Analog servos run at ~60 Hz updates
  Serial.print(pulselength); Serial.println(" us per period"); 
  pulselength /= 4096;  // 12 bits of resolution
  Serial.print(pulselength); Serial.println(" us per bit"); 
  pulse *= 1000000;  // convert input seconds to us
  pulse /= pulselength;
  Serial.println(pulse);
  pwm.setPWM(n, 0, pulse);
}

void servoMinUpToMid(struct servo servoID) {
  Serial.println(servoID.id);
    for (uint16_t pulselen = servoID.minPos; pulselen < servoID.midPos; pulselen++) {
    pwm.setPWM(servoID.id, 0, pulselen);
    }
    Serial.println("Done MinUpToMid");
}

void servoMaxDownToMid(struct servo servoID) {
  Serial.println(servoID.id);
    for (uint16_t pulselen = servoID.maxPos; pulselen > servoID.midPos; pulselen--) {
    pwm.setPWM(servoID.id, 0, pulselen);
    }
    Serial.println("Done MaxDownToMid");
}

void servoMidUpToMax(struct servo servoID) {
  Serial.println(servoID.id);
    for (uint16_t pulselen = servoID.midPos; pulselen < servoID.maxPos; pulselen++) {
    pwm.setPWM(servoID.id, 0, pulselen);
    }
    Serial.println("Done MidUpToMax");
}

void servoMidDownToMin(struct servo servoID) {
  Serial.println(servoID.id);
    for (uint16_t pulselen = servoID.midPos; pulselen > servoID.minPos; pulselen--) {
    pwm.setPWM(servoID.id, 0, pulselen);
    }
    Serial.println("Done MidDownToMin");
}

void servoMaxDownToMin(struct servo servoID) {
  Serial.println(servoID.id);
    for (uint16_t pulselen = servoID.maxPos; pulselen > servoID.minPos; pulselen--) {
    pwm.setPWM(servoID.id, 0, pulselen);
    }
    Serial.println("Done MaxDownToMin");

}

void servoMinUpToMax(struct servo servoID) {
  Serial.println(servoID.id);
    for (uint16_t pulselen = servoID.minPos; pulselen < servoID.maxPos; pulselen++) {
    pwm.setPWM(servoID.id, 0, pulselen);
    }
    Serial.println("MinUpToMax");
}

void servoMinUpToMax2(struct servo servoID) {
  Serial.println(servoID.id);
    for (uint16_t pulselen = servoID.minPos; pulselen < servoID.maxPos2; pulselen++) {
    pwm.setPWM(servoID.id, 0, pulselen);
    }
    Serial.println("MinUpToMax");
}

void loop() {
      g=grab1.readRangeContinuousMillimeters();
     // if (!grab1.timeoutOccurred()){
      if ((g < 60)&&(g>1)&&(grabstate==0)&&(wriststate==0)) {
      servoMinUpToMid(servo2);
      servoMaxDownToMid(servo5);
      servoMaxDownToMid(servo3);
      servoMaxDownToMid(servo4);
      delay(50);
      servoMinUpToMid(servo1);
      grabstate=1;
      }
     // }
      g=grab1.readRangeContinuousMillimeters();
      Serial.print(g);
//      if (!grab1.timeoutOccurred()){
        if((g < 50)&&(g>1)&&(grabstate==0)&&(wriststate==1)){
      servoMinUpToMax(servo1);
      servoMinUpToMax(servo2);
      servoMaxDownToMin(servo4);
      servoMaxDownToMin(servo5);
      delay(50);
      servoMaxDownToMin(servo3);
      grabstate=1;
      }
    //  }
      p=pinch1.readRangeContinuousMillimeters();
      Serial.print(p);
//      if (!pinch1.timeoutOccurred()){
        if ((p < 50)&&(p>1)&&(grabstate==0)) {
      servoMinUpToMax2(servo2);
      delay(50);
      servoMinUpToMax2(servo1);
      grabstate=1;
      }
//      }
      o=open1.readRangeContinuousMillimeters();
      Serial.print(o);
 //     if (!open1.timeoutOccurred()){
      if ((o < 40)&&(o>1)) {
      servoMinUpToMax(servo3);
      delay(100);
      servoMaxDownToMin(servo1);
      servoMaxDownToMin(servo2);
      servoMinUpToMax(servo4);  
      servoMinUpToMax(servo5);
      delay(3000);
      grabstate=0;
      }
    //  }
      w=wrist1.readRangeContinuousMillimeters();
      w3=wrist3.readRangeContinuousMillimeters();
      Serial.print(w);
    //  if (!wrist1.timeoutOccurred()){
      if (((w < 80)&&(w>1)&&(grabstate==0)&&(wriststate==0))||((w3 < 80)&&(w3>1)&&(grabstate==0)&&(wriststate==0))) {
      servoMinUpToMax(servo0);
      delay(3000);
      wriststate=1;
      }
     // }
     if (wriststate==1){
      w2=wrist2.readRangeContinuousMillimeters();
      Serial.print(w2);
     // if (!wrist2.timeoutOccurred()){
      if ((w2 < 100)&&(w2>1)&&(grabstate==0)&&(wriststate==1)) {
      servoMaxDownToMin(servo0);
      wriststate=0;
      }
    //  }
     }
      else{
    }
  }
