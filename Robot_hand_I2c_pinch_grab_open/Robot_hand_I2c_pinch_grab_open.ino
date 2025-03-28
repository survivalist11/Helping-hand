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
#include <PDM.h>
#include <nikarts-project-1_inferencing.h>
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
VL53L0X pinch1;
VL53L0X grab1;
VL53L0X sizeread0;
VL53L0X sizeread2;
VL53L0X wrist1;
VL53L0X wrist2;
VL53L0X wrist3;
VL53L0X open1;
#define SERVOMINSTOP 100
#define SERVOMIN  (120) // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMID 300
#define SERVOMID2 270
#define SERVOMID3 280 
#define SERVOMID35 270
#define SERVOMID4 210
#define SERVOMID45 200
#define SERVOMAX  (400) // This is the 'maximum' pulse length count (out of 4096)
#define SERVOMAXSTOP 210
#define SERVOMAXSTOP1 230
#define SERVOMAXSTOP2 200
#define SERVOMAXSTOP3 380
#define SERVOMAXSTOP4 315
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates
//int sensorPin = A0;
//int sensorPin1 = A1;
//int sensorPin2 = A2;// select the input pin for the potentiometer      // select the pin for the LED
int grabstate=0;
int wriststate=0;
int pinchstate=0;
int w;
int o;
int g;
int p;
int w2;
int w3;
int s0;
int s2;
//unsigned long duration;
//unsigned long duration1;
//unsigned long duration2;
struct servo {
  int id;
   int minPos;
   int midPos;
   int midPos2;
   int maxPos;
   int maxPos2;
   int maxPos3;
};
typedef struct {
    int16_t *buffer;
    uint8_t buf_ready;
    uint32_t buf_count;
    uint32_t n_samples;
} inference_t;
static inference_t inference;
static signed short sampleBuffer[2048];
static bool debug_nn = false; // Set this to true to see e.g. features generated from the raw signal
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
struct servo servo6;
void setup() {
  servo0.id = 0;
  servo0.minPos = SERVOMINSTOP; 
  servo0.midPos = SERVOMID;
  servo0.maxPos = SERVOMAXSTOP;
  servo1.id = 1;
  servo1.minPos = SERVOMIN;
  servo1.midPos = SERVOMID;
  servo1.midPos2 = SERVOMID45;
  servo1.maxPos = SERVOMAX;
  servo1.maxPos2 = SERVOMAXSTOP1;
  servo1.maxPos3 = SERVOMAXSTOP3;
  servo2.id = 2;
  servo2.minPos = SERVOMIN;
  servo2.midPos = SERVOMID;
  servo2.midPos2 = SERVOMID45;
  servo2.maxPos = SERVOMAX;
  servo2.maxPos2 = SERVOMAXSTOP2;
  servo2.maxPos3 = SERVOMAXSTOP4;
  servo3.id = 3;
  servo3.minPos = SERVOMIN;
  servo3.midPos = SERVOMID;
  servo3.midPos2 = SERVOMID3;
  servo3.maxPos = SERVOMAX;
  servo4.id = 4;
  servo4.minPos = SERVOMIN;
  servo4.midPos = SERVOMID4;
  servo4.midPos2 = SERVOMID35;
  servo4.maxPos = SERVOMAX;
  servo5.id = 5;
  servo5.minPos = SERVOMIN;
  servo5.midPos = SERVOMID4;
  servo5.midPos2 = SERVOMID4;
  servo5.maxPos = SERVOMAX;
  servo6.id = 6;
  servo6.minPos = SERVOMINSTOP;
  servo6.midPos = SERVOMID;
  servo6.midPos2 = SERVOMID3;
  servo6.maxPos = SERVOMAXSTOP;
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
  open1.setAddress((uint8_t)04);
  Serial.println("15");
  digitalWrite(A2, HIGH);
  delay(150);
  Serial.println("13");
  wrist2.init(true);
  Serial.println("14");
  delay(100);
  wrist2.setAddress((uint8_t)05);
  Serial.println("15");
  Serial.println("addresses set");
  digitalWrite(A3, HIGH);
  delay(150);
  Serial.println("13");
  wrist3.init(true);
  Serial.println("14");
  delay(100);
  wrist3.setAddress((uint8_t)06);
  Serial.println("15");
  Serial.println("addresses set");

  digitalWrite(10, HIGH);
  delay(150);
  Serial.println("13");
  sizeread2.init(true);
  Serial.println("14");
  delay(100);
  sizeread2.setAddress((uint8_t)07);
  Serial.println("15");
  Serial.println("addresses set");

//  digitalWrite(10, HIGH);
//  delay(150);
//  Serial.println("13");
//  sizeread2.init(true);
//  Serial.println("14");
//  delay(100);
//  sizeread2.setAddress((uint8_t)08);
//  Serial.println("15");
//  Serial.println("addresses set");
  
  servoMinUpToMax(servo3);
  delay(100);
  servoMaxDownToMin(servo1);
  servoMaxDownToMin(servo2);
  servoMinUpToMax(servo4);  
  servoMaxDownToMin(servo0);
  servoMaxDownToMin(servo6);
  servoMinUpToMax(servo5);
pinch1.startContinuous();
grab1.startContinuous();
wrist1.startContinuous();
wrist3.startContinuous();
open1.startContinuous();
sizeread0.startContinuous();
sizeread2.startContinuous();
    Serial.println("Edge Impulse Inferencing Demo");
    // summary of inferencing settings (from model_metadata.h)
    ei_printf("Inferencing settings:\n");
    ei_printf("\tInterval: %.2f ms.\n", (float)EI_CLASSIFIER_INTERVAL_MS);
    ei_printf("\tFrame size: %d\n", EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE);
    ei_printf("\tSample length: %d ms.\n", EI_CLASSIFIER_RAW_SAMPLE_COUNT / 16);
    ei_printf("\tNo. of classes: %d\n", sizeof(ei_classifier_inferencing_categories) / sizeof(ei_classifier_inferencing_categories[0]));
    if (microphone_inference_start(EI_CLASSIFIER_RAW_SAMPLE_COUNT) == false) {
        ei_printf("ERR: Could not allocate audio buffer (size %d), this could be due to the window length of your model\r\n", EI_CLASSIFIER_RAW_SAMPLE_COUNT);
        return;
    }
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
void servoMinUpToMid2(struct servo servoID) {
  Serial.println(servoID.id);
    for (uint16_t pulselen = servoID.minPos; pulselen < servoID.midPos2; pulselen++) {
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
void servoMaxDownToMid2(struct servo servoID) {
  Serial.println(servoID.id);
    for (uint16_t pulselen = servoID.maxPos; pulselen > servoID.midPos2; pulselen--) {
    pwm.setPWM(servoID.id, 0, pulselen);
    }
    Serial.println("Done MaxDownToMid2");
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
void servoMinUpToMax3(struct servo servoID) {
  Serial.println(servoID.id);
    for (uint16_t pulselen = servoID.minPos; pulselen < servoID.maxPos3; pulselen++) {
    pwm.setPWM(servoID.id, 0, pulselen);
    }
    Serial.println("MinUpToMax");
}
void servoMaxDownToMin2(struct servo servoID) {
  Serial.println(servoID.id);
    for (uint16_t pulselen = servoID.maxPos2; pulselen > servoID.minPos; pulselen--) {
    pwm.setPWM(servoID.id, 0, pulselen);
    }
    Serial.println("MinUpToMax");
}
void servoMaxDownToMin3(struct servo servoID) {
  Serial.println(servoID.id);
    for (uint16_t pulselen = servoID.maxPos3; pulselen > servoID.minPos; pulselen--) {
    pwm.setPWM(servoID.id, 0, pulselen);
    }
    Serial.println("MinUpToMax");
}
void loop() {
      g=grab1.readRangeContinuousMillimeters();
      s2=sizeread2.readRangeContinuousMillimeters();
      Serial.println(s2);
     // if (!grab1.timeoutOccurred()){
      if ((g < 40)&&(g>1)&&(grabstate==0)&&(wriststate==0)&&(s2>50)&&(s2>1)) {
      servoMaxDownToMid(servo5);
      servoMaxDownToMid(servo3);
      servoMaxDownToMid(servo4);
      delay(200);
      servoMinUpToMid(servo1);
      servoMinUpToMid(servo2);
      grabstate=1;
      }
      if ((g < 40)&&(g>1)&&(grabstate==0)&&(wriststate==0)&&(s2<50)&&(s2>1)) {
      servoMaxDownToMid2(servo5);
      servoMaxDownToMid2(servo3);
      servoMaxDownToMid2(servo4);
      delay(200);
      servoMinUpToMid2(servo2);
      servoMinUpToMid2(servo1);
      grabstate=1;
      }
     // }
      g=grab1.readRangeContinuousMillimeters();
//      if (!grab1.timeoutOccurred()){
        if((g < 40)&&(g>1)&&(grabstate==0)&&(wriststate==1)&&(pinchstate==0)){
      servoMinUpToMax(servo1);
      servoMinUpToMax(servo2);
      servoMaxDownToMin(servo4);
      servoMaxDownToMin(servo5);
      delay(200);
      servoMaxDownToMin(servo3);
      grabstate=1;
      }
    //  }
      p=pinch1.readRangeContinuousMillimeters();
//      if (!pinch1.timeoutOccurred()){
        if ((p < 80)&&(p>40)&&(grabstate==0)&&(wriststate==0)&&(pinchstate==0)) {
      servoMinUpToMax2(servo2);
      servoMinUpToMax2(servo1);
      grabstate=1;
      pinchstate=1;
      }
      p=pinch1.readRangeContinuousMillimeters();
      if ((p < 80)&&(p>1)&&(grabstate==0)&&(wriststate==1)&&(pinchstate==0)){
        servoMinUpToMax3(servo2);
        servoMinUpToMax3(servo1);
        grabstate=1;
        pinchstate=2;
      }
      
//      }
      o=open1.readRangeContinuousMillimeters();
 //     if (!open1.timeoutOccurred()){
      if ((o < 40)&&(o>4)&&(grabstate==1)&&(pinchstate==0)) {
      servoMinUpToMax(servo4);
      delay(100);
      servoMinUpToMax(servo3);
      delay(100);
      servoMaxDownToMin(servo1);
      delay(100);
      servoMinUpToMax(servo5);
      delay(200);  
      servoMaxDownToMin(servo2);
      delay(3000);
      grabstate=0;
      }
      o=open1.readRangeContinuousMillimeters();
      if ((o < 40)&&(o>4)&&(grabstate==1)&&(pinchstate==1)) {
      servoMaxDownToMin2(servo1);
      servoMaxDownToMin2(servo2);
      delay(3000);
      grabstate=0;
      pinchstate=0;
      }
       o=open1.readRangeContinuousMillimeters();
      if ((o < 40)&&(o>4)&&(grabstate==1)&&(pinchstate==2)) {
      servoMaxDownToMin3(servo1);
      servoMaxDownToMin3(servo2);
      delay(3000);
      grabstate=0;
      pinchstate=0;
      }
      o=open1.readRangeContinuousMillimeters();
      if ((o < 40)&&(o>4)&&(grabstate==0)&&(pinchstate==0)) {
       ei_printf("Starting inferencing in 2 seconds...\n");
  ei_printf("Recording...\n");
  bool m = microphone_inference_record();
  if (!m) {
    ei_printf("ERR: Failed to record audio...\n");
    return;
  }
  ei_printf("Recording done\n");
  signal_t signal;
  signal.total_length = EI_CLASSIFIER_RAW_SAMPLE_COUNT;
  signal.get_data = &microphone_audio_signal_get_data;
  ei_impulse_result_t result = {0};
  EI_IMPULSE_ERROR r = run_classifier(&signal, &result, debug_nn);
  if (r != EI_IMPULSE_OK) {
    ei_printf("ERR: Failed to run classifier (%d)\n", r);
    return;
  }
  ei_printf("Predictions ");
  ei_printf("(DSP: %d ms., Classification: %d ms., Anomaly: %d ms.)",
    result.timing.dsp, result.timing.classification, result.timing.anomaly);
  ei_printf(": \n");
  for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
    ei_printf("    %s: %.5f\n", result.classification[ix].label, result.classification[ix].value);
  }
  if (result.classification[0].value>0.80){
    servoMinUpToMax(servo6);
  }
  if (result.classification[3].value>0.80){
    servoMaxDownToMin(servo6);
  }
#if EI_CLASSIFIER_HAS_ANOMALY == 1
  ei_printf("    anomaly score: %.3f\n", result.anomaly);
#endif
}
    //  }
      w=wrist1.readRangeContinuousMillimeters();
    //  if (!wrist1.timeoutOccurred()){
      if ((w < 150)&&(w>110)&&(grabstate==0)&&(wriststate==0)) {
      servoMinUpToMax(servo0);
      delay(1000);
      wriststate=1;
      }
     // }
     if (wriststate==1){
      w3=wrist3.readRangeContinuousMillimeters();
     // if (!wrist2.timeoutOccurred()){
      if ((w3 < 180)&&(w3>3)&&(grabstate==0)&&(wriststate==1)) {
      servoMaxDownToMin(servo0);
      delay(1000);
      wriststate=0;
      }
    //  }
     }
      else{
      }
  }
  static void pdm_data_ready_inference_callback(void)
{
    int bytesAvailable = PDM.available();
    // read into the sample buffer
    int bytesRead = PDM.read((char *)&sampleBuffer[0], bytesAvailable);
    if (inference.buf_ready == 0) {
        for(int i = 0; i < bytesRead>>1; i++) {
            inference.buffer[inference.buf_count++] = sampleBuffer[i];
            if(inference.buf_count >= inference.n_samples) {
                inference.buf_count = 0;
                inference.buf_ready = 1;
                break;
            }
        }
    }
}
/**
 * @brief      Init inferencing struct and setup/start PDM
 *
 * @param[in]  n_samples  The n samples
 *
 * @return     { description_of_the_return_value }
 */
static bool microphone_inference_start(uint32_t n_samples)
{
    inference.buffer = (int16_t *)malloc(n_samples * sizeof(int16_t));
    if(inference.buffer == NULL) {
        return false;
    }
    inference.buf_count  = 0;
    inference.n_samples  = n_samples;
    inference.buf_ready  = 0;
    // configure the data receive callback
    PDM.onReceive(&pdm_data_ready_inference_callback);
    PDM.setBufferSize(4096);
    // initialize PDM with:
    // - one channel (mono mode)
    // - a 16 kHz sample rate
    if (!PDM.begin(1, EI_CLASSIFIER_FREQUENCY)) {
        ei_printf("Failed to start PDM!");
        microphone_inference_end();
        return false;
    }
    // set the gain, defaults to 20
    PDM.setGain(127);
    return true;
}
/**
 * @brief      Wait on new data
 *
 * @return     True when finished
 */
static bool microphone_inference_record(void)
{
    inference.buf_ready = 0;
    inference.buf_count = 0;
    while(inference.buf_ready == 0) {
        delay(10);
    }
    return true;
}
/**
 * Get raw audio signal data
 */
static int microphone_audio_signal_get_data(size_t offset, size_t length, float *out_ptr)
{
    numpy::int16_to_float(&inference.buffer[offset], out_ptr, length);
    return 0;
}
/**
 * @brief      Stop PDM and release buffers
 */
static void microphone_inference_end(void)
{
    PDM.end();
    free(inference.buffer);
}
