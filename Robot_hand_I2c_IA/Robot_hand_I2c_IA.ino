#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <VL53L0X.h>
#include <PDM.h>
#include <nikarts-project-1_inferencing.h>
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
VL53L0X pinch1, grab1, sizeread0, sizeread2, wrist1, wrist2, wrist3, open1;
#define SERVOMINSTOP 100 //miniumum servo angle
#define SERVOMIN  120 //grabbing angle
#define SERVOMID 300 //pinching angle
#define SERVOMID2 270 //pinching angle 2
#define SERVOMID3 280 //pinching angle 3
#define SERVOMID35 270 //grabbing angle 2
#define SERVOMID4 210 //grabbing angle 3
#define SERVOMID45 200 //grabbing angle 4
#define SERVOMAX  (400) // This is the 'maximum' pulse length count (out of 4096)
#define SERVOMAXSTOP 210 //Grabbing angle 1 for wrist in tilted position
#define SERVOMAXSTOP1 230 //Grabbing angle 2 for wrist in tilted position
#define SERVOMAXSTOP2 200 //Grabbing angle 3 for wrist in tilted position
#define SERVOMAXSTOP3 380 //Grabbing angle 4 for wrist in tilted position
#define SERVOMAXSTOP4 315 //Grabbing angle 5 for wrist in tilted position
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates
int grabstate = 0, wriststate = 0, pinchstate = 0;
int w, o, g, p, w2, w3, s0, s2;
//unsigned long duration;
//unsigned long duration1;
//unsigned long duration2;
struct servo {
  int id, minPos, midPos, midPos2, maxPos, maxPos2, maxPos3;
} servos[7];
typedef struct {
    int16_t *buffer;
    uint8_t buf_ready;
    uint32_t buf_count;
    uint32_t n_samples;
} inference_t;
static inference_t inference;
static signed short sampleBuffer[2048];
static bool debug_nn = false; 
void setup() {
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);
  Wire.begin();
  Serial.begin(9600);
  for (int pin = 2; pin <= A3; ++pin) {
        pinMode(pin, OUTPUT);
        digitalWrite(pin, LOW); // Initialize pins to LOW
   }
  initializeSensors();
  initializeServos();
  initializeInferencing();
}

void initializeServos() {
  servos[0] = {0, SERVOMINSTOP, SERVOMID, -1, SERVOMAXSTOP, -1, -1};
  servos[1] = {1, SERVOMIN, SERVOMID, SERVOMID45, SERVOMAX, SERVOMAXSTOP1, SERVOMAXSTOP3};
  servos[2] = {2, SERVOMIN, SERVOMID, SERVOMID45, SERVOMAX, SERVOMAXSTOP2, SERVOMAXSTOP4};
  servos[3] = {3, SERVOMIN, SERVOMID, SERVOMID3, SERVOMAX, -1, -1};
  servos[4] = {4, SERVOMIN, SERVOMID4, SERVOMID35, SERVOMAX, -1, -1};
  servos[5] = {5, SERVOMIN, SERVOMID4, SERVOMID4, SERVOMAX, -1, -1};
  servos[6] = {6, SERVOMINSTOP, SERVOMID, SERVOMID3, SERVOMAXSTOP, -1, -1};
}

void initializeSensors() {
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
}
void initializeInferencing(){
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
void performContinuousReadings() {
  g = grab1.readRangeContinuousMillimeters();
  s2=sizeread2.readRangeContinuousMillimeters();
  p=pinch1.readRangeContinuousMillimeters();
  o=open1.readRangeContinuousMillimeters();
  w=wrist1.readRangeContinuousMillimeters();
  w3=wrist3.readRangeContinuousMillimeters();
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
     performContinuousReadings();
     if (!grab1.timeoutOccurred()){
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
      }
     if (!grab1.timeoutOccurred()){
        if((g < 40)&&(g>1)&&(grabstate==0)&&(wriststate==1)&&(pinchstate==0)){
      servoMinUpToMax(servo1);
      servoMinUpToMax(servo2);
      servoMaxDownToMin(servo4);
      servoMaxDownToMin(servo5);
      delay(200);
      servoMaxDownToMin(servo3);
      grabstate=1;
      }
      }

      if (!pinch1.timeoutOccurred()){
      if ((p < 80)&&(p>40)&&(grabstate==0)&&(wriststate==0)&&(pinchstate==0)) {
      servoMinUpToMax2(servo2);
      servoMinUpToMax2(servo1);
      grabstate=1;
      pinchstate=1;
      }
      if ((p < 80)&&(p>1)&&(grabstate==0)&&(wriststate==1)&&(pinchstate==0)){
        servoMinUpToMax3(servo2);
        servoMinUpToMax3(servo1);
        grabstate=1;
        pinchstate=2;
      }
      
     }
     if (!open1.timeoutOccurred()){
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
      if ((o < 40)&&(o>4)&&(grabstate==1)&&(pinchstate==1)) {
      servoMaxDownToMin2(servo1);
      servoMaxDownToMin2(servo2);
      delay(3000);
      grabstate=0;
      pinchstate=0;
      }
      if ((o < 40)&&(o>4)&&(grabstate==1)&&(pinchstate==2)) {
      servoMaxDownToMin3(servo1);
      servoMaxDownToMin3(servo2);
      delay(3000);
      grabstate=0;
      pinchstate=0;
      }
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
    if (!wrist1.timeoutOccurred()){
      if ((w < 150)&&(w>110)&&(grabstate==0)&&(wriststate==0)) {
      servoMinUpToMax(servo0);
      delay(1000);
      wriststate=1;
      }
      }
     if (wriststate==1){
      if (!wrist2.timeoutOccurred()){
      if ((w3 < 180)&&(w3>3)&&(grabstate==0)&&(wriststate==1)) {
      servoMaxDownToMin(servo0);
      delay(1000);
      wriststate=0;
      }
      }
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
