#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

// Adjust Servo Pulse Range (Tweak These!)
#define SERVO_MIN  100   // Pulse width for 0°
#define SERVO_MAX  700   // Pulse width for 270°
#define SERVO_FREQ 50    

// Servo channels on PCA9685
#define SERVO_1 0  // Base rotation
#define SERVO_2 1  // Angler joint
#define SERVO_3 2  // Side-to-side motion

// Function to map angle (0-270) to PCA9685 pulse width
int angleToPulse(int angle) {
    int pulse = map(angle, 0, 270, SERVO_MIN, SERVO_MAX);
    Serial.print("Angle: "); Serial.print(angle);
    Serial.print(" -> Pulse: "); Serial.println(pulse);
    return pulse;
}

void setup() {
    Serial.begin(115200);
    Serial.println("Starting PCA9685 Servo Calibration...");

    Wire.begin();
    pwm.begin();
    pwm.setPWMFreq(SERVO_FREQ);
    delay(500);
    
    Serial.println("PCA9685 Initialized!");

    // Test 0°, 90°, 180°, and 270° to verify calibration
    pwm.setPWM(SERVO_1, 0, angleToPulse(195));
    pwm.setPWM(SERVO_2, 0, angleToPulse(93)); 
    pwm.setPWM(SERVO_3, 0, angleToPulse(93));
    
   
}

void loop() {
    // No movement in loop; servos remain in last position
}
