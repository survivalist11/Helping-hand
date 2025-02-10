#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40); // Default I2C address

// Servo parameters
#define SERVO_MIN  150   // Minimum pulse length count
#define SERVO_MAX  600   // Maximum pulse length count
#define SERVO_FREQ 50    // 50 Hz frequency

// Arm link lengths (adjust according to your setup)
const float L1 = 10.0;  
const float L2 = 10.0;  

// Servo channels on PCA9685
#define SERVO_1 0  // First joint (Base)
#define SERVO_2 1  // Second joint (Elbow)

// Function to map angle (0-180 degrees) to PCA9685 pulse width
int angleToPulse(int angle) {
    return map(angle, 0, 180, SERVO_MIN, SERVO_MAX);
}

// Inverse Kinematics function
void moveTo(float x, float y) {
    float d = sqrt(x*x + y*y);
    if (d > (L1 + L2)) return; // Point is unreachable

    float theta2 = acos((x*x + y*y - L1*L1 - L2*L2) / (2 * L1 * L2));
    float theta1 = atan2(y, x) - atan2(L2 * sin(theta2), L1 + L2 * cos(theta2));

    int angle1 = int(theta1 * 180.0 / PI);  
    int angle2 = int(theta2 * 180.0 / PI);

    // Send angles to servos via PCA9685
    pwm.setPWM(SERVO_1, 0, angleToPulse(angle1 + 90));  
    pwm.setPWM(SERVO_2, 0, angleToPulse(angle2 + 90));
}

// Function to move the arm in a straight line
void moveInLine(float x0, float y0, float xf, float yf, int steps) {
    for (int i = 0; i <= steps; i++) {
        float x = x0 + (xf - x0) * i / steps;
        float y = y0 + (yf - y0) * i / steps;
        moveTo(x, y);
        delay(50);
    }
}

void setup() {
    Wire.begin();
    pwm.begin();
    pwm.setPWMFreq(SERVO_FREQ);
    delay(500);
}

void loop() {
    moveInLine(5, 5, 10, 5, 20);  // Move from (5,5) to (10,5) in a straight line
    delay(1000);
}
