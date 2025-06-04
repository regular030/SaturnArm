#include <wiringPi.h>
#include <softPwm.h>
#include <iostream>
#include <sstream>
#include <cmath>
#include <unistd.h>
#include <string>
#include <atomic>
#include <thread>

// Pin Definitions
const int DIR_PIN  = 2;    // Stepper direction 
const int STEP_PIN = 3;    // Stepper step 

const int SERVO1_PIN = 15; // Joint 1 Left
const int SERVO2_PIN = 16; // Joint 1 Right
const int SERVO3_PIN = 17; // Joint 2 Elbow
const int CLAW_PIN   = 18; // SG90 Claw 

// Rotary encoder pins
const int ENC1_A = 21, ENC1_B = 22;
const int ENC2_A = 23, ENC2_B = 24;
const int ENC3_A = 25, ENC3_B = 26;
const int ENC4_A = 19, ENC4_B = 20;

// Arm dimensions (in cm)
const float L1 = 13.7;
const float L2 = 10.0;

std::atomic<bool> running(true);
std::atomic<bool> stopFlag(false);

// Globals for encoder-controlled joint angles
std::atomic<int> joint1_angle(90);
std::atomic<int> joint2_angle(90);
std::atomic<int> claw_angle(90);
std::atomic<int> z_height(0);

// Stepper Movement
void moveStepperToZ(int targetZ) {
    int currentZ = z_height.load();
    int delta = targetZ - currentZ;
    if (delta == 0) return;
    bool dir = delta > 0;
    moveStepper(std::abs(delta) * 100, dir); // Adjust 100 for resolution
    z_height = targetZ;
}

void moveStepper(int steps, bool dir) {
    digitalWrite(DIR_PIN, dir);
    for (int i = 0; i < steps && running && !stopFlag; ++i) {
        digitalWrite(STEP_PIN, HIGH);
        delayMicroseconds(500);
        digitalWrite(STEP_PIN, LOW);
        delayMicroseconds(500);
    }
}

// Servo Control
void setServo(int pin, int angle) {
    angle = std::max(0, std::min(180, angle));
    int pulse = (angle * 0.11) + 5;
    softPwmWrite(pin, pulse);
}

// Inverse Kinematics
bool calculateAngles(float x, float y, float& theta1, float& theta2) {
    float dist = sqrt(x * x + y * y);
    if (dist > L1 + L2 || dist < fabs(L1 - L2)) return false;
    if (y >= -2.0 && x >= -5.5 && x <= 5.5) return false;

    float angleA = acos((L1*L1 + dist*dist - L2*L2) / (2 * L1 * dist));
    float angleB = atan2(y, x);
    theta1 = angleB - angleA;

    float angleC = acos((L1*L1 + L2*L2 - dist*dist) / (2 * L1 * L2));
    theta2 = M_PI - angleC;

    return true;
}

int radToDeg(float rad) {
    return static_cast<int>(rad * 180.0 / M_PI);
}

// Calibration
void calibrateArm() {
    std::cout << "[CALIBRATION] Move the arm to straight-up position, then press [Enter]...\n";
    std::cin.get();
    joint1_angle = 90;
    joint2_angle = 90;
    z_height = L1 + L2;
    std::cout << "[INFO] Calibration complete.\n";
}

// Move to Coordinate
void moveTo(float x, float y, int z) {
    float t1, t2;
    if (!calculateAngles(x, y, t1, t2)) {
        std::cout << "[ERROR] Point not possible.\n";
        return;
    }

    int angle1 = radToDeg(t1);
    int angle2 = radToDeg(t2);

    if (angle1 < 0 || angle1 > 180 || angle2 < 0 || angle2 > 180) {
        std::cout << "[ERROR] Joint angle limit exceeded.\n";
        return;
    }

    std::cout << "[INFO] Moving to point...\n";

    stopFlag = true; usleep(20000); stopFlag = false; // Interrupt any motion
    moveStepperToZ(z);

    joint1_angle = angle1;
    joint2_angle = angle2;

    setServo(SERVO1_PIN, angle1);
    setServo(SERVO2_PIN, angle1);
    setServo(SERVO3_PIN, 180 - angle2);
}

// Command Parser
bool parseCoordinates(const std::string& input, float& x, float& y, int& z) {
    size_t xPos = input.find("x:");
    size_t yPos = input.find("y:");
    size_t zPos = input.find("z:");
    if (xPos == std::string::npos || yPos == std::string::npos || zPos == std::string::npos)
        return false;
    try {
        x = std::stof(input.substr(xPos + 2, yPos - (xPos + 2)));
        y = std::stof(input.substr(yPos + 2, zPos - (yPos + 2)));
        z = std::stoi(input.substr(zPos + 2));
        return true;
    } catch (...) {
        return false;
    }
}

// Encoder Thread
void encoderHandler(int pinA, int pinB, std::atomic<int>& value, int minV, int maxV) {
    int lastState = digitalRead(pinA);
    while (running) {
        int state = digitalRead(pinA);
        if (state != lastState) {
            if (digitalRead(pinB) != state) {
                value = std::min(maxV, value + 1);
            } else {
                value = std::max(minV, value - 1);
            }
        }
        lastState = state;
        delay(2);
    }
}

// MAIN
int main() {
    wiringPiSetupGpio();

    pinMode(DIR_PIN, OUTPUT);
    pinMode(STEP_PIN, OUTPUT);

    softPwmCreate(SERVO1_PIN, 0, 200);
    softPwmCreate(SERVO2_PIN, 0, 200);
    softPwmCreate(SERVO3_PIN, 0, 200);
    softPwmCreate(CLAW_PIN,   0, 200);

    pinMode(ENC1_A, INPUT); pinMode(ENC1_B, INPUT);
    pinMode(ENC2_A, INPUT); pinMode(ENC2_B, INPUT);
    pinMode(ENC3_A, INPUT); pinMode(ENC3_B, INPUT);
    pinMode(ENC4_A, INPUT); pinMode(ENC4_B, INPUT);

    std::thread enc1(encoderHandler, ENC1_A, ENC1_B, std::ref(joint1_angle), 0, 180);
    std::thread enc2(encoderHandler, ENC2_A, ENC2_B, std::ref(joint2_angle), 0, 180);
    std::thread enc3(encoderHandler, ENC3_A, ENC3_B, std::ref(claw_angle),   0, 180);
    std::thread enc4(encoderHandler, ENC4_A, ENC4_B, std::ref(z_height),    0, 30);

    std::string input;
    float x, y; int z;

    std::cout << "=== Robotic Arm Interface ===\n";
    std::cout << "Type 'calibrate' to begin calibration.\n";
    std::cout << "Type coordinates like: x:10,y:5,z:-2\n";
    std::cout << "Type 'stop' to halt all actions.\n";

    while (true) {
        std::cout << "> ";
        std::getline(std::cin, input);

        if (input == "calibrate") {
            calibrateArm();
        } else if (input == "stop") {
            stopFlag = true;
            std::cout << "[INFO] All actions stopped.\n";
        } else if (parseCoordinates(input, x, y, z)) {
            moveTo(x, y, z);
        } else {
            std::cout << "[ERROR] Invalid input. Use: x:#,y:#,z:# or 'calibrate' or 'stop'\n";
        }
    }

    running = false;
    enc1.join(); enc2.join(); enc3.join(); enc4.join();
    return 0;
}
