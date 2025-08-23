#include "arm_controller.h"
#include <chrono>
#include <thread>
#include <cmath>
#include <iostream>
#include <algorithm> // for std::clamp

#ifndef SHOULDER_MIN_ANGLE
#define SHOULDER_MIN_ANGLE -90
#endif
#ifndef SHOULDER_MAX_ANGLE
#define SHOULDER_MAX_ANGLE 90
#endif
#ifndef ELBOW_MIN_ANGLE
#define ELBOW_MIN_ANGLE -90
#endif
#ifndef ELBOW_MAX_ANGLE
#define ELBOW_MAX_ANGLE 90
#endif

ArmController::ArmController() : pi(-1), calibrated(false), running(true) {}

ArmController::~ArmController() {
    running = false;
    if (pi >= 0) pigpio_stop(pi);
}

bool ArmController::init() {
    pi = pigpio_start(nullptr, nullptr);
    if (pi < 0) {
        std::cerr << "Failed to initialize pigpio" << std::endl;
        return false;
    }

    // Setup GPIO pins
    set_mode(pi, DIR_PIN, PI_OUTPUT);
    set_mode(pi, STEP_PIN, PI_OUTPUT);

    // Setup servos (2 motors + claw optional)
    setup_servo(SERVO1_PIN); // base
    setup_servo(SERVO3_PIN); // elbow
    setup_servo(CLAW_PIN);   // optional claw

    // Commented out encoders for now (avoid segfaults)
    /*
    const int encoder_pins[] = {ENC_STEPPER_A, ENC_STEPPER_B,
                                ENC_BASE_A, ENC_BASE_B,
                                ENC_JOINT2_A, ENC_JOINT2_B,
                                ENC_CLAW_A, ENC_CLAW_B};
    for (int pin : encoder_pins) {
        set_mode(pi, pin, PI_INPUT);
        set_pull_up_down(pi, pin, PI_PUD_UP);
        callback_ex(pi, pin, EITHER_EDGE, encoder_callback, this);
    }
    */

    std::cout << "Arm controller initialized" << std::endl;
    return true;
}

void ArmController::setup_servo(int pin) {
    set_mode(pi, pin, PI_OUTPUT);
    set_servo_pulsewidth(pi, pin, 1500);  // Center
}

// Minimal safe IK calculation for 2-link planar arm
bool ArmController::calculate_angles(float x, float z,
                                     float& theta_base, float& theta_elbow) {
    theta_base = 0.0f;
    theta_elbow = 0.0f;

    const float L1 = 13.7f; // first arm link
    const float L2 = 10.0f; // second arm link

    float dist = sqrtf(x*x + z*z);

    // Check reach
    if (dist > (L1 + L2) || dist < fabs(L1 - L2)) {
        std::cerr << "[IK] Target out of reach" << std::endl;
        return false;
    }

    // Law of cosines for elbow angle
    float cos_elbow = (x*x + z*z - L1*L1 - L2*L2) / (2*L1*L2);
    cos_elbow = std::clamp(cos_elbow, -1.0f, 1.0f);
    theta_elbow = acosf(cos_elbow); // elbow down

    // Shoulder angle
    theta_base = atan2f(z, x) - atan2f(L2*sinf(theta_elbow), L1 + L2*cosf(theta_elbow));

    // Clamp angles to limits (in radians)
    theta_base  = std::clamp(theta_base, SHOULDER_MIN_ANGLE * (float)M_PI/180.0f,
                                         SHOULDER_MAX_ANGLE * (float)M_PI/180.0f);
    theta_elbow = std::clamp(theta_elbow, ELBOW_MIN_ANGLE * (float)M_PI/180.0f,
                                         ELBOW_MAX_ANGLE * (float)M_PI/180.0f);

    return true;
}

void ArmController::move_to(float x, float z) {
    if (pi < 0) {
        std::cerr << "[ERROR] pigpio not initialized!" << std::endl;
        return;
    }

    float theta_base = 0.0f;
    float theta_elbow = 0.0f;

    if (!calculate_angles(x, z, theta_base, theta_elbow)) {
        std::cerr << "[Move] Target unreachable\n";
        return;
    }

    // Convert to degrees
    int base_deg  = static_cast<int>(theta_base * 180.0f / M_PI);
    int elbow_deg = static_cast<int>(theta_elbow * 180.0f / M_PI);

    // Convert to pulse width and clamp
    auto clamp_pw = [](int angle){
        return std::clamp(1500 + angle*10, 500, 2500);
    };

    set_servo_pulsewidth(pi, SERVO1_PIN, clamp_pw(base_deg));  // base
    set_servo_pulsewidth(pi, SERVO3_PIN, clamp_pw(elbow_deg)); // elbow

    std::cout << "[Move] Base=" << base_deg << "°, Elbow=" << elbow_deg << "°\n";
}

// Optional emergency stop
void ArmController::emergency_stop() {
    if (pi < 0) return;
    set_servo_pulsewidth(pi, SERVO1_PIN, 0);
    set_servo_pulsewidth(pi, SERVO3_PIN, 0);
    set_servo_pulsewidth(pi, CLAW_PIN, 0);
    running = false;
    std::cout << "EMERGENCY STOP ACTIVATED" << std::endl;
}

// Simple servo test
void ArmController::test_servos() {
    if (pi < 0) return;
    const int min_pw = 1000;
    const int max_pw = 2000;

    const int test_pins[] = {SERVO1_PIN, SERVO3_PIN, CLAW_PIN};
    for (int pin : test_pins) {
        set_servo_pulsewidth(pi, pin, min_pw);
        std::this_thread::sleep_for(std::chrono::seconds(1));
        set_servo_pulsewidth(pi, pin, max_pw);
        std::this_thread::sleep_for(std::chrono::seconds(1));
        set_servo_pulsewidth(pi, pin, 1500);
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    std::cout << "Servo test complete." << std::endl;
}

void ArmController::set_servo_angle(int servo_number, int angle) {
    if (pi < 0) return;
    int pulse = std::clamp(1500 + angle * 10, 500, 2500);

    int pin = -1;
    switch (servo_number) {
        case 1: pin = SERVO1_PIN; break; // base
        case 3: pin = SERVO3_PIN; break; // elbow
        case 4: pin = CLAW_PIN;   break;
        default:
            std::cerr << "[ERROR] Invalid servo number: " << servo_number << std::endl;
            return;
    }

    set_servo_pulsewidth(pi, pin, pulse);
    std::cout << "[Servo] Servo" << servo_number << " → pulse: " << pulse << "\n";
}

void ArmController::calibrate() {
    stepper_pos = 0;
    base_pos = 0;
    joint2_pos = 0;
    claw_pos = 0;
    calibrated = true;
    std::cout << "Calibration complete - encoders reset" << std::endl;
}
