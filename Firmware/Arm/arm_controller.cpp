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

ArmController::ArmController() : pi(-1), calibrated(false) {}

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

    // Setup servos
    setup_servo(SERVO2_PIN);  // skip SERVO1
    setup_servo(SERVO3_PIN);
    setup_servo(CLAW_PIN);

    // Setup encoder callbacks
    const int encoder_pins[] = {
        ENC_STEPPER_A, ENC_STEPPER_B,
        ENC_BASE_A, ENC_BASE_B,
        ENC_JOINT2_A, ENC_JOINT2_B,
        ENC_CLAW_A, ENC_CLAW_B
    };

    for (int pin : encoder_pins) {
        set_mode(pi, pin, PI_INPUT);
        set_pull_up_down(pi, pin, PI_PUD_UP);
        callback_ex(pi, pin, EITHER_EDGE, encoder_callback, this);
    }

    std::cout << "Arm controller initialized" << std::endl;
    return true;
}

void ArmController::setup_servo(int pin) {
    set_mode(pi, pin, PI_OUTPUT);
    set_servo_pulsewidth(pi, pin, 1500);  // Center
}

void ArmController::update_encoder(int gpio, int level, uint32_t tick) {
    static uint32_t last_tick = 0;
    if (tick - last_tick < 1000) return;
    last_tick = tick;

    if (gpio == ENC_BASE_A || gpio == ENC_BASE_B) {
        int a = gpio_read(pi, ENC_BASE_A);
        int b = gpio_read(pi, ENC_BASE_B);
        base_pos += (a == b) ? -1 : 1;
    } else if (gpio == ENC_STEPPER_A || gpio == ENC_STEPPER_B) {
        int a = gpio_read(pi, ENC_STEPPER_A);
        int b = gpio_read(pi, ENC_STEPPER_B);
        stepper_pos += (a == b) ? -1 : 1;
    } else if (gpio == ENC_JOINT2_A || gpio == ENC_JOINT2_B) {
        int a = gpio_read(pi, ENC_JOINT2_A);
        int b = gpio_read(pi, ENC_JOINT2_B);
        joint2_pos += (a == b) ? -1 : 1;
    } else if (gpio == ENC_CLAW_A || gpio == ENC_CLAW_B) {
        int a = gpio_read(pi, ENC_CLAW_A);
        int b = gpio_read(pi, ENC_CLAW_B);
        claw_pos += (a == b) ? -1 : 1;
    }
}

void ArmController::encoder_callback(int pi, unsigned int gpio, unsigned int edge, uint32_t tick, void *userdata) {
    ArmController* self = static_cast<ArmController*>(userdata);
    int level = gpio_read(pi, gpio);
    self->update_encoder(gpio, level, tick);
}

void ArmController::move_stepper(int steps, bool dir) {
    gpio_write(pi, DIR_PIN, dir);
    for (int i = 0; i < steps && running; ++i) {
        gpio_write(pi, STEP_PIN, 1);
        std::this_thread::sleep_for(std::chrono::microseconds(500));
        gpio_write(pi, STEP_PIN, 0);
        std::this_thread::sleep_for(std::chrono::microseconds(500));
        stepper_pos += dir ? 1 : -1;
    }
}

bool ArmController::calculate_angles(float x, float z,
                                     float& theta_base, float& theta_elbow) {
    // Base rotation
    theta_base = atan2f(z, x);

    // Project to plane
    float r = sqrtf(x*x + z*z);
    float dz = z;

    float dist = sqrtf(r*r + dz*dz);
    float L1 = 13.7f;
    float L2 = 10.0f;

    // Check reach
    if (dist > L1 + L2 || dist < fabs(L1 - L2)) return false;

    // Law of cosines
    float cos_theta2 = (r*r + dz*dz - L1*L1 - L2*L2) / (2 * L1 * L2);
    float theta2 = acosf(std::clamp(cos_theta2, -1.0f, 1.0f)); // elbow down
    float theta1 = atan2f(dz, r) - atan2f(L2*sinf(theta2), L1 + L2*cosf(theta2));

    // Convert to degrees and clamp
    theta_base  = std::clamp(theta_base, -90.0f * M_PI/180.0f, 90.0f * M_PI/180.0f);
    theta_elbow = std::clamp(theta1, -90.0f * M_PI/180.0f, 90.0f * M_PI/180.0f);

    return true;
}

void ArmController::move_to(float x, float z) {
    float theta_base, theta_elbow;
    if (!calculate_angles(x, z, theta_base, theta_elbow)) {
        std::cerr << "Target unreachable\n";
        return;
    }

    auto clamp_pw = [](int angle){
        int pw = 1500 + angle*10;
        return std::clamp(pw, 500, 2500);
    };

    int base_deg  = (int)(theta_base * 180.0f / M_PI);
    int elbow_deg = (int)(theta_elbow * 180.0f / M_PI);

    set_servo_pulsewidth(pi, SERVO2_PIN, clamp_pw(base_deg));
    set_servo_pulsewidth(pi, SERVO3_PIN, clamp_pw(elbow_deg));

    std::cout << "[Move] base=" << base_deg << "°, elbow=" << elbow_deg << "°\n";
}

void ArmController::emergency_stop() {
    set_servo_pulsewidth(pi, SERVO2_PIN, 0);
    set_servo_pulsewidth(pi, SERVO3_PIN, 0);
    set_servo_pulsewidth(pi, CLAW_PIN, 0);
    running = false;
    std::cout << "EMERGENCY STOP ACTIVATED" << std::endl;
}

void ArmController::calibrate() {
    stepper_pos = 0;
    base_pos = 0;
    joint2_pos = 0;
    claw_pos = 0;
    calibrated = true;
    std::cout << "Calibration complete - encoders reset" << std::endl;
}

void ArmController::test_servos() {
    const int min_pw = 1000;
    const int max_pw = 2000;

    std::cout << "Testing servos (excluding servo1)..." << std::endl;

    const int test_pins[] = {SERVO2_PIN, SERVO3_PIN, CLAW_PIN};

    for (int pin : test_pins) {
        std::cout << "Testing servo on GPIO " << pin << std::endl;

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
    int pulse = 1500 + angle * 10;
    if (pulse < 500) pulse = 500;
    if (pulse > 2500) pulse = 2500;

    int pin = -1;
    switch (servo_number) {
        case 2: pin = SERVO2_PIN; break;
        case 3: pin = SERVO3_PIN; break;
        case 4: pin = CLAW_PIN;   break;
        default:
            std::cerr << "[ERROR] Invalid servo number: " << servo_number << std::endl;
            return;
    }

    std::cout << "[Servo] Setting servo" << servo_number << " to angle " << angle
              << " → pulse width: " << pulse << "\n";

    set_servo_pulsewidth(pi, pin, pulse);
}
