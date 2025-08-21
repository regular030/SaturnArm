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

bool ArmController::calculate_angles(float x, float z, float& theta1, float& theta2) {
    float dz = z - 22.0f;
    float dist = sqrtf(x * x + dz * dz);
    float max_reach = L1 + L2 - SAFETY_MARGIN;

    std::cout << "[IK] Distance to target: " << dist << " cm, Max reach: " << max_reach << " cm\n";

    if (dist > max_reach) {
        std::cerr << "[ERROR] Exceeds max reach with safety margin (" << max_reach << " cm)\n";
        return false;
    }

    for (int attempt = 0; attempt < 2; ++attempt) {
        bool elbow_down = (attempt == 0);
        float angleA = acosf((L1*L1 + dist*dist - L2*L2) / (2 * L1 * dist));
        float angleB = atan2f(dz, x);
        float t1 = elbow_down ? (angleB - angleA) : (angleB + angleA);
        float angleC = acosf((L1*L1 + L2*L2 - dist*dist) / (2 * L1 * L2));
        float t2 = elbow_down ? (M_PI - angleC) : (angleC - M_PI);

        float deg1 = t1 * 180.0f / static_cast<float>(M_PI);
        float deg2 = t2 * 180.0f / static_cast<float>(M_PI);

        if (deg1 >= SHOULDER_MIN_ANGLE && deg1 <= SHOULDER_MAX_ANGLE &&
            deg2 >= ELBOW_MIN_ANGLE && deg2 <= ELBOW_MAX_ANGLE) {
            theta1 = std::clamp(t1, static_cast<float>(SHOULDER_MIN_ANGLE) * static_cast<float>(M_PI) / 180.0f, static_cast<float>(SHOULDER_MAX_ANGLE) * static_cast<float>(M_PI) / 180.0f);
            theta2 = std::clamp(t2, static_cast<float>(ELBOW_MIN_ANGLE) * static_cast<float>(M_PI) / 180.0f, static_cast<float>(ELBOW_MAX_ANGLE) * static_cast<float>(M_PI) / 180.0f);
            std::cout << "[IK] Elbow " << (elbow_down ? "down" : "up")
                      << " → θ1=" << deg1 << "°, θ2=" << deg2 << "°\n";
            return true;
        }
    }

    std::cerr << "[WARN] No valid IK solution\n";
    return false;
}

void ArmController::move_to(float x, float z, int /*unused_y*/) {
    float theta1, theta2;
    if (!calculate_angles(x, z, theta1, theta2)) {
        std::cerr << "Invalid target position: x=" << x << " z=" << z << std::endl;
        return;
    }

    int target_shoulder = std::clamp(static_cast<int>(theta1 * 180.0f / M_PI), SHOULDER_MIN_ANGLE, SHOULDER_MAX_ANGLE);
    int target_elbow = std::clamp(static_cast<int>(theta2 * 180.0f / M_PI), ELBOW_MIN_ANGLE, ELBOW_MAX_ANGLE);

    // Clamp angles
    if (target_shoulder > 90) target_shoulder = 90;
    if (target_shoulder < -90) target_shoulder = -90;
    if (target_elbow > 90) target_elbow = 90;
    if (target_elbow < -90) target_elbow = -90;

    std::cout << "[Move] Target angles: shoulder=" << target_shoulder
              << "°, elbow=" << target_elbow << "°\n";

    auto clamp_pw = [](int angle) {
        int pw = 1500 + angle * 10;
        if (pw < 500) pw = 500;
        if (pw > 2500) pw = 2500;
        return pw;
    };

    int shoulder_pw = clamp_pw(target_shoulder);
    int elbow_pw = clamp_pw(target_elbow);

    set_servo_pulsewidth(pi, SERVO2_PIN, shoulder_pw); // shoulder
    set_servo_pulsewidth(pi, SERVO3_PIN, elbow_pw);    // elbow

    std::cout << "Moved to position: x=" << x << " z=" << z << std::endl;
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
