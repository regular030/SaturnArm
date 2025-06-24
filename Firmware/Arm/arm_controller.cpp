#include "arm_controller.h"
#include <chrono>
#include <thread>

ArmController::ArmController() : pi(-1) {}

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
    setup_servo(SERVO1_PIN);
    setup_servo(SERVO2_PIN);
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
    set_servo_pulsewidth(pi, pin, 1500);  // Center position
}

void ArmController::update_encoder(int gpio, int level, uint32_t tick) {
    static uint32_t last_tick = 0;
    if (tick - last_tick < 1000) return;  // 1ms debounce
    last_tick = tick;
    
    if (gpio == ENC_BASE_A || gpio == ENC_BASE_B) {
        int a = gpio_read(pi, ENC_BASE_A);
        int b = gpio_read(pi, ENC_BASE_B);
        base_pos += (a == b) ? -1 : 1;
    }
    else if (gpio == ENC_STEPPER_A || gpio == ENC_STEPPER_B) {
        int a = gpio_read(pi, ENC_STEPPER_A);
        int b = gpio_read(pi, ENC_STEPPER_B);
        stepper_pos += (a == b) ? -1 : 1;
    }
    else if (gpio == ENC_JOINT2_A || gpio == ENC_JOINT2_B) {
        int a = gpio_read(pi, ENC_JOINT2_A);
        int b = gpio_read(pi, ENC_JOINT2_B);
        joint2_pos += (a == b) ? -1 : 1;
    }
    else if (gpio == ENC_CLAW_A || gpio == ENC_CLAW_B) {
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

bool ArmController::calculate_angles(float x, float y, float& theta1, float& theta2) {
    float dist = sqrtf(x * x + y * y);
    float max_reach = L1 + L2 - SAFETY_MARGIN;
    
    if (dist > max_reach) {
        std::cerr << "[ERROR] Exceeds max reach with safety margin (" 
                  << max_reach << " cm)" << std::endl;
        return false;
    }
    
    float angleA = acosf((L1*L1 + dist*dist - L2*L2) / (2 * L1 * dist));
    float angleB = atan2f(y, x);
    theta1 = angleB - angleA;
    
    float angleC = acosf((L1*L1 + L2*L2 - dist*dist) / (2 * L1 * L2));
    theta2 = M_PI - angleC;
    
    return true;
}

void ArmController::move_to(float x, float y, int z) {
    float theta1, theta2;
    if (!calculate_angles(x, y, theta1, theta2)) {
        std::cerr << "Invalid target position: x=" << x << " y=" << y << std::endl;
        return;
    }
    
    int target_base = static_cast<int>(theta1 * 180.0f / M_PI);
    int target_joint2 = static_cast<int>(theta2 * 180.0f / M_PI);
    
    // Convert z height to steps
    int target_z_steps = z * STEPS_PER_MM;
    int current_z_steps = stepper_pos;
    int delta_z = target_z_steps - current_z_steps;
    
    if (delta_z != 0) {
        move_stepper(abs(delta_z), delta_z > 0);
    }
    
    // Move servos
    set_servo_pulsewidth(pi, SERVO1_PIN, 1500 + 10 * target_base);
    set_servo_pulsewidth(pi, SERVO2_PIN, 1500 + 10 * target_base);
    set_servo_pulsewidth(pi, SERVO3_PIN, 1500 + 10 * target_joint2);
    
    std::cout << "Moved to position: x=" << x << " y=" << y << " z=" << z << std::endl;
}

void ArmController::emergency_stop() {
    set_servo_pulsewidth(pi, SERVO1_PIN, 0);
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
    std::cout << "Calibration complete - encoders reset" << std::endl;
}
