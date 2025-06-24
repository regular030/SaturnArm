#pragma once
#include <pigpiod_if2.h>
#include <atomic>
#include <thread>
#include <cmath>
#include <iostream>

class ArmController {
public:
    ArmController();
    ~ArmController();
    
    bool init();
    void move_to(float x, float y, int z);
    void emergency_stop();
    void calibrate();
    
    // Encoder positions
    int get_stepper_position() const { return stepper_pos; }
    int get_base_position() const { return base_pos; }
    int get_joint2_position() const { return joint2_pos; }
    int get_claw_position() const { return claw_pos; }
    
private:
    int pi;
    std::atomic<int> stepper_pos{0};
    std::atomic<int> base_pos{0};
    std::atomic<int> joint2_pos{0};
    std::atomic<int> claw_pos{0};
    std::atomic<bool> running{true};
    
    // Arm dimensions (cm)
    const float L1 = 13.7f;
    const float L2 = 10.0f;
    const float SAFETY_MARGIN = 1.0f;
    const uint STEPS_PER_MM = 100;
    
    // Pin Definitions (Broadcom numbering)
    const int DIR_PIN = 17;
    const int STEP_PIN = 27;
    const int SERVO1_PIN = 22;
    const int SERVO2_PIN = 23;
    const int SERVO3_PIN = 24;
    const int CLAW_PIN = 25;
    
    // Encoder Pins
    const int ENC_STEPPER_A = 5;
    const int ENC_STEPPER_B = 6;
    const int ENC_BASE_A = 12;
    const int ENC_BASE_B = 13;
    const int ENC_JOINT2_A = 16;
    const int ENC_JOINT2_B = 26;
    const int ENC_CLAW_A = 20;
    const int ENC_CLAW_B = 21;
    
    void setup_servo(int pin);
    void move_stepper(int steps, bool dir);
    void update_encoder(int gpio, int level, uint32_t tick);
    static void encoder_callback(int pi, unsigned int gpio, unsigned int edge, uint32_t tick, void *userdata);
    bool calculate_angles(float x, float y, float& theta1, float& theta2);
};
