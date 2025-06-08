#ifndef ARM_CONTROL_H
#define ARM_CONTROL_H

#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "hardware/uart.h"
#include <atomic>
#include <string>
#include <cmath>

// Pin Definitions
constexpr uint DIR_PIN = 2;
constexpr uint STEP_PIN = 3;
constexpr uint SERVO1_PIN = 15;
constexpr uint SERVO2_PIN = 16;
constexpr uint SERVO3_PIN = 17;
constexpr uint CLAW_PIN = 18;

// Rotary Encoder Pins
constexpr uint ENC_STEPPER_A = 4;
constexpr uint ENC_STEPPER_B = 5;
constexpr uint ENC_BASE_A = 6;
constexpr uint ENC_BASE_B = 7;
constexpr uint ENC_JOINT2_A = 8;
constexpr uint ENC_JOINT2_B = 9;
constexpr uint ENC_CLAW_A = 10;
constexpr uint ENC_CLAW_B = 11;

// UART Configuration
#define UART_ID uart0
#define BAUD_RATE 115200
#define UART_TX_PIN 0
#define UART_RX_PIN 1

// Arm dimensions (cm)
constexpr float L1 = 13.7f;
constexpr float L2 = 10.0f;
constexpr float SAFETY_MARGIN = 1.0f;
constexpr uint STEPS_PER_MM = 100; // Steps per millimeter for stepper

// Global state
extern std::atomic<bool> running;
extern std::atomic<bool> stopFlag;
extern std::atomic<int> joint1_angle;
extern std::atomic<int> joint2_angle;
extern std::atomic<int> claw_angle;
extern std::atomic<int> z_height;
extern float MAX_REACH;
extern constexpr uint PWM_WRAP = 20000;

// Helper functions
template <typename T>
constexpr const T& clamp(const T& v, const T& lo, const T& hi);

// Hardware initialization
void init_hardware();
void setup_uart();
void setup_servo(uint pin);
void init_encoders();

// Servo control
void set_servo(uint pin, int angle);

// Stepper control
void moveStepper(int steps, bool dir);
void moveStepperToZ(int targetZ);

// Kinematics
bool calculateAngles(float x, float y, float& theta1, float& theta2);
int radToDeg(float rad);

// Arm control
void calibrateArm();
void moveTo(float x, float y, int z);
void emergency_stop();

// Rotary Encoders
void update_encoder(uint gpio, uint32_t events);
int32_t get_stepper_position();
int32_t get_base_position();
int32_t get_joint2_position();
int32_t get_claw_position();

// Communication
bool parseCoordinates(const std::string& input, float& x, float& y, int& z);
std::string read_uart_line();
void uart_send_message(const char* message);

#endif // ARM_CONTROL_H
