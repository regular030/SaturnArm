#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "hardware/uart.h"

#include <cmath>
#include <string>
#include <atomic>

// Pin Definitions
const uint DIR_PIN  = 2;
const uint STEP_PIN = 3;

const uint SERVO1_PIN = 15;
const uint SERVO2_PIN = 16;
const uint SERVO3_PIN = 17;
const uint CLAW_PIN   = 18;

// UART Configuration
#define UART_ID uart0
#define BAUD_RATE 115200
#define UART_TX_PIN 0
#define UART_RX_PIN 1

// Arm dimensions (cm)
const float L1 = 13.7f;
const float L2 = 10.0f;
float MAX_REACH = L1 + L2;
const float SAFETY_MARGIN = 1.0f;  // 1cm headroom

// Global state
std::atomic<bool> running(true);
std::atomic<bool> stopFlag(false);
std::atomic<int> joint1_angle(90);
std::atomic<int> joint2_angle(90);
std::atomic<int> claw_angle(90);
std::atomic<int> z_height(0);

constexpr uint PWM_WRAP = 20000;

// Helper clamp
template <typename T>
constexpr const T& clamp(const T& v, const T& lo, const T& hi) {
    return (v < lo) ? lo : (hi < v) ? hi : v;
}

// Setup UART
void setup_uart() {
    uart_init(UART_ID, BAUD_RATE);
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);
}

// Setup PWM for servo
void setup_servo(uint pin) {
    gpio_set_function(pin, GPIO_FUNC_PWM);
    uint slice = pwm_gpio_to_slice_num(pin);
    pwm_config config = pwm_get_default_config();
    pwm_config_set_clkdiv(&config, 64.f);
    pwm_config_set_wrap(&config, PWM_WRAP);
    pwm_init(slice, &config, true);
}

// Set servo angle (0-180)
void set_servo(uint pin, int angle) {
    angle = clamp(angle, 0, 180);
    float min_us = 1000.0f;
    float max_us = 2000.0f;
    float us = min_us + ((angle / 180.0f) * (max_us - min_us));
    uint level = (uint)(us * PWM_WRAP / 20000.0f);
    uint slice = pwm_gpio_to_slice_num(pin);
    uint channel = pwm_gpio_to_channel(pin);
    pwm_set_chan_level(slice, channel, level);
}

// Move stepper
void moveStepper(int steps, bool dir) {
    gpio_put(DIR_PIN, dir);
    for (int i = 0; i < steps && running && !stopFlag; ++i) {
        gpio_put(STEP_PIN, 1);
        sleep_us(500);
        gpio_put(STEP_PIN, 0);
        sleep_us(500);
    }
}

// Move stepper to target Z
void moveStepperToZ(int targetZ) {
    int currentZ = z_height.load();
    int delta = targetZ - currentZ;
    if (delta == 0) return;
    bool dir = delta > 0;
    moveStepper(abs(delta) * 100, dir);
    z_height = targetZ;
}

// Inverse kinematics
bool calculateAngles(float x, float y, float& theta1, float& theta2) {
    // Apply safety boundaries
    float dist = sqrtf(x * x + y * y);
    if (dist > MAX_REACH - SAFETY_MARGIN) {
        uart_puts(UART_ID, "[ERROR] Exceeds max reach with safety margin\n");
        return false;
    }
    
    // Calculate angles
    if (dist > L1 + L2 || dist < fabsf(L1 - L2)) return false;

    float angleA = acosf((L1*L1 + dist*dist - L2*L2) / (2 * L1 * dist));
    float angleB = atan2f(y, x);
    theta1 = angleB - angleA;

    float angleC = acosf((L1*L1 + L2*L2 - dist*dist) / (2 * L1 * L2));
    theta2 = M_PI - angleC;

    return true;
}

int radToDeg(float rad) {
    return static_cast<int>(rad * 180.0f / M_PI);
}

// Calibration sequence
void calibrateArm() {
    uart_puts(UART_ID, "[CALIBRATION] Starting...\n");
    
    // Reset to default position
    joint1_angle = 90;
    joint2_angle = 90;
    z_height = static_cast<int>(L1 + L2);
    
    // Move to physical position
    set_servo(SERVO1_PIN, 90);
    set_servo(SERVO2_PIN, 90);
    set_servo(SERVO3_PIN, 90);
    moveStepperToZ(z_height);
    
    // Calculate new max reach with safety margin
    MAX_REACH = L1 + L2 - SAFETY_MARGIN;
    
    // Send calibration data to Pi Zero
    char cal_msg[50];
    snprintf(cal_msg, sizeof(cal_msg), "CAL_DONE:%.2f\n", MAX_REACH);
    uart_puts(UART_ID, cal_msg);
}

// Move to coordinates
void moveTo(float x, float y, int z) {
    float t1, t2;
    if (!calculateAngles(x, y, t1, t2)) {
        uart_puts(UART_ID, "[ERROR] Point unreachable\n");
        return;
    }

    int angle1 = radToDeg(t1);
    int angle2 = radToDeg(t2);

    if (angle1 < 0 || angle1 > 180 || angle2 < 0 || angle2 > 180) {
        uart_puts(UART_ID, "[ERROR] Joint angle limit exceeded\n");
        return;
    }

    // Send confirmation to VR system
    char pos_msg[50];
    snprintf(pos_msg, sizeof(pos_msg), "MOVING:%.2f,%.2f,%d\n", x, y, z);
    uart_puts(UART_ID, pos_msg);

    stopFlag = true; 
    sleep_ms(20); 
    stopFlag = false;
    moveStepperToZ(z);

    joint1_angle = angle1;
    joint2_angle = angle2;

    set_servo(SERVO1_PIN, angle1);
    set_servo(SERVO2_PIN, angle1);
    set_servo(SERVO3_PIN, 180 - angle2);
}

// Parse coordinates from VR system
bool parseCoordinates(const std::string& input, float& x, float& y, int& z) {
    if (input.empty()) return false;
    
    // Expected format: "x:10.5,y:-3.2,z:15"
    size_t xPos = input.find("x:");
    size_t yPos = input.find("y:");
    size_t zPos = input.find("z:");
    if (xPos == std::string::npos || yPos == std::string::npos || zPos == std::string::npos)
        return false;

    // Manual parsing without exceptions
    std::string xStr = input.substr(xPos + 2, yPos - (xPos + 2));
    std::string yStr = input.substr(yPos + 2, zPos - (yPos + 2));
    std::string zStr = input.substr(zPos + 2);

    char* endptr = nullptr;
    x = strtof(xStr.c_str(), &endptr);
    if (endptr == xStr.c_str() || *endptr != '\0') return false;

    y = strtof(yStr.c_str(), &endptr);
    if (endptr == yStr.c_str() || *endptr != '\0') return false;

    z = strtol(zStr.c_str(), &endptr, 10);
    if (endptr == zStr.c_str() || *endptr != '\0') return false;

    return true;
}

// Read UART input with timeout
std::string read_uart_line() {
    std::string input;
    absolute_time_t timeout = make_timeout_time_ms(100);
    
    while (true) {
        if (uart_is_readable(UART_ID)) {
            char c = uart_getc(UART_ID);
            if (c == '\n') break;
            input += c;
        }
        if (time_reached(timeout)) break;
        sleep_ms(1);
    }
    return input;
}

int main() {
    stdio_init_all();
    setup_uart();

    // Initialize GPIOs
    gpio_init(DIR_PIN); 
    gpio_set_dir(DIR_PIN, GPIO_OUT);
    gpio_init(STEP_PIN); 
    gpio_set_dir(STEP_PIN, GPIO_OUT);

    // Setup servos
    setup_servo(SERVO1_PIN);
    setup_servo(SERVO2_PIN);
    setup_servo(SERVO3_PIN);
    setup_servo(CLAW_PIN);

    // Initial positions
    set_servo(SERVO1_PIN, 90);
    set_servo(SERVO2_PIN, 90);
    set_servo(SERVO3_PIN, 90);
    set_servo(CLAW_PIN, 90);
    moveStepperToZ(0);

    uart_puts(UART_ID, "=== VR Robotic Arm Ready ===\n");
    uart_puts(UART_ID, "Send 'calibrate' to begin\n");
    uart_puts(UART_ID, "Send coordinates: x:val,y:val,z:val\n");
    uart_puts(UART_ID, "Send 'stop' to halt movement\n");

    while (true) {
        std::string input = read_uart_line();
        
        if (!input.empty()) {
            // Echo command for debugging
            uart_puts(UART_ID, ("RECV: " + input + "\n").c_str());
            
            if (input == "calibrate") {
                calibrateArm();
            } else if (input == "stop") {
                stopFlag = true;
                uart_puts(UART_ID, "[STOP] All movement halted\n");
            } else {
                float x, y;
                int z;
                if (parseCoordinates(input, x, y, z)) {
                    moveTo(x, y, z);
                } else {
                    uart_puts(UART_ID, "[ERROR] Invalid command\n");
                }
            }
        }
        
        sleep_ms(10);  // Reduce CPU usage
    }

    return 0;
}
