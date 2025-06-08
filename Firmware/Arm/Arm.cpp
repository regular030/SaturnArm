#include "Arm.h"

// Global state initialization
std::atomic<bool> running(true);
std::atomic<bool> stopFlag(false);
std::atomic<int> joint1_angle(90);
std::atomic<int> joint2_angle(90);
std::atomic<int> claw_angle(90);
std::atomic<int> z_height(0);
float MAX_REACH = L1 + L2;

// Rotary Encoder State
static volatile int32_t stepper_pos = 0;
static volatile int32_t base_pos = 0;
static volatile int32_t joint2_pos = 0;
static volatile int32_t claw_pos = 0;
static uint32_t last_encoder_time = 0;

// Helper clamp
template <typename T>
constexpr const T& clamp(const T& v, const T& lo, const T& hi) {
    return (v < lo) ? lo : (hi < v) ? hi : v;
}

void init_hardware() {
    setup_uart();
    
    // Initialize GPIOs
    gpio_init(DIR_PIN);
    gpio_set_dir(DIR_PIN, GPIO_OUT);
    gpio_init(STEP_PIN);
    gpio_set_dir(STEP_PIN, GPIO_OUT);

    // Initialize servos
    setup_servo(SERVO1_PIN);
    setup_servo(SERVO2_PIN);
    setup_servo(SERVO3_PIN);
    setup_servo(CLAW_PIN);

    // Initialize encoders
    init_encoders();
}

void setup_uart() {
    uart_init(UART_ID, BAUD_RATE);
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);
}

void setup_servo(uint pin) {
    gpio_set_function(pin, GPIO_FUNC_PWM);
    uint slice = pwm_gpio_to_slice_num(pin);
    pwm_config config = pwm_get_default_config();
    pwm_config_set_clkdiv(&config, 64.f);
    pwm_config_set_wrap(&config, PWM_WRAP);
    pwm_init(slice, &config, true);
}

void init_encoders() {
    // Configure encoder pins
    const uint encoder_pins[] = {
        ENC_STEPPER_A, ENC_STEPPER_B,
        ENC_BASE_A, ENC_BASE_B,
        ENC_JOINT2_A, ENC_JOINT2_B,
        ENC_CLAW_A, ENC_CLAW_B
    };
    
    for (uint pin : encoder_pins) {
        gpio_init(pin);
        gpio_set_dir(pin, GPIO_IN);
        gpio_pull_up(pin);
    }

    // Set up interrupts
    gpio_set_irq_enabled_with_callback(ENC_STEPPER_A, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &update_encoder);
    gpio_set_irq_enabled(ENC_STEPPER_B, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
    gpio_set_irq_enabled(ENC_BASE_A, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
    gpio_set_irq_enabled(ENC_BASE_B, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
    gpio_set_irq_enabled(ENC_JOINT2_A, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
    gpio_set_irq_enabled(ENC_JOINT2_B, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
    gpio_set_irq_enabled(ENC_CLAW_A, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
    gpio_set_irq_enabled(ENC_CLAW_B, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
}

// Update encoder tracking for continuous rotation
void update_encoder(uint gpio, uint32_t events) {
    static uint32_t last_time = 0;
    uint32_t now = time_us_32();
    if (now - last_time < 1000) return; // 1ms debounce
    last_time = now;

    // Base encoder (continuous, no wrap)
    if (gpio == ENC_BASE_A || gpio == ENC_BASE_B) {
        bool a = gpio_get(ENC_BASE_A);
        bool b = gpio_get(ENC_BASE_B);
        base_pos += (a ^ b) ? -1 : 1;
    }
    // Stepper encoder (Z axis, continuous)
    else if (gpio == ENC_STEPPER_A || gpio == ENC_STEPPER_B) {
        bool a = gpio_get(ENC_STEPPER_A);
        bool b = gpio_get(ENC_STEPPER_B);
        stepper_pos += (a ^ b) ? -1 : 1;
    }
    // Joint2 encoder (continuous)
    else if (gpio == ENC_JOINT2_A || gpio == ENC_JOINT2_B) {
        bool a = gpio_get(ENC_JOINT2_A);
        bool b = gpio_get(ENC_JOINT2_B);
        joint2_pos += (a ^ b) ? -1 : 1;
    }
    // Claw encoder (continuous)
    else if (gpio == ENC_CLAW_A || gpio == ENC_CLAW_B) {
        bool a = gpio_get(ENC_CLAW_A);
        bool b = gpio_get(ENC_CLAW_B);
        claw_pos += (a ^ b) ? -1 : 1;
    }
}

int32_t get_stepper_position() { return stepper_pos; }
int32_t get_base_position() { return base_pos; }
int32_t get_joint2_position() { return joint2_pos; }
int32_t get_claw_position() { return claw_pos; }

void set_servo(uint pin, int speed) {
    speed = clamp(speed, -100, 100);
    
    float us;
    if (speed == 0) {
        us = 1500; // Stop pulse width
    } else if (speed > 0) {
        us = 1500 + 10 * speed; // 1500-2500µs for CW
    } else {
        us = 1500 + 10 * speed; // 500-1500µs for CCW
    }
    
    uint level = (uint)(us * PWM_WRAP / 20000.0f);
    pwm_set_gpio_level(pin, level);
}

void stop_all_servos() {
    set_servo(SERVO1_PIN, 0);
    set_servo(SERVO2_PIN, 0);
    set_servo(SERVO3_PIN, 0);
}

void moveStepper(int steps, bool dir) {
    gpio_put(DIR_PIN, dir);
    for (int i = 0; i < steps && running && !stopFlag; ++i) {
        gpio_put(STEP_PIN, 1);
        sleep_us(500);
        gpio_put(STEP_PIN, 0);
        sleep_us(500);
        
        // Update encoder position based on movement
        stepper_pos += dir ? 1 : -1;
    }
}

void moveStepperToZ(int targetZ) {
    int target_pos = targetZ * STEPS_PER_MM;
    int current_pos = get_stepper_position();
    int delta = target_pos - current_pos;
    
    if (delta == 0) return;
    
    bool dir = delta > 0;
    moveStepper(abs(delta), dir);
    z_height = targetZ;
}

bool calculateAngles(float x, float y, float& theta1, float& theta2) {
    float dist = sqrtf(x * x + y * y);
    if (dist > MAX_REACH - SAFETY_MARGIN) {
        uart_send_message("[ERROR] Exceeds max reach with safety margin\n");
        return false;
    }
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

void calibrateArm() {
    uart_send_message("Calibrating - move arm to zero position manually\n");
    uart_send_message("Then send 'done' via UART\n");
    
    // Wait for user to position arm
    while (true) {
        std::string input = read_uart_line();
        if (input == "done") break;
        sleep_ms(100);
    }
    
    // Reset encoder positions
    base_pos = 0;
    joint2_pos = 0;
    stepper_pos = 0;
    claw_pos = 0;
    
    MAX_REACH = L1 + L2 - SAFETY_MARGIN;
    uart_send_message("Calibration complete\n");
}

void moveTo(float x, float y, int z) {
    float t1, t2;
    if (!calculateAngles(x, y, t1, t2)) {
        uart_send_message("[ERROR] Point unreachable\n");
        return;
    }

    int target_base = radToDeg(t1);
    int target_joint2 = radToDeg(t2);
    
    // Calculate shortest rotational path
    int base_move = (target_base - (base_pos % 360) + 540) % 360 - 180;
    int joint2_move = (target_joint2 - (joint2_pos % 360) + 540) % 360 - 180;

    // Determine direction and speed
    int base_speed = base_move > 0 ? SERVO_FULL_SPEED : -SERVO_FULL_SPEED;
    int joint2_speed = joint2_move > 0 ? SERVO_FULL_SPEED : -SERVO_FULL_SPEED;
    
    // Start movement
    moveStepperToZ(z);
    set_servo(SERVO1_PIN, base_speed);
    set_servo(SERVO2_PIN, base_speed);
    set_servo(SERVO3_PIN, -joint2_speed);

    // Fine-tune approach
    while (abs(base_move) > 5 || abs(joint2_move) > 5) {
        // Reduce speed when close
        if (abs(base_move) < 30) base_speed = base_move > 0 ? SERVO_SLOW_SPEED : -SERVO_SLOW_SPEED;
        if (abs(joint2_move) < 30) joint2_speed = joint2_move > 0 ? SERVO_SLOW_SPEED : -SERVO_SLOW_SPEED;
        
        set_servo(SERVO1_PIN, base_speed);
        set_servo(SERVO2_PIN, base_speed);
        set_servo(SERVO3_PIN, -joint2_speed);

        // Update remaining movement
        base_move = (target_base - (base_pos % 360) + 540) % 360 - 180;
        joint2_move = (target_joint2 - (joint2_pos % 360) + 540) % 360 - 180;
        
        sleep_ms(10);
    }

    stop_all_servos();
    joint1_angle = target_base;
    joint2_angle = target_joint2;
    
    char msg[50];
    snprintf(msg, sizeof(msg), "Reached: %.1f,%.1f,%d\n", x, y, z);
    uart_send_message(msg);
}

bool parseCoordinates(const std::string& input, float& x, float& y, int& z) {
    if (input.empty()) return false;
    
    size_t xPos = input.find("x:");
    size_t yPos = input.find("y:");
    size_t zPos = input.find("z:");
    if (xPos == std::string::npos || yPos == std::string::npos || zPos == std::string::npos)
        return false;

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

void uart_send_message(const char* message) {
    uart_puts(UART_ID, message);
}

int main() {
    stdio_init_all();
    init_hardware();

    uart_send_message("=== VR Robotic Arm Ready ===\n");
    uart_send_message("Send 'calibrate' to begin\n");
    uart_send_message("Send coordinates: x:val,y:val,z:val\n");
    uart_send_message("Send 'stop' to halt movement\n");

    while (true) {
        std::string input = read_uart_line();
        
        if (!input.empty()) {
            uart_send_message(("RECV: " + input + "\n").c_str());
            
            if (input == "calibrate") {
                calibrateArm();
            } 
            else if (input == "stop") {
                stop_all_servos();
                stopFlag = true;
                uart_send_message("[STOP] All movement halted\n");
            } 
            else {
                float x, y;
                int z;
                if (parseCoordinates(input, x, y, z)) {
                    moveTo(x, y, z);
                } else {
                    uart_send_message("[ERROR] Invalid command\n");
                }
            }
        }
        
        sleep_ms(10);
    }

    return 0;
}
