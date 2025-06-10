#include "pico/cyw43_arch.h"
#include "lwip/tcp.h"
#include "Arm.h"
#include "wifi.h"
#include "ov2640.h"
#include <string.h>
#include <stdlib.h>
#include "pico/multicore.h"

#define TCP_PORT 80
#define BUF_SIZE 2048
#define WS_KEY_LENGTH 24
#define CAMERA_WIDTH 640
#define CAMERA_HEIGHT 480
#define CAMERA_FPS 10

typedef struct {
    struct tcp_pcb *pcb;
    char buffer[BUF_SIZE];
    int request_len;
    bool is_websocket;
    uint8_t ws_frame[128];
} tcp_server_state_t;

static tcp_server_state_t *active_ws_client = NULL;
static uint8_t camera_buffer[CAMERA_WIDTH * CAMERA_HEIGHT * 2];
static bool camera_initialized = false;

const char *ws_response_header = 
    "HTTP/1.1 101 Switching Protocols\r\n"
    "Upgrade: websocket\r\n"
    "Connection: Upgrade\r\n"
    "Sec-WebSocket-Accept: ";

const char *ws_magic_string = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11";

bool init_camera() {
    if (!ov2640_init()) {
        printf("Failed to initialize camera\n");
        return false;
    }
    ov2640_set_resolution(CAMERA_WIDTH, CAMERA_HEIGHT);
    ov2640_set_format(OV2640_FORMAT_RGB565);
    camera_initialized = true;
    return true;
}

void capture_frame() {
    if (camera_initialized) {
        ov2640_capture_frame(camera_buffer, sizeof(camera_buffer));
    }
}

bool wifi_init() {
    if (cyw43_arch_init()) return false;
    cyw43_arch_enable_sta_mode();
    return cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWORD, 
                                            CYW43_AUTH_WPA2_AES_PSK, 10000) == 0;
}

void send_http_response(struct tcp_pcb *tpcb, const char *message) {
    char response[512];
    snprintf(response, sizeof(response),
        "HTTP/1.1 200 OK\r\n"
        "Content-Type: text/html\r\n"
        "Connection: close\r\n\r\n"
        "<html><body>%s</body></html>", 
        message);
    tcp_write(tpcb, response, strlen(response), TCP_WRITE_FLAG_COPY);
}

void send_json_response(struct tcp_pcb *tpcb, const char *json) {
    char response[512];
    snprintf(response, sizeof(response),
        "HTTP/1.1 200 OK\r\n"
        "Content-Type: application/json\r\n"
        "Connection: close\r\n\r\n"
        "%s", 
        json);
    tcp_write(tpcb, response, strlen(response), TCP_WRITE_FLAG_COPY);
}

void handle_encoder_request(struct tcp_pcb *tpcb) {
    char json[256];
    snprintf(json, sizeof(json),
        "{\"stepper\":%d,\"base\":%d,\"joint2\":%d,\"claw\":%d}",
        get_stepper_position(),
        get_base_position(),
        get_joint2_position(),
        get_claw_position());
    
    if (((tcp_server_state_t*)tpcb->callback_arg)->is_websocket) {
        uint8_t frame[256];
        int len = strlen(json);
        frame[0] = 0x81;
        frame[1] = len;
        memcpy(frame + 2, json, len);
        tcp_write(tpcb, frame, len + 2, TCP_WRITE_FLAG_COPY);
    } else {
        send_json_response(tpcb, json);
    }
}

void handle_move_request(struct tcp_pcb *tpcb, const char *params) {
    float x, y;
    int z;
    if (sscanf(params, "x=%f&y=%f&z=%d", &x, &y, &z) == 3) {
        moveTo(x, y, z);
        send_http_response(tpcb, "Arm movement command received");
    } else {
        send_http_response(tpcb, "Invalid parameters");
    }
}

void handle_direct_control(struct tcp_pcb *tpcb, const char *params) {
    int base, joint2, claw, z;
    if (sscanf(params, "base=%d&joint2=%d&claw=%d&z=%d", 
              &base, &joint2, &claw, &z) == 4) {
        moveStepperToZ(z);
        set_servo(SERVO1_PIN, base);
        set_servo(SERVO2_PIN, base);
        set_servo(SERVO3_PIN, joint2);
        set_servo(CLAW_PIN, claw);
        send_http_response(tpcb, "Direct control applied");
    } else {
        send_http_response(tpcb, "Invalid direct control parameters");
    }
}

void handle_websocket_frame(tcp_server_state_t *state, struct pbuf *p) {
    uint8_t *data = (uint8_t*)p->payload;
    if (data[0] == 0x81) {
        int len = data[1] & 0x7F;
        char message[128];
        memcpy(message, data + 2, len);
        message[len] = '\0';
        
        if (strstr(message, "\"cmd\":\"move\"")) {
            float x, y;
            int z;
            if (sscanf(message, "{\"cmd\":\"move\",\"x\":%f,\"y\":%f,\"z\":%d}", &x, &y, &z) == 3) {
                moveTo(x, y, z);
            }
        }
    }
}

void handle_camera_stream(struct tcp_pcb *tpcb) {
    if (!camera_initialized && !init_camera()) {
        const char *error = "HTTP/1.1 500 Internal Error\r\n\r\nCamera initialization failed";
        tcp_write(tpcb, error, strlen(error), TCP_WRITE_FLAG_COPY);
        tcp_close(tpcb);
        return;
    }

    char response[512];
    snprintf(response, sizeof(response),
        "HTTP/1.1 200 OK\r\n"
        "Content-Type: multipart/x-mixed-replace; boundary=frame\r\n"
        "Connection: keep-alive\r\n\r\n");
    
    tcp_write(tpcb, response, strlen(response), TCP_WRITE_FLAG_COPY);

    while (active_ws_client == (tcp_server_state_t*)tpcb->callback_arg) {
        capture_frame();
        
        char frame_header[128];
        int header_len = snprintf(frame_header, sizeof(frame_header),
            "--frame\r\n"
            "Content-Type: image/jpeg\r\n"
            "Content-Length: %d\r\n\r\n", sizeof(camera_buffer));
        
        tcp_write(tpcb, frame_header, header_len, TCP_WRITE_FLAG_COPY);
        tcp_write(tpcb, (char*)camera_buffer, sizeof(camera_buffer), TCP_WRITE_FLAG_COPY);
        
        sleep_ms(1000/CAMERA_FPS);
    }
}

err_t tcp_recv_callback(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err) {
    tcp_server_state_t *state = (tcp_server_state_t*)arg;
    if (!p) {
        if (state->is_websocket && active_ws_client == state) {
            active_ws_client = NULL;
        }
        tcp_close(tpcb);
        free(state);
        return ERR_OK;
    }

    if (state->is_websocket) {
        handle_websocket_frame(state, p);
        pbuf_free(p);
        return ERR_OK;
    }

    pbuf_copy_partial(p, state->buffer, p->tot_len, 0);
    state->buffer[p->tot_len] = '\0';
    
    if (strstr(state->buffer, "Upgrade: websocket")) {
        char *key_start = strstr(state->buffer, "Sec-WebSocket-Key: ");
        if (key_start) {
            key_start += 19;
            char *key_end = strstr(key_start, "\r\n");
            if (key_end) {
                char response[256];
                snprintf(response, sizeof(response),
                    "%s%s\r\n\r\n",
                    ws_response_header,
                    "dummy-key-for-now");
                
                tcp_write(tpcb, response, strlen(response), TCP_WRITE_FLAG_COPY);
                state->is_websocket = true;
                active_ws_client = state;
            }
        }
    }
    else if (strstr(state->buffer, "GET /encoders")) {
        handle_encoder_request(tpcb);
    }
    else if (strstr(state->buffer, "GET /move?")) {
        handle_move_request(tpcb, strstr(state->buffer, "?") + 1);
    }
    else if (strstr(state->buffer, "GET /direct?")) {
        handle_direct_control(tpcb, strstr(state->buffer, "?") + 1);
    }
    else if (strstr(state->buffer, "GET /calibrate")) {
        calibrateArm();
        send_http_response(tpcb, "Calibration started");
    }
    else if (strstr(state->buffer, "GET /stop")) {
        emergency_stop();
        send_http_response(tpcb, "Emergency stop activated");
    }
    else if (strstr(state->buffer, "GET /camera")) {
        handle_camera_stream(tpcb);
    }
    else {
        send_http_response(tpcb, "VR Arm Control Server Ready");
    }

    pbuf_free(p);
    return ERR_OK;
}

err_t tcp_accept_callback(void *arg, struct tcp_pcb *newpcb, err_t err) {
    tcp_server_state_t *state = (tcp_server_state_t*)malloc(sizeof(tcp_server_state_t));
    if (!state) return ERR_MEM;
    memset(state, 0, sizeof(tcp_server_state_t));
    state->pcb = newpcb;
    tcp_arg(newpcb, state);
    tcp_recv(newpcb, tcp_recv_callback);
    return ERR_OK;
}

bool start_web_server(tcp_server_state_t *state) {
    state->pcb = tcp_new();
    if (!state->pcb) return false;
    
    err_t err = tcp_bind(state->pcb, IP_ADDR_ANY, TCP_PORT);
    if (err != ERR_OK) {
        tcp_close(state->pcb);
        return false;
    }
    
    state->pcb = tcp_listen(state->pcb);
    if (!state->pcb) {
        tcp_close(state->pcb);
        return false;
    }
    
    tcp_accept(state->pcb, tcp_accept_callback);
    return true;
}

void send_continuous_updates() {
    while (true) {
        if (active_ws_client) {
            handle_encoder_request(active_ws_client->pcb);
        }
        sleep_ms(50);
    }
}

int main() {
    stdio_init_all();
    init_hardware();

    if (!wifi_init()) {
        uart_send_message("WiFi connection failed\n");
        return 1;
    }

    tcp_server_state_t server_state = {0};
    if (!start_web_server(&server_state)) {
        uart_send_message("Web server failed to start\n");
        return 1;
    }

    uart_send_message("VR Arm Control System Ready\n");
    
    multicore_launch_core1(send_continuous_updates);

    while (true) {
        // Handle UART commands
        std::string cmd = read_uart_line();
        if (!cmd.empty()) {
            if (cmd == "calibrate") calibrateArm();
            else if (cmd == "stop") emergency_stop();
            else {
                float x, y;
                int z;
                if (parseCoordinates(cmd, x, y, z)) {
                    moveTo(x, y, z);
                }
            }
        }

        // Handle WiFi
        cyw43_arch_poll();
        sleep_ms(10);
    }

    return 0;
}
