#include "pico/cyw43_arch.h"
#include "lwip/tcp.h"
#include "Arm.h"
#include "wifi.h"

#define TCP_PORT 80
#define BUF_SIZE 2048

typedef struct {
    struct tcp_pcb *pcb;
    char buffer[BUF_SIZE];
    int request_len;
} tcp_server_state_t;

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
    send_json_response(tpcb, json);
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

err_t tcp_recv_callback(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err) {
    tcp_server_state_t *state = (tcp_server_state_t*)arg;
    if (!p) {
        tcp_close(tpcb);
        return ERR_OK;
    }

    pbuf_copy_partial(p, state->buffer, p->tot_len, 0);
    state->buffer[p->tot_len] = '\0';
    
    if (strstr(state->buffer, "GET /encoders")) {
        handle_encoder_request(tpcb);
    }
    else if (strstr(state->buffer, "GET /move?")) {
        handle_move_request(tpcb, strstr(state->buffer, "?") + 1);
    }
    else if (strstr(state->buffer, "GET /calibrate")) {
        calibrateArm();
        send_http_response(tpcb, "Calibration started");
    }
    else if (strstr(state->buffer, "GET /stop")) {
        emergency_stop();
        send_http_response(tpcb, "Emergency stop activated");
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
    state->pcb = newpcb;
    tcp_arg(newpcb, state);
    tcp_recv(newpcb, tcp_recv_callback);
    return ERR_OK;
}

bool start_web_server(tcp_server_state_t *state) {
    state->pcb = tcp_new();
    tcp_bind(state->pcb, IP_ADDR_ANY, TCP_PORT);
    state->pcb = tcp_listen(state->pcb);
    tcp_accept(state->pcb, tcp_accept_callback);
    return true;
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
