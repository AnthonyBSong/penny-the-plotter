#include <cstdio>
#include <cstring>
#include <string>

#include "pico/stdlib.h"
#include "hardware/pwm.h"

#include "pico/cyw43_arch.h"
#include "lwip/udp.h"
#include "lwip/pbuf.h"
#include "lwip/ip_addr.h"

// PIN ASSIGNMENTS (Temporary for now)
#define PWM_LEFT_GPIO    TODO 
#define PWM_RIGHT_GPIO   TODO 
#define DIR_LEFT_GPIO    TODO 
#define DIR_RIGHT_GPIO   TODO 

static const float MAX_SPEED = 60.0f; 

static float g_lws = 0.0f;
static float g_rws = 0.0f;

// PWM HELPERS

static void setup_pwm_gpio(uint gpio, uint32_t freq_hz, float initial_duty_percent) {
    // Use PWM function on this pin
    gpio_set_function(gpio, GPIO_FUNC_PWM);
    uint slice = pwm_gpio_to_slice_num(gpio);

    // Simple mapping: wrap = 100 → duty in [0..100] directly
    uint16_t wrap = 100;
    pwm_set_wrap(slice, wrap);

    // Compute clock divider for desired PWM frequency
    float clk_sys = 125000000.0f;  // default RP2040 clock
    float clkdiv = clk_sys / (freq_hz * (wrap + 1));
    pwm_set_clkdiv(slice, clkdiv);

    // Initial duty
    uint channel = pwm_gpio_to_channel(gpio);
    uint16_t level = (uint16_t)((initial_duty_percent / 100.0f) * wrap);
    pwm_set_chan_level(slice, channel, level);

    pwm_set_enabled(slice, true);
}

static void set_pwm_duty(uint gpio, float duty_percent) {
    if (duty_percent < 0.0f) duty_percent = 0.0f;
    if (duty_percent > 100.0f) duty_percent = 100.0f;

    uint slice = pwm_gpio_to_slice_num(gpio);
    uint channel = pwm_gpio_to_channel(gpio);

    // wrap = 100 → duty directly
    uint16_t level = (uint16_t)duty_percent;
    pwm_set_chan_level(slice, channel, level);
}

// UDP CALLBACk

static void udp_recv_cb(void *arg, struct udp_pcb *pcb, struct pbuf *p,
                        const ip_addr_t *addr, u16_t port) {
    if (!p) return;

    // Copy payload into a C++ string
    std::string payload;
    payload.assign((char *)p->payload, p->len);

    pbuf_free(p);

    // Expect "x_coor,y_coor"
    float x_coor = 0.0f;
    float y_coor = 0.0f;

    if (sscanf(payload.c_str(), "%f,%f", &x_coor, &y_coor) == 2) {
        float lws = y_coor + x_coor;
        float rws = -1.0f * (y_coor - x_coor);

        g_lws = lws;
        g_rws = rws;

        printf("UDP: x=%f, y=%f, lws=%f, rws=%f\n", x_coor, y_coor, lws, rws);
    } else {
        printf("Malformed UDP packet: '%s'\n", payload.c_str());
    }
}

int main() {
    stdio_init_all();
    sleep_ms(2000); // give USB serial time

    // WiFi INIT

    if (cyw43_arch_init()) {
        printf("Failed to init cyw43_arch\n");
        return -1;
    }

    cyw43_arch_enable_sta_mode();

    // TODO: put your WiFi credentials here
    const char *ssid = "YOUR_SSID";
    const char *pass = "YOUR_PASSWORD";

    printf("Connecting to WiFi SSID: %s\n", ssid);
    if (cyw43_arch_wifi_connect_timeout_ms(ssid, pass,
                                           CYW43_AUTH_WPA2_AES_PSK,
                                           30000)) {
        printf("Failed to connect to WiFi\n");
        return -1;
    }
    printf("WiFi connected.\n");

    // GPIO DIR PINS

    gpio_init(DIR_LEFT_GPIO);
    gpio_set_dir(DIR_LEFT_GPIO, GPIO_OUT);
    gpio_put(DIR_LEFT_GPIO, 0);

    gpio_init(DIR_RIGHT_GPIO);
    gpio_set_dir(DIR_RIGHT_GPIO, GPIO_OUT);
    gpio_put(DIR_RIGHT_GPIO, 0);

    // PWM PINS

    setup_pwm_gpio(PWM_LEFT_GPIO, 1000, 0.0f);   // 1 kHz PWM
    setup_pwm_gpio(PWM_RIGHT_GPIO, 1000, 0.0f);

    // UDP SETUP

    struct udp_pcb *pcb = udp_new_ip_type(IPADDR_TYPE_ANY);
    if (!pcb) {
        printf("Failed to create UDP PCB\n");
        return -1;
    }

    err_t err = udp_bind(pcb, IP_ANY_TYPE, 5005);  // same as your Python port
    if (err != ERR_OK) {
        printf("UDP bind failed: %d\n", err);
        return -1;
    }

    udp_recv(pcb, udp_recv_cb, nullptr);

    printf("UDP listener up on port 5005.\n");

    // Control Loop

    while (true) {
        float lws = g_lws;
        float rws = g_rws;

        if (lws < 0.0f) {
            float speed = (1.0f + lws) * MAX_SPEED;
            if (speed < 0.0f) speed = 0.0f;
            if (speed > 100.0f) speed = 100.0f;
            set_pwm_duty(PWM_LEFT_GPIO, speed);
            gpio_put(DIR_LEFT_GPIO, 1);   // reverse
        } else {
            float speed = (lws) * MAX_SPEED;
            if (speed < 0.0f) speed = 0.0f;
            if (speed > 100.0f) speed = 100.0f;
            set_pwm_duty(PWM_LEFT_GPIO, speed);
            gpio_put(DIR_LEFT_GPIO, 0);   // forward
        }

        if (rws < 0.0f) {
            float speed = (1.0f + rws) * MAX_SPEED;
            if (speed < 0.0f) speed = 0.0f;
            if (speed > 100.0f) speed = 100.0f;
            set_pwm_duty(PWM_RIGHT_GPIO, speed);
            gpio_put(DIR_RIGHT_GPIO, 1);  // reverse
        } else {
            float speed = (rws) * MAX_SPEED;
            if (speed < 0.0f) speed = 0.0f;
            if (speed > 100.0f) speed = 100.0f;
            set_pwm_duty(PWM_RIGHT_GPIO, speed);
            gpio_put(DIR_RIGHT_GPIO, 0);  // forward
        }

        sleep_ms(10);
    }

    cyw43_arch_deinit();
    return 0;
}
