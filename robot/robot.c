/**
 * Copyright (c) 2022 Andrew McDonnell
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

// Standard libraries
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <pico/multicore.h>
#include "hardware/sync.h"
#include "hardware/gpio.h"
#include "hardware/timer.h"
#include "hardware/uart.h"
#include "hardware/pwm.h"
#include "stdio.h"

#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"

#include "lwip/pbuf.h"
#include "lwip/udp.h"
#include "lwip/opt.h"
#include "lwip/debug.h"
#include "lwip/stats.h"
#include "lwip/dns.h"
#include "lwip/netif.h"
#include "pt_cornell_rp2040_v1_4.h"

// ======================================
// udp constants
#define UDP_PORT 5555
#define UDP_MSG_LEN_MAX 64
#define UDP_TARGET "10.59.35.63" // my laptop addresss
#define UDP_INTERVAL_MS 10

// ======================================
// Motor Control Pin Definitions
// ======================================

// -------- Module A (Left Swerve Module) --------
#define EN_A1   10
#define IN1_A1  11
#define IN2_A1  12

#define EN_A2   13
#define IN1_A2  14
#define IN2_A2  15

// -------- Module B (Right Swerve Module) --------
#define EN_B1   26
#define IN1_B1  19
#define IN2_B1  18

// L9110H motor driver for B2 (no enable pin, uses IA/IB for direction + PWM)
#define IA_B2   4
#define IB_B2   5

// PWM duty cycle max
#define MAX_DUTY 500

// ======================================
// ENCODER DEFINES (QUADRATURE: A/B SIGNALS)
// ======================================
// Each encoder has two signal wires: phase A and phase B
// green wire as A
// yellow wire as B

#define ENC_A1_A    6
#define ENC_A1_B    7

#define ENC_A2_A    16
#define ENC_A2_B    17

#define ENC_B1_A    2
#define ENC_B1_B    3

#define ENC_B2_A    20
#define ENC_B2_B    21

// ======================================
// Encoder global state
// ======================================
volatile int32_t enc_count_a1 = 0;
volatile int32_t enc_count_a2 = 0;
volatile int32_t enc_count_b1 = 0;
volatile int32_t enc_count_b2 = 0;

// ======================================
// Motor Control Helper Functions
// ======================================

void pwm_setup(uint pin) {
    gpio_set_function(pin, GPIO_FUNC_PWM);
    uint slice = pwm_gpio_to_slice_num(pin);
    pwm_set_wrap(slice, MAX_DUTY);
    pwm_set_clkdiv(slice, 100.0f);   // ~1 kHz PWM
    pwm_set_enabled(slice, true);
}

void motor_set(int in1, int in2, int en, int duty) {
    gpio_put(in1, duty > 0);
    gpio_put(in2, duty < 0);
    uint slice = pwm_gpio_to_slice_num(en);
    uint level = abs(duty);
    if (level > MAX_DUTY) level = MAX_DUTY;
    pwm_set_chan_level(slice, pwm_gpio_to_channel(en), level);
}

void stop_motor(int in1, int in2, int en) {
    gpio_put(in1, 0);
    gpio_put(in2, 0);
    uint slice = pwm_gpio_to_slice_num(en);
    pwm_set_chan_level(slice, pwm_gpio_to_channel(en), 0);
}

// L9110H motor control: PWM on IA for forward, PWM on IB for reverse
void l9110h_motor_set(int ia_pin, int ib_pin, int duty) {
    uint slice_ia = pwm_gpio_to_slice_num(ia_pin);
    uint slice_ib = pwm_gpio_to_slice_num(ib_pin);
    uint level = abs(duty);
    if (level > MAX_DUTY) level = MAX_DUTY;
    
    if (duty > 0) {
        // Forward: PWM on IA, LOW on IB
        pwm_set_chan_level(slice_ia, pwm_gpio_to_channel(ia_pin), level);
        pwm_set_chan_level(slice_ib, pwm_gpio_to_channel(ib_pin), 0);
    } else if (duty < 0) {
        // Reverse: LOW on IA, PWM on IB
        pwm_set_chan_level(slice_ia, pwm_gpio_to_channel(ia_pin), 0);
        pwm_set_chan_level(slice_ib, pwm_gpio_to_channel(ib_pin), level);
    } else {
        // Brake mode: both HIGH to actively stop
        pwm_set_chan_level(slice_ia, pwm_gpio_to_channel(ia_pin), MAX_DUTY);
        pwm_set_chan_level(slice_ib, pwm_gpio_to_channel(ib_pin), MAX_DUTY);
    }
}

void l9110h_motor_stop(int ia_pin, int ib_pin) {
    // Brake mode: set both pins to full duty (HIGH) to actively stop the motor
    uint slice_ia = pwm_gpio_to_slice_num(ia_pin);
    uint slice_ib = pwm_gpio_to_slice_num(ib_pin);
    pwm_set_chan_level(slice_ia, pwm_gpio_to_channel(ia_pin), MAX_DUTY);
    pwm_set_chan_level(slice_ib, pwm_gpio_to_channel(ib_pin), MAX_DUTY);
}

// ======================================
// Encoder Quadrature Decoding (ISR)
// ======================================
// Called on any edge of channel A pins
// Reads channel B to determine direction

void encoder_isr(uint gpio, uint32_t events) {
    bool a_state, b_state;
    
    if (gpio == ENC_A1_A) {
        a_state = gpio_get(ENC_A1_A);
        b_state = gpio_get(ENC_A1_B);
        if (a_state == b_state) {
            enc_count_a1++;
        } else {
            enc_count_a1--;
        }
    }
    else if (gpio == ENC_A2_A) {
        a_state = gpio_get(ENC_A2_A);
        b_state = gpio_get(ENC_A2_B);
        if (a_state == b_state) {
            enc_count_a2++;
        } else {
            enc_count_a2--;
        }
    }
    else if (gpio == ENC_B1_A) {
        a_state = gpio_get(ENC_B1_A);
        b_state = gpio_get(ENC_B1_B);
        if (a_state == b_state) {
            enc_count_b1++;
        } else {
            enc_count_b1--;
        }
    }
    else if (gpio == ENC_B2_A) {
        a_state = gpio_get(ENC_B2_A);
        b_state = gpio_get(ENC_B2_B);
        if (a_state == b_state) {
            enc_count_b2++;
        } else {
            enc_count_b2--;
        }
    }
}

// ======================================
// Encoder Initialization
// ======================================
void encoder_init() {
    // Initialize encoder A pins as inputs with pull-ups
    int enc_a_pins[] = {ENC_A1_A, ENC_A2_A, ENC_B1_A, ENC_B2_A};
    int enc_b_pins[] = {ENC_A1_B, ENC_A2_B, ENC_B1_B, ENC_B2_B};
    
    for (int i = 0; i < 4; i++) {
        // Channel A - triggers interrupt
        gpio_init(enc_a_pins[i]);
        gpio_set_dir(enc_a_pins[i], GPIO_IN);
        gpio_pull_up(enc_a_pins[i]);
        
        // Channel B - read for direction
        gpio_init(enc_b_pins[i]);
        gpio_set_dir(enc_b_pins[i], GPIO_IN);
        gpio_pull_up(enc_b_pins[i]);
    }
    
    // Set up interrupt callback (shared for all GPIOs)
    gpio_set_irq_enabled_with_callback(ENC_A1_A, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &encoder_isr);
    
    // Enable interrupts for other encoder A channels (callback already registered)
    gpio_set_irq_enabled(ENC_A2_A, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
    gpio_set_irq_enabled(ENC_B1_A, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
    gpio_set_irq_enabled(ENC_B2_A, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
}

// Helper functions to read/reset encoder counts
int32_t get_encoder_a1() { return enc_count_a1; }
int32_t get_encoder_a2() { return enc_count_a2; }
int32_t get_encoder_b1() { return enc_count_b1; }
int32_t get_encoder_b2() { return enc_count_b2; }

void reset_encoder_a1() { enc_count_a1 = 0; }
void reset_encoder_a2() { enc_count_a2 = 0; }
void reset_encoder_b1() { enc_count_b1 = 0; }
void reset_encoder_b2() { enc_count_b2 = 0; }

void reset_all_encoders() {
    enc_count_a1 = 0;
    enc_count_a2 = 0;
    enc_count_b1 = 0;
    enc_count_b2 = 0;
}

// ======================================
// Module Motor Control Functions
// ======================================

// ---------- MODULE A (Left) ----------
void moduleA_motor1(int duty) { motor_set(IN1_A1, IN2_A1, EN_A1, duty); }
void moduleA_motor2(int duty) { motor_set(IN1_A2, IN2_A2, EN_A2, duty); }

void moduleA_stop() {
    stop_motor(IN1_A1, IN2_A1, EN_A1);
    stop_motor(IN1_A2, IN2_A2, EN_A2);
}

// ---------- MODULE B (Right) ----------
void moduleB_motor1(int duty) { motor_set(IN1_B1, IN2_B1, EN_B1, duty); }
void moduleB_motor2(int duty) { l9110h_motor_set(IA_B2, IB_B2, duty); }  // L9110H driver

void moduleB_stop() {
    stop_motor(IN1_B1, IN2_B1, EN_B1);
    l9110h_motor_stop(IA_B2, IB_B2);  // L9110H driver
}

// ======================================
// High-Level Swerve Commands
// ======================================
// Differential swerve math:
//   wheel_speed  = (m1 - m2)
//   steering_yaw = (m1 + m2)

void moduleA_set(int wheel_cmd, int yaw_cmd) {
    int m1 = yaw_cmd + wheel_cmd;
    int m2 = yaw_cmd - wheel_cmd;
    moduleA_motor1(m1);
    moduleA_motor2(m2);
}

void moduleB_set(int wheel_cmd, int yaw_cmd) {
    int m1 = yaw_cmd + wheel_cmd;
    int m2 = yaw_cmd - wheel_cmd;
    moduleB_motor1(m1);
    moduleB_motor2(m2);
}

// ======================================
// Whole Robot Movement Commands
// ======================================

void robot_drive(int linear, int angular) {
    // linear: positive=forward, negative=backward
    // angular: positive=left, negative=right
    moduleA_set(linear, angular);
    moduleB_set(linear, angular);
}

void robot_forward(int speed) {
    moduleA_set(+speed, 0);
    moduleB_set(+speed, 0);
}

void robot_backward(int speed) {
    moduleA_set(-speed, 0);
    moduleB_set(-speed, 0);
}

void robot_rotate_left(int yaw) {
    moduleA_set(0, +yaw);
    moduleB_set(0, +yaw);
}

void robot_rotate_right(int yaw) {
    moduleA_set(0, -yaw);
    moduleB_set(0, -yaw);
}

void robot_stop() {
    moduleA_stop();
    moduleB_stop();
}

// =======================================
// necessary to connect to wireless
// !!! Do NOT post this info !!!
#define WIFI_SSID "XXX"
#define WIFI_PASSWORD "XXX"

// =======================================
// protothreads and thread communication
char recv_data[UDP_MSG_LEN_MAX];

// Motor command values (from UDP)
// Format: "<linear_speed> <angular_speed>"
// linear_speed:  -500 to 500 (negative=backward, positive=forward)
// angular_speed: -500 to 500 (negative=right, positive=left)
volatile int linear_cmd = 0;
volatile int angular_cmd = 0;

// Debug/stats tracking
volatile uint32_t packets_received = 0;
volatile uint32_t packets_sent = 0;
volatile uint32_t parse_errors = 0;
volatile uint32_t last_recv_time_ms = 0;

// interthread communication
struct pt_sem new_udp_recv_s, new_udp_send_s ;

#if LWIP_UDP

static struct udp_pcb *udpecho_raw_pcb;
struct pbuf *p ;

static void
udpecho_raw_recv(void *arg, struct udp_pcb *upcb, struct pbuf *p,
                 const ip_addr_t *addr, u16_t port)
{
  LWIP_UNUSED_ARG(arg);

  if (p != NULL) {
    //printf("p payload in call back: = %s\n", p->payload);
    memcpy(recv_data, p->payload, UDP_MSG_LEN_MAX);
    // can signal from an ISR -- BUT NEVER wait in an ISR
    PT_SEM_SIGNAL(pt, &new_udp_recv_s) ;
    
    /* free the pbuf */
    pbuf_free(p);
  }
  else printf("NULL pt in callback");
}

// ===================================
// Define the recv callback 
void 
udpecho_raw_init(void)
{
  udpecho_raw_pcb = udp_new_ip_type(IPADDR_TYPE_ANY);
  p = pbuf_alloc(PBUF_TRANSPORT, UDP_MSG_LEN_MAX+1, PBUF_RAM);

  if (udpecho_raw_pcb != NULL) {
    err_t err;
    // netif_ip4_addr returns the picow ip address
    err = udp_bind(udpecho_raw_pcb, netif_ip4_addr(netif_list), UDP_PORT); //DHCP addr

    if (err == ERR_OK) {
      udp_recv(udpecho_raw_pcb, udpecho_raw_recv, NULL);
      //printf("Set up recv callback\n");
    } else {
      printf("bind error");
    }
  } else {
    printf("udpecho_raw_pcb error");
  }
}

#endif /* LWIP_UDP */
// end recv setup

// =======================================
// UDP send thead
// for now just bounces back the recv packet

static PT_THREAD (protothread_udp_send(struct pt *pt))
 { PT_BEGIN(pt);
    static struct udp_pcb* pcb;
    pcb = udp_new();
    pcb->remote_port = UDP_PORT ;
    pcb->local_port = UDP_PORT ;

    static ip_addr_t addr;
    ipaddr_aton(UDP_TARGET, &addr);
    
    while (true) {
        
        PT_SEM_WAIT(pt, &new_udp_send_s) ;

        struct pbuf *p = pbuf_alloc(PBUF_TRANSPORT, UDP_MSG_LEN_MAX+1, PBUF_RAM);
        char *req = (char *)p->payload;
        memset(req, 0, UDP_MSG_LEN_MAX+1);
        
        // Send debug info: seq#, motor cmds, uptime, stats
        uint32_t uptime_ms = to_ms_since_boot(get_absolute_time());
        sprintf(req, "%lu|%d,%d|rx:%lu,err:%lu", 
                packets_sent,           // packet sequence number
                linear_cmd, angular_cmd, // current motor commands
                packets_received,        // total packets received
                parse_errors);           // parse error count
        
        err_t er = udp_sendto(pcb, p, &addr, UDP_PORT);
       
        pbuf_free(p);
        if (er != ERR_OK) {
            printf("UDP send error=%d\n", er);
        } else {
            packets_sent++;
        }

#if PICO_CYW43_ARCH_POLL
        cyw43_arch_poll();
        sleep_ms(BEACON_INTERVAL_MS);
#else
        PT_YIELD_usec(UDP_INTERVAL_MS*1000);
#endif
    }

    PT_END(pt);
}

// ==================================================
// udp recv processing - parses motor commands
// Format: "<linear_speed> <angular_speed>"
// ==================================================
static PT_THREAD (protothread_udp_recv(struct pt *pt))
{
    PT_BEGIN(pt);
    
    // data structure for interval timer
    PT_INTERVAL_INIT() ;

    while(1) {
        // wait for new packet
        PT_SEM_WAIT(pt, &new_udp_recv_s) ;
        
        packets_received++;
        last_recv_time_ms = to_ms_since_boot(get_absolute_time());

        // Parse motor command: "linear angular"
        int new_linear = 0, new_angular = 0;
        int parsed = sscanf(recv_data, "%d %d", &new_linear, &new_angular);
        
        if (parsed == 2) {
            // Clamp values to valid range
            if (new_linear > MAX_DUTY) new_linear = MAX_DUTY;
            if (new_linear < -MAX_DUTY) new_linear = -MAX_DUTY;
            if (new_angular > MAX_DUTY) new_angular = MAX_DUTY;
            if (new_angular < -MAX_DUTY) new_angular = -MAX_DUTY;
            
            linear_cmd = new_linear;
            angular_cmd = new_angular;
            
            // Apply motor commands
            robot_drive(linear_cmd, angular_cmd);
            
            printf("[RX] lin=%d ang=%d\n", linear_cmd, angular_cmd);
        } else {
            parse_errors++;
            printf("[ERR] Bad packet: '%s' (parsed %d)\n", recv_data, parsed);
        }
        
        // tell send thread to acknowledge
        PT_SEM_SIGNAL(pt, &new_udp_send_s) ;

        PT_YIELD_INTERVAL(10) ;
        //
        // NEVER exit while
    } // END WHILE(1)
    PT_END(pt);
} // motor command thread

// ==================================================
// toggle cyw43 LED  
// Heartbeat indicator - blinks to show system is alive
// ==================================================
static PT_THREAD (protothread_toggle_cyw43(struct pt *pt))
{
    PT_BEGIN(pt);
    static bool LED_state = false ;
    //
    // data structure for interval timer
    PT_INTERVAL_INIT() ;

    while(1) {
        //
        LED_state = !LED_state ;
        // the onboard LED is attached to the wifi module
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, LED_state);
        // Fixed 500ms blink as heartbeat
        PT_YIELD_INTERVAL(500000) ;
        //
        // NEVER exit while
    } // END WHILE(1)
    PT_END(pt);
} // heartbeat thread


// ====================================================
// Motor GPIO/PWM initialization
// ====================================================
void motor_init() {
    // Standard motor driver pins (EN/IN1/IN2)
    int standard_pins[] = {
        EN_A1, IN1_A1, IN2_A1,
        EN_A2, IN1_A2, IN2_A2,
        EN_B1, IN1_B1, IN2_B1
    };

    // Configure standard motor pins as outputs
    for (int i = 0; i < 9; i++) {
        gpio_init(standard_pins[i]);
        gpio_set_dir(standard_pins[i], GPIO_OUT);
    }

    // Setup PWM for standard EN pins
    pwm_setup(EN_A1);
    pwm_setup(EN_A2);
    pwm_setup(EN_B1);
    
    // L9110H motor driver for B2 - both IA and IB need PWM
    pwm_setup(IA_B2);
    pwm_setup(IB_B2);
    
    // Start with motors stopped
    robot_stop();
}

// ====================================================
int main() {
  // =======================
  // init the serial
    stdio_init_all();

  // =======================
  // init the motors
    motor_init();
    printf("Motors initialized\n");

  // =======================
  // init the encoders
    encoder_init();
    printf("Encoders initialized\n");

  // =======================
  // init the wifi network
  
    // Add delay after power-up to let voltage stabilize (important for battery power)
    printf("Waiting for power to stabilize...\n");
    sleep_ms(1000);

    if (cyw43_arch_init()) {
        printf("failed to initialise cyw43\n");
        return 1;
    }

    cyw43_arch_enable_sta_mode();

    // Retry WiFi connection with delays between attempts (helps with battery power)
    int max_retries = 5;
    int retry_count = 0;
    bool connected = false;

    while (!connected && retry_count < max_retries) {
        printf("WiFi connection attempt %d/%d...\n", retry_count + 1, max_retries);
        
        if (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWORD, 
                CYW43_AUTH_WPA2_AES_PSK, 30000) == 0) {
            connected = true;
            printf("Connected: picoW IP addr: %s\n", ip4addr_ntoa(netif_ip4_addr(netif_list)));
        } else {
            printf("Connection attempt %d failed, retrying...\n", retry_count + 1);
            retry_count++;
            sleep_ms(2000);  // Wait before retry to let power stabilize
        }
    }

    if (!connected) {
        printf("Failed to connect to WiFi after %d attempts\n", max_retries);
        return 1;
    }

    //============================
    // UDP receive ISR routines
    udpecho_raw_init();

      //========================================
  // start core 1 threads -- none here
  //multicore_reset_core1();
  //multicore_launch_core1(&core1_main);

  // === config threads ========================
  // for core 0

  // init the thread control semaphores
  PT_SEM_INIT(&new_udp_send_s, 0) ;
  PT_SEM_INIT(&new_udp_recv_s, 0) ;

  //printf("Starting threads\n") ;
  pt_add_thread(protothread_udp_send);
  pt_add_thread(protothread_udp_recv);
  pt_add_thread(protothread_toggle_cyw43) ;
  //pt_add_thread(protothread_serial) ;
  //
  // === initalize the scheduler ===============
  pt_schedule_start ;

    cyw43_arch_deinit();
    return 0;
}