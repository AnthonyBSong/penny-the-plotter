/**
 * Copyright (c) 2022 Andrew McDonnell
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

// Standard libraries
#include <string.h>
#include <stdlib.h>
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
#define EN_A1   15
#define IN1_A1  14
#define IN2_A1  13

#define EN_A2   10
#define IN1_A2  12
#define IN2_A2  11

// -------- Module B (Right Swerve Module) --------
#define EN_B1   16
#define IN1_B1  17
#define IN2_B1  18

#define EN_B2   19
#define IN1_B2  20
#define IN2_B2  21

// PWM duty cycle max
#define MAX_DUTY 500

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
void moduleB_motor2(int duty) { motor_set(IN1_B2, IN2_B2, EN_B2, duty); }

void moduleB_stop() {
    stop_motor(IN1_B1, IN2_B1, EN_B1);
    stop_motor(IN1_B2, IN2_B2, EN_B2);
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
#define WIFI_SSID "Anthony"
#define WIFI_PASSWORD "OnUv2NjT"

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
    // All motor control pins
    int all_pins[] = {
        EN_A1, IN1_A1, IN2_A1,
        EN_A2, IN1_A2, IN2_A2,
        EN_B1, IN1_B1, IN2_B1,
        EN_B2, IN1_B2, IN2_B2
    };

    // Configure all pins as outputs
    for (int i = 0; i < 12; i++) {
        gpio_init(all_pins[i]);
        gpio_set_dir(all_pins[i], GPIO_OUT);
    }

    // Setup PWM for EN pins
    pwm_setup(EN_A1);
    pwm_setup(EN_A2);
    pwm_setup(EN_B1);
    pwm_setup(EN_B2);
    
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
  // init the wifi network
    if (cyw43_arch_init()) {
        printf("failed to initialise cyw43\n");
        return 1;
    }

    cyw43_arch_enable_sta_mode();

    printf("Connecting to Wi-Fi...\n");
    if (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK, 50000)) {
        printf("failed to connect: cyw43 arch wifi timeout.\n");
        return 1;
    } else {
      // optional print addr
        printf("Connected: picoW IP addr: %s\n", ip4addr_ntoa(netif_ip4_addr(netif_list)));
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