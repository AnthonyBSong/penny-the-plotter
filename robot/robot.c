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
#define UDP_TARGET "10.59.35.63" // laptop address
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
#define EN_B1   4   // Slice 2 (all 4 enables now on different slices)
#define IN1_B1  19
#define IN2_B1  18

#define EN_B2   22
#define IN1_B2  8
#define IN2_B2  9

// PWM duty cycle max
#define MAX_DUTY 500

// ======================================
// ENCODER DEFINES (QUADRATURE: A/B SIGNALS)
// =============================S=========

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
// Closed-loop control state
// ======================================

// Motor speed setpoints derived from linear_cmd/angular_cmd
volatile int mA1_sp = 0;
volatile int mA2_sp = 0;
volatile int mB1_sp = 0;
volatile int mB2_sp = 0;

// Actual PWM duties applied to motors (control loop updates these)
static int duty_A1 = 0;
static int duty_A2 = 0;
static int duty_B1 = 0;
static int duty_B2 = 0;

// Simple proportional gains
#define MOTOR_KP_NUM 1
#define MOTOR_KP_DEN 4   // Kp ≈ 0.25

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
    uint slice_ia = pwm_gpio_to_slice_num(ia_pin);
    uint slice_ib = pwm_gpio_to_slice_num(ib_pin);
    pwm_set_chan_level(slice_ia, pwm_gpio_to_channel(ia_pin), MAX_DUTY);
    pwm_set_chan_level(slice_ib, pwm_gpio_to_channel(ib_pin), MAX_DUTY);
}

// ======================================
// Encoder Quadrature Decoding (ISR)
// ======================================

void encoder_isr(uint gpio, uint32_t events) {
    bool a_state, b_state;
    
    if (gpio == ENC_A1_A) {
        a_state = gpio_get(ENC_A1_A);
        b_state = gpio_get(ENC_A1_B);
        if (a_state == b_state) enc_count_a1++;
        else                    enc_count_a1--;
    }
    else if (gpio == ENC_A2_A) {
        a_state = gpio_get(ENC_A2_A);
        b_state = gpio_get(ENC_A2_B);
        if (a_state == b_state) enc_count_a2++;
        else                    enc_count_a2--;
    }
    else if (gpio == ENC_B1_A) {
        a_state = gpio_get(ENC_B1_A);
        b_state = gpio_get(ENC_B1_B);
        if (a_state == b_state) enc_count_b1++;
        else                    enc_count_b1--;
    }
    else if (gpio == ENC_B2_A) {
        a_state = gpio_get(ENC_B2_A);
        b_state = gpio_get(ENC_B2_B);
        if (a_state == b_state) enc_count_b2++;
        else                    enc_count_b2--;
    }
}

// ======================================
// Encoder Initialization
// ======================================
void encoder_init() {
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
    
    // Shared callback for all A channels
    gpio_set_irq_enabled_with_callback(
        ENC_A1_A,
        GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL,
        true,
        &encoder_isr
    );
    
    gpio_set_irq_enabled(ENC_A2_A, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
    gpio_set_irq_enabled(ENC_B1_A, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
    gpio_set_irq_enabled(ENC_B2_A, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
}

// ======================================
// Module Motor Control Functions (low-level)
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
// High-Level Swerve Mapping (setpoints)
// ======================================
// Differential swerve math:
//   wheel_speed  = (m1 - m2)
//   steering_yaw = (m1 + m2)

volatile int linear_cmd  = 0;
volatile int angular_cmd = 0;

void update_motor_setpoints_from_cmd(void) {
    int wheel = linear_cmd;
    int yaw   = angular_cmd;

    // these are logical "desired speeds" (sign = direction)
    mA1_sp = yaw + wheel;
    mA2_sp = yaw - wheel;
    mB1_sp = yaw + wheel;
    mB2_sp = yaw - wheel;
}

// Convenience helpers (still go through robot_drive)
void robot_drive(int linear, int angular) {
    // clamp commands to MAX_DUTY
    if (linear >  MAX_DUTY) linear =  MAX_DUTY;
    if (linear < -MAX_DUTY) linear = -MAX_DUTY;
    if (angular >  MAX_DUTY) angular =  MAX_DUTY;
    if (angular < -MAX_DUTY) angular = -MAX_DUTY;

    linear_cmd  = linear;
    angular_cmd = angular;
    update_motor_setpoints_from_cmd();
}

void robot_forward(int speed)  { robot_drive(+speed, 0); }
void robot_backward(int speed) { robot_drive(-speed, 0); }
void robot_rotate_left(int yaw){ robot_drive(0, +yaw); }
void robot_rotate_right(int yaw){ robot_drive(0, -yaw); }

void robot_stop() {
    robot_drive(0, 0);
    duty_A1 = duty_A2 = duty_B1 = duty_B2 = 0;
    moduleA_stop();
    moduleB_stop();
}

// =======================================
// WiFi
// =======================================
#define WIFI_SSID "Anthony"
#define WIFI_PASSWORD "OnUv2NjT"

// =======================================
// protothreads and thread communication
// =======================================
char recv_data[UDP_MSG_LEN_MAX];

volatile uint32_t packets_received = 0;
volatile uint32_t packets_sent = 0;
volatile uint32_t parse_errors = 0;
volatile uint32_t last_recv_time_ms = 0;

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
    memcpy(recv_data, p->payload, UDP_MSG_LEN_MAX);
    PT_SEM_SIGNAL(pt, &new_udp_recv_s) ;
    pbuf_free(p);
  }
  else printf("NULL pt in callback");
}

void udpecho_raw_init(void)
{
  udpecho_raw_pcb = udp_new_ip_type(IPADDR_TYPE_ANY);
  p = pbuf_alloc(PBUF_TRANSPORT, UDP_MSG_LEN_MAX+1, PBUF_RAM);

  if (udpecho_raw_pcb != NULL) {
    err_t err;
    err = udp_bind(udpecho_raw_pcb, netif_ip4_addr(netif_list), UDP_PORT); //DHCP addr

    if (err == ERR_OK) {
      udp_recv(udpecho_raw_pcb, udpecho_raw_recv, NULL);
    } else {
      printf("bind error");
    }
  } else {
    printf("udpecho_raw_pcb error");
  }
}

#endif /* LWIP_UDP */

// =======================================
// UDP send thread
// =======================================
static PT_THREAD (protothread_udp_send(struct pt *pt))
{
    PT_BEGIN(pt);
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
        
        uint32_t uptime_ms = to_ms_since_boot(get_absolute_time());
        sprintf(req, "%lu|%d,%d|rx:%lu,err:%lu",
                packets_sent,
                linear_cmd, angular_cmd,
                packets_received,
                parse_errors);
        
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
        PT_YIELD_usec(UDP_INTERVAL_MS * 1000);
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
    PT_INTERVAL_INIT();

    while(1) {
        PT_SEM_WAIT(pt, &new_udp_recv_s) ;
        
        packets_received++;
        last_recv_time_ms = to_ms_since_boot(get_absolute_time());

        int new_linear = 0, new_angular = 0;
        int parsed = sscanf(recv_data, "%d %d", &new_linear, &new_angular);
        
        if (parsed == 2) {
            robot_drive(new_linear, new_angular);
            printf("[RX] lin=%d ang=%d\n", linear_cmd, angular_cmd);
        } else {
            parse_errors++;
            printf("[ERR] Bad packet: '%s' (parsed %d)\n", recv_data, parsed);
        }
        
        PT_SEM_SIGNAL(pt, &new_udp_send_s) ;
        PT_YIELD_INTERVAL(10);
    }
    PT_END(pt);
}

// ==================================================
// toggle cyw43 LED  (heartbeat)
// ==================================================
static PT_THREAD (protothread_toggle_cyw43(struct pt *pt))
{
    PT_BEGIN(pt);
    static bool LED_state = false ;
    PT_INTERVAL_INIT();

    while(1) {
        LED_state = !LED_state ;
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, LED_state);
        PT_YIELD_INTERVAL(500000); // 500 ms
    }
    PT_END(pt);
}

// ==================================================
// Closed-loop motor control thread
// ==================================================

// helper to get sign
static inline int sgn(int x) {
    if (x > 0) return 1;
    if (x < 0) return -1;
    return 0;
}

// P-control relative to a target magnitude
static void equalize_to_ref(
    int *duty, int ref_mag, int actual_delta, int desired_sign)
{
    if (desired_sign == 0) {
        *duty = 0;
        return;
    }

    // If duty is 0 and we need to move, initialize to a minimum value
    if (*duty == 0 && ref_mag > 0) {
        *duty = desired_sign * (ref_mag / 2);  // Start at half the reference
        return;
    }

    int actual_mag = abs(actual_delta);
    int error      = ref_mag - actual_mag;
    int delta      = (error * MOTOR_KP_NUM) / MOTOR_KP_DEN;

    int new_mag = abs(*duty) + delta;
    
    // Rate limiting: don't increase too fast (max change of 50 per loop)
    int max_change = 50;
    int current_mag = abs(*duty);
    if (new_mag > current_mag + max_change) {
        new_mag = current_mag + max_change;
    } else if (new_mag < current_mag - max_change) {
        new_mag = current_mag - max_change;
    }
    
    if (new_mag < 0)          new_mag = 0;
    if (new_mag > MAX_DUTY)   new_mag = MAX_DUTY;

    *duty = desired_sign * new_mag;
}

// per-motor P-control to track its own setpoint magnitude
static void track_own_setpoint(
    int *duty, int setpoint, int actual_delta)
{
    int sig = sgn(setpoint);
    if (sig == 0) {
        *duty = 0;
        return;
    }

    int target_mag = abs(setpoint);
    int actual_mag = abs(actual_delta);
    int error      = target_mag - actual_mag;
    int delta      = (error * MOTOR_KP_NUM) / MOTOR_KP_DEN;

    int new_mag = abs(*duty) + delta;
    
    // Rate limiting: don't increase too fast (max change of 50 per loop)
    int max_change = 50;
    int current_mag = abs(*duty);
    if (new_mag > current_mag + max_change) {
        new_mag = current_mag + max_change;
    } else if (new_mag < current_mag - max_change) {
        new_mag = current_mag - max_change;
    }
    
    if (new_mag < 0)        new_mag = 0;
    if (new_mag > MAX_DUTY) new_mag = MAX_DUTY;

    *duty = sig * new_mag;
}

static PT_THREAD (protothread_motor_control(struct pt *pt))
{
    PT_BEGIN(pt);
    PT_INTERVAL_INIT();

    static int32_t last_a1 = 0;
    static int32_t last_a2 = 0;
    static int32_t last_b1 = 0;
    static int32_t last_b2 = 0;
    static bool initialized = false;

    while (1) {
        // Snapshot encoder counts and compute deltas (speed estimates)
        int32_t cA1 = enc_count_a1;
        int32_t cA2 = enc_count_a2;
        int32_t cB1 = enc_count_b1;
        int32_t cB2 = enc_count_b2;

        int32_t dA1 = cA1 - last_a1;
        int32_t dA2 = cA2 - last_a2;
        int32_t dB1 = cB1 - last_b1;
        int32_t dB2 = cB2 - last_b2;

        last_a1 = cA1;
        last_a2 = cA2;
        last_b1 = cB1;
        last_b2 = cB2;

        int sigA1 = sgn(mA1_sp);
        int sigA2 = sgn(mA2_sp);
        int sigB1 = sgn(mB1_sp);
        int sigB2 = sgn(mB2_sp);

        // Case 1: robot stopped
        if (linear_cmd == 0 && angular_cmd == 0) {
            duty_A1 = duty_A2 = duty_B1 = duty_B2 = 0;
            initialized = false;
        }
        // Case 2: pure forward/backward -> equalize all motors to one reference
        else if (angular_cmd == 0 && linear_cmd != 0) {
            // Open-loop startup: initialize duties to setpoint values if not initialized
            if (!initialized) {
                duty_A1 = mA1_sp;
                duty_A2 = mA2_sp;
                duty_B1 = mB1_sp;
                duty_B2 = mB2_sp;
                initialized = true;
                printf("[CTRL] Open-loop startup: A1=%d A2=%d B1=%d B2=%d\n", 
                       duty_A1, duty_A2, duty_B1, duty_B2);
            }
            
            // Use A2 or B1 as reference (more reliable than A1)
            // Prefer A2, fallback to B1 if A2 encoder is broken
            int a2_encoder = (dA2 != 0) ? dA2 : dB2;
            int ref_mag = abs(a2_encoder);
            if (ref_mag < 1) {
                ref_mag = abs(dB1);  // Try B1
                if (ref_mag < 1) {
                    ref_mag = abs(mA1_sp);  // Fallback to setpoint
                }
            }

            // All motors equalize to the same reference magnitude
            // This prevents A1 from going too fast independently
            equalize_to_ref(&duty_A1, ref_mag, dA1, sigA1);
            
            // A2 and B2 are on the same motor controller channel - use same duty
            equalize_to_ref(&duty_A2, ref_mag, a2_encoder, sigA2);
            duty_B2 = duty_A2;  // A2 and B2 share same physical channel - same duty
            
            // B1 equalizes to ref magnitude
            equalize_to_ref(&duty_B1, ref_mag, dB1, sigB1);
        }
        // Case 3: pure rotation -> all four should match
        else if (linear_cmd == 0 && angular_cmd != 0) {
            // Open-loop startup
            if (!initialized) {
                duty_A1 = mA1_sp;
                duty_A2 = mA2_sp;
                duty_B1 = mB1_sp;
                duty_B2 = mB2_sp;
                initialized = true;
                printf("[CTRL] Open-loop startup (rotation): A1=%d A2=%d B1=%d B2=%d\n", 
                       duty_A1, duty_A2, duty_B1, duty_B2);
            }
            
            int ref_mag = abs(dA1);
            if (ref_mag < 1) ref_mag = abs(mA1_sp);

            track_own_setpoint(&duty_A1, mA1_sp, dA1);
            
            // A2 and B2 are on the same motor controller channel - use same duty
            int a2_encoder = (dA2 != 0) ? dA2 : dB2;
            equalize_to_ref(&duty_A2, ref_mag, a2_encoder, sigA2);
            duty_B2 = duty_A2;  // A2 and B2 share same physical channel
            
            equalize_to_ref(&duty_B1, ref_mag, dB1, sigB1);
        }
        // Case 4: mixed arc motion -> each motor tracks its own setpoint
        else {
            // Open-loop startup
            if (!initialized) {
                duty_A1 = mA1_sp;
                duty_A2 = mA2_sp;
                duty_B1 = mB1_sp;
                duty_B2 = mB2_sp;
                initialized = true;
                printf("[CTRL] Open-loop startup (arc): A1=%d A2=%d B1=%d B2=%d\n", 
                       duty_A1, duty_A2, duty_B1, duty_B2);
            }
            
            track_own_setpoint(&duty_A1, mA1_sp, dA1);
            
            // A2 and B2 are on the same motor controller channel - use same duty
            // Use A2's setpoint but share the duty with B2
            int a2_encoder = (dA2 != 0) ? dA2 : dB2;
            track_own_setpoint(&duty_A2, mA2_sp, a2_encoder);
            duty_B2 = duty_A2;  // A2 and B2 share same physical channel
            
            track_own_setpoint(&duty_B1, mB1_sp, dB1);
        }

        // Apply updated duties to hardware
        moduleA_motor1(duty_A1);
        moduleA_motor2(duty_A2);
        moduleB_motor1(duty_B1);
        moduleB_motor2(duty_B2);
        
        // Debug: Print motor info periodically (every 20 loops = ~1 second)
        static int debug_counter = 0;
        if (++debug_counter >= 20) {
            printf("Motor duties: A1=%d A2=%d B1=%d B2=%d | Setpoints: A1=%d A2=%d B1=%d B2=%d\n",
                   duty_A1, duty_A2, duty_B1, duty_B2,
                   mA1_sp, mA2_sp, mB1_sp, mB2_sp);
            printf("Encoder deltas: A1=%ld A2=%ld B1=%ld B2=%ld\n",
                   dA1, dA2, dB1, dB2);
            debug_counter = 0;
        }

        // Very important: yield so other threads keep running (non-blocking)
        PT_YIELD_INTERVAL(50000); // 50 ms control loop
    }

    PT_END(pt);
}

// ====================================================
// Motor GPIO/PWM initialization
// ====================================================
void motor_init() {
    int standard_pins[] = {
        EN_A1, IN1_A1, IN2_A1,
        EN_A2, IN1_A2, IN2_A2,
        EN_B1, IN1_B1, IN2_B1,
        EN_B2, IN1_B2, IN2_B2
    };

    for (int i = 0; i < 12; i++) {
        gpio_init(standard_pins[i]);
        gpio_set_dir(standard_pins[i], GPIO_OUT);
    }

    pwm_setup(EN_A1);
    pwm_setup(EN_A2);
    pwm_setup(EN_B1);
    pwm_setup(EN_B2);
    
    // Print PWM slice info for debugging
    printf("PWM Slice Assignments:\n");
    printf("  EN_A1 (GP%d): Slice %d, Channel %c\n", EN_A1, pwm_gpio_to_slice_num(EN_A1), pwm_gpio_to_channel(EN_A1) ? 'B' : 'A');
    printf("  EN_A2 (GP%d): Slice %d, Channel %c\n", EN_A2, pwm_gpio_to_slice_num(EN_A2), pwm_gpio_to_channel(EN_A2) ? 'B' : 'A');
    printf("  EN_B1 (GP%d): Slice %d, Channel %c\n", EN_B1, pwm_gpio_to_slice_num(EN_B1), pwm_gpio_to_channel(EN_B1) ? 'B' : 'A');
    printf("  EN_B2 (GP%d): Slice %d, Channel %c\n", EN_B2, pwm_gpio_to_slice_num(EN_B2), pwm_gpio_to_channel(EN_B2) ? 'B' : 'A');
    
    // Verify B2 pins are configured correctly
    printf("\nB2 Motor Pin Configuration:\n");
    printf("  EN_B2: GP%d (PWM)\n", EN_B2);
    printf("  IN1_B2: GP%d (GPIO)\n", IN1_B2);
    printf("  IN2_B2: GP%d (GPIO)\n", IN2_B2);
    
    robot_stop();
}

// ====================================================
int main() {
    stdio_init_all();

    motor_init();
    printf("Motors initialized\n");

    encoder_init();
    printf("Encoders initialized\n");

    printf("Waiting for power to stabilize...\n");
    sleep_ms(1000);

    if (cyw43_arch_init()) {
        printf("failed to initialise cyw43\n");
        return 1;
    }

    cyw43_arch_enable_sta_mode();

    int max_retries = 5;
    int retry_count = 0;
    bool connected = false;

    while (!connected && retry_count < max_retries) {
        printf("WiFi connection attempt %d/%d...\n", retry_count + 1, max_retries);
        
        if (cyw43_arch_wifi_connect_timeout_ms(
                WIFI_SSID, WIFI_PASSWORD, 
                CYW43_AUTH_WPA2_AES_PSK, 30000) == 0) {
            connected = true;
            printf("Connected: picoW IP addr: %s\n",
                   ip4addr_ntoa(netif_ip4_addr(netif_list)));
        } else {
            printf("Connection attempt %d failed, retrying...\n",
                   retry_count + 1);
            retry_count++;
            sleep_ms(2000);
        }
    }

    if (!connected) {
        printf("Failed to connect to WiFi after %d attempts\n", max_retries);
        return 1;
    }

    udpecho_raw_init();

    PT_SEM_INIT(&new_udp_send_s, 0);
    PT_SEM_INIT(&new_udp_recv_s, 0);

    pt_add_thread(protothread_udp_send);
    pt_add_thread(protothread_udp_recv);
    pt_add_thread(protothread_toggle_cyw43);
    pt_add_thread(protothread_motor_control);

    pt_schedule_start;

    cyw43_arch_deinit();
    return 0;
}