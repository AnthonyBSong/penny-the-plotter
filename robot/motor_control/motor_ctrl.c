#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include <stdlib.h>

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

// ============================================================
//   HELPER FUNCTIONS
// ============================================================

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

// ============================================================
//  MODULE MOTOR CONTROL
//  (These act on the raw motor pair for each swerve module)
// ============================================================

// ---------- MODULE A ----------
void moduleA_motor1(int duty) { motor_set(IN1_A1, IN2_A1, EN_A1, duty); }
void moduleA_motor2(int duty) { motor_set(IN1_A2, IN2_A2, EN_A2, duty); }

void moduleA_stop() {
    stop_motor(IN1_A1, IN2_A1, EN_A1);
    stop_motor(IN1_A2, IN2_A2, EN_A2);
}

// ---------- MODULE B ----------
void moduleB_motor1(int duty) { motor_set(IN1_B1, IN2_B1, EN_B1, duty); }
void moduleB_motor2(int duty) { motor_set(IN1_B2, IN2_B2, EN_B2, duty); }

void moduleB_stop() {
    stop_motor(IN1_B1, IN2_B1, EN_B1);
    stop_motor(IN1_B2, IN2_B2, EN_B2);
}

// ============================================================
//  HIGH-LEVEL SWERVE COMMANDS
// ============================================================
//
//  duty ranges from -255 to +255
//
//  Differential swerve math:
//
//  wheel_speed  = (m1 - m2)
//  steering_yaw = (m1 + m2)
//
// ============================================================

// -------- MODULE A: Set wheel + steering simultaneously --------
void moduleA_set(int wheel_cmd, int yaw_cmd) {
    int m1 = yaw_cmd + wheel_cmd;
    int m2 = yaw_cmd - wheel_cmd;

    moduleA_motor1(m1);
    moduleA_motor2(m2);
}

// -------- MODULE B: Set wheel + steering simultaneously --------
void moduleB_set(int wheel_cmd, int yaw_cmd) {
    int m1 = yaw_cmd + wheel_cmd;
    int m2 = yaw_cmd - wheel_cmd;

    moduleB_motor1(m1);
    moduleB_motor2(m2);
}

// ============================================================
//   WHOLE ROBOT MOVEMENTS (2 modules together)
// ============================================================

// forward = both wheels translate forward
void robot_forward(int speed) {
    moduleA_set(+speed, 0);
    moduleB_set(+speed, 0);
}

// backward
void robot_backward(int speed) {
    moduleA_set(-speed, 0);
    moduleB_set(-speed, 0);
}

// rotate in place (turn left)
void robot_rotate_left(int yaw) {
    moduleA_set(0, +yaw);
    moduleB_set(0, +yaw);
}

// rotate in place (turn right)
void robot_rotate_right(int yaw) {
    moduleA_set(0, -yaw);
    moduleB_set(0, -yaw);
}

void robot_stop() {
    moduleA_stop();
    moduleB_stop();
}

// ============================================================
// MAIN TEST
// ============================================================

int main() {
    stdio_init_all();

    // Init GPIO
    int all_pins[] = {
        EN_A1, IN1_A1, IN2_A1,
        EN_A2, IN1_A2, IN2_A2,
        EN_B1, IN1_B1, IN2_B1,
        EN_B2, IN1_B2, IN2_B2
    };

    // Configure pins
    for (int i = 0; i < 12; i++) {
        gpio_init(all_pins[i]);
        gpio_set_dir(all_pins[i], GPIO_OUT);
    }

    // PWM for EN pins
    pwm_setup(EN_A1);
    pwm_setup(EN_A2);
    pwm_setup(EN_B1);
    pwm_setup(EN_B2);

    while (true) {

        // printf("Forward test\n");
        // robot_forward(250);
        // sleep_ms(15000);

        // robot_stop();
        // sleep_ms(700);

        printf("Rotate left\n");
        robot_rotate_left(500);
        sleep_ms(15000);

        robot_stop();
        sleep_ms(700);

        printf("Backward\n");
        robot_backward(500);
        sleep_ms(15000);

        // robot_stop();
        // sleep_ms(700);
    }
}
