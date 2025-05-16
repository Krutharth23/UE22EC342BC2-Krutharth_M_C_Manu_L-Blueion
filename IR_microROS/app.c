#include <stdio.h>
#include <unistd.h>
#include <inttypes.h>
#include <math.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/int32.h>

#include <driver/gpio.h>
#include <driver/ledc.h>

#ifdef ESP_PLATFORM
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#endif

// Macro functions
#define constrain(amt, low, high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc);vTaskDelete(NULL);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}

// Constants
#define FRAME_TIME 100 // 1000 / FRAME_TIME = FPS
#define SLEEP_TIME 10

// Motor PINS
#define LED_BUILTIN 33
#define PIN_LEFT_FORWARD 12
#define PIN_LEFT_BACKWARD 13
#define PIN_RIGHT_FORWARD 27
#define PIN_RIGHT_BACKWARD 14

// IR sensor PINS
#define IR1_PIN 18  // MSB
#define IR2_PIN 4   
#define IR3_PIN 5   // LSB

// PWM Channels (Reserve channel 0 and 1 for camera)
#define PWM_LEFT_FORWARD LEDC_CHANNEL_2
#define PWM_LEFT_BACKWARD LEDC_CHANNEL_3
#define PWM_RIGHT_FORWARD LEDC_CHANNEL_4
#define PWM_RIGHT_BACKWARD LEDC_CHANNEL_5

// Other PWM settings
#define PWM_FREQUENCY 50
#define PWM_RESOLUTION LEDC_TIMER_12_BIT
#define PWM_TIMER LEDC_TIMER_1
#define PWM_MODE LEDC_HIGH_SPEED_MODE

// Motor PWM range
#define PWM_MOTOR_MIN 750
#define PWM_MOTOR_MAX 4095

// Message containers
geometry_msgs__msg__Twist msg;
std_msgs__msg__Int32 int_msg;

// Function declarations
void setupPins();
void setupRos();
void cmd_vel_callback(const void *msgin);
void timer_callback(rcl_timer_t *timer, int64_t last_call_time);
void pub_timer_callback(rcl_timer_t *timer, int64_t last_call_time);
float fmap(float val, float in_min, float in_max, float out_min, float out_max);

// Publisher globals
rcl_publisher_t int_publisher;
rcl_timer_t pub_timer;

void appMain(void *arg) {
    setupPins();
    setupRos();
}

void setupPins() {
    // Built-in LED
    gpio_reset_pin(LED_BUILTIN);
    gpio_set_direction(LED_BUILTIN, GPIO_MODE_INPUT_OUTPUT);

    // IR sensor pins
    gpio_set_direction(IR1_PIN, GPIO_MODE_INPUT);
    gpio_set_direction(IR2_PIN, GPIO_MODE_INPUT);
    gpio_set_direction(IR3_PIN, GPIO_MODE_INPUT);

    // PWM timer
    ledc_timer_config_t ledc_timer = {
        .duty_resolution = PWM_RESOLUTION,
        .freq_hz = PWM_FREQUENCY,
        .speed_mode = PWM_MODE,
        .timer_num = PWM_TIMER,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ledc_timer_config(&ledc_timer);

    // PWM channels
    ledc_channel_config_t ledc_channel[4] = {
        {.channel = PWM_LEFT_FORWARD, .duty = 0, .gpio_num = PIN_LEFT_FORWARD, .speed_mode = PWM_MODE, .hpoint = 0, .timer_sel = PWM_TIMER},
        {.channel = PWM_LEFT_BACKWARD, .duty = 0, .gpio_num = PIN_LEFT_BACKWARD, .speed_mode = PWM_MODE, .hpoint = 0, .timer_sel = PWM_TIMER},
        {.channel = PWM_RIGHT_FORWARD, .duty = 0, .gpio_num = PIN_RIGHT_FORWARD, .speed_mode = PWM_MODE, .hpoint = 0, .timer_sel = PWM_TIMER},
        {.channel = PWM_RIGHT_BACKWARD, .duty = 0, .gpio_num = PIN_RIGHT_BACKWARD, .speed_mode = PWM_MODE, .hpoint = 0, .timer_sel = PWM_TIMER},
    };

    for (int i = 0; i < 4; i++) {
        ledc_channel_config(&ledc_channel[i]);
    }
}

void setupRos() {
    // Micro-ROS
    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;

    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

    rcl_node_t node;
    RCCHECK(rclc_node_init_default(&node, "IR_microROS", "", &support));

    // Subscriber
    rcl_subscription_t subscriber;
    RCCHECK(rclc_subscription_init_default(
        &subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "/cmd_vel"));

    // Motor control timer
    rcl_timer_t timer;
    RCCHECK(rclc_timer_init_default(
        &timer,
        &support,
        RCL_MS_TO_NS(FRAME_TIME),
        timer_callback));

    // IR publisher
    RCCHECK(rclc_publisher_init_default(
        &int_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "/IR_vals"));

    // IR sensor timer
    RCCHECK(rclc_timer_init_default(
        &pub_timer,
        &support,
        RCL_MS_TO_NS(100),  // 1 second
        pub_timer_callback));

    // Executor
    rclc_executor_t executor;
    RCCHECK(rclc_executor_init(&executor, &support.context, 3, &allocator));
    RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &cmd_vel_callback, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_timer(&executor, &timer));
    RCCHECK(rclc_executor_add_timer(&executor, &pub_timer));

    int_msg.data = 0;

    while (1) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(SLEEP_TIME));
        usleep(SLEEP_TIME * 100);
    }

    // Cleanup (will never reach here)
    RCCHECK(rcl_subscription_fini(&subscriber, &node));
    RCCHECK(rcl_node_fini(&node));
    vTaskDelete(NULL);
}

void cmd_vel_callback(const void *msgin) {
    // Not needed — msg is global
}

void timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
    if (timer == NULL) return;
    gpio_set_level(LED_BUILTIN, !gpio_get_level(LED_BUILTIN));

    float linear = constrain(msg.linear.x, -1, 1);
    float angular = constrain(msg.angular.z, -1, 1);
    float left = (linear - angular) / 2.0f;
    float right = (linear + angular) / 2.0f;

    uint16_t pwmLeft = (uint16_t) fmap(fabs(left), 0, 1, PWM_MOTOR_MIN, PWM_MOTOR_MAX);
    uint16_t pwmRight = (uint16_t) fmap(fabs(right), 0, 1, PWM_MOTOR_MIN, PWM_MOTOR_MAX);

    ledc_set_duty(PWM_MODE, PWM_LEFT_FORWARD, pwmLeft * (left > 0));
    ledc_set_duty(PWM_MODE, PWM_LEFT_BACKWARD, pwmLeft * (left < 0));
    ledc_set_duty(PWM_MODE, PWM_RIGHT_FORWARD, pwmRight * (right > 0));
    ledc_set_duty(PWM_MODE, PWM_RIGHT_BACKWARD, pwmRight * (right < 0));

    ledc_update_duty(PWM_MODE, PWM_LEFT_FORWARD);
    ledc_update_duty(PWM_MODE, PWM_LEFT_BACKWARD);
    ledc_update_duty(PWM_MODE, PWM_RIGHT_FORWARD);
    ledc_update_duty(PWM_MODE, PWM_RIGHT_BACKWARD);

    printf("%d, %d %d, %d, %d %d, %f, %f\n", pwmLeft, left > 0, left < 0, pwmRight, right > 0, right < 0, left, right);
}

void pub_timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
    if (timer == NULL) return;

    int ir1 = !gpio_get_level(IR1_PIN);
    int ir2 = !gpio_get_level(IR2_PIN);
    int ir3 = !gpio_get_level(IR3_PIN);

    int_msg.data = ir1*100 + ir2*10 + ir3;

    RCSOFTCHECK(rcl_publish(&int_publisher, &int_msg, NULL));
    printf("Published IR value: %d (bin: %d%d%d)\n", int_msg.data, ir1, ir2, ir3);
}

float fmap(float val, float in_min, float in_max, float out_min, float out_max) {
    return (val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

