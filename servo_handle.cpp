// Copyright 2020 Espressif Systems (Shanghai) Co. Ltd.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "esp_log.h"
#include "esp_err.h"
#include "driver/ledc.h"
#include "servo_handle.h"

static const char *TAG = "servo";

#define SERVO_CHECK(a, str, ret_val) \
    if (!(a)) { \
        ESP_LOGE(TAG,"%s(%d): %s", __FUNCTION__, __LINE__, str); \
        return (ret_val); \
    }

#define SERVO_LEDC_INIT_BITS LEDC_TIMER_10_BIT
#define SERVO_FREQ_MIN       50
#define SERVO_FREQ_MAX       400

static uint32_t g_full_duty = 0;
static servo_config_t g_cfg[LEDC_SPEED_MODE_MAX] = {0};

static uint32_t calculate_duty(ledc_mode_t speed_mode, float angle)
{
    float angle_us = angle / g_cfg[speed_mode].max_angle * (g_cfg[speed_mode].max_width_us - g_cfg[speed_mode].min_width_us) + g_cfg[speed_mode].min_width_us;
    ESP_LOGD(TAG, "angle us: %f", angle_us);
    uint32_t duty = (uint32_t)((float)g_full_duty * (angle_us) * g_cfg[speed_mode].freq / (1000000.0f));
    return duty;
}

static float calculate_angle(ledc_mode_t speed_mode, uint32_t duty)
{
    float angle_us = (float)duty * 1000000.0f / (float)g_full_duty / (float)g_cfg[speed_mode].freq;
    angle_us -= g_cfg[speed_mode].min_width_us;
    angle_us = angle_us < 0.0f ? 0.0f : angle_us;
    float angle = angle_us * g_cfg[speed_mode].max_angle / (g_cfg[speed_mode].max_width_us - g_cfg[speed_mode].min_width_us);
    return angle;
}

esp_err_t iot_servo_init(ledc_mode_t speed_mode, const servo_config_t *config)
{
    esp_err_t ret;
    SERVO_CHECK(NULL != config, "Pointer of config is invalid", ESP_ERR_INVALID_ARG);
    SERVO_CHECK(config->channel_number > 0 && config->channel_number <= LEDC_CHANNEL_MAX, "Servo channel number out the range", ESP_ERR_INVALID_ARG);
    SERVO_CHECK(config->freq <= SERVO_FREQ_MAX && config->freq >= SERVO_FREQ_MIN, "Servo pwm frequency out the range", ESP_ERR_INVALID_ARG);
    uint64_t pin_mask = 0;
    uint32_t ch_mask = 0;
    for (size_t i = 0; i < config->channel_number; i++) {
        uint64_t _pin_mask = 1ULL << config->channels.servo_pin[i];
        uint32_t _ch_mask = 1UL << config->channels.ch[i];
        SERVO_CHECK(!(pin_mask & _pin_mask), "servo gpio has a duplicate", ESP_ERR_INVALID_ARG);
        SERVO_CHECK(!(ch_mask & _ch_mask), "servo channel has a duplicate", ESP_ERR_INVALID_ARG);
        SERVO_CHECK(GPIO_IS_VALID_OUTPUT_GPIO(config->channels.servo_pin[i]), "servo gpio invalid", ESP_ERR_INVALID_ARG);
        pin_mask |= _pin_mask;
        ch_mask |= _ch_mask;
    }

    ledc_timer_config_t ledc_timer = {
        .speed_mode = speed_mode,            // timer mode
        .duty_resolution = SERVO_LEDC_INIT_BITS,     // resolution of PWM duty
        .timer_num = config->timer_number,            // timer index
        .freq_hz = config->freq,                     // frequency of PWM signal
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ret = ledc_timer_config(&ledc_timer);
    SERVO_CHECK(ESP_OK == ret, "ledc timer configuration failed", ESP_FAIL);
    for (size_t i = 0; i < config->channel_number; i++) {
        ledc_channel_config_t ledc_ch = {
            .gpio_num   = config->channels.servo_pin[i],
            .speed_mode = speed_mode,
            .channel    = config->channels.ch[i],
            .intr_type  = LEDC_INTR_DISABLE,
            .timer_sel  = config->timer_number,
            .duty       = calculate_duty(speed_mode, 0),
            .hpoint     = 0
        };
        ret = ledc_channel_config(&ledc_ch);
        SERVO_CHECK(ESP_OK == ret, "ledc channel configuration failed", ESP_FAIL);
    }
    g_full_duty = (1 << SERVO_LEDC_INIT_BITS) - 1;
    g_cfg[speed_mode] = *config;

    return ESP_OK;
}

esp_err_t iot_servo_deinit(ledc_mode_t speed_mode)
{
    SERVO_CHECK(speed_mode < LEDC_SPEED_MODE_MAX, "LEDC speed mode invalid", ESP_ERR_INVALID_ARG);
    for (size_t i = 0; i < g_cfg[speed_mode].channel_number; i++) {
        ledc_stop(speed_mode, g_cfg[speed_mode].channels.ch[i], 0);
    }
    ledc_timer_rst(speed_mode, g_cfg[speed_mode].timer_number);
    g_full_duty = 0;
    return ESP_OK;
}

esp_err_t iot_servo_write_angle(ledc_mode_t speed_mode, uint8_t channel, float angle)
{
    SERVO_CHECK(speed_mode < LEDC_SPEED_MODE_MAX, "LEDC speed mode invalid", ESP_ERR_INVALID_ARG);
    SERVO_CHECK(channel < LEDC_CHANNEL_MAX, "LEDC channel number too large", ESP_ERR_INVALID_ARG);
    SERVO_CHECK(angle >= 0.0f, "Angle can't to be negative", ESP_ERR_INVALID_ARG);
    esp_err_t ret;
    uint32_t duty = calculate_duty(speed_mode, angle);
    ret = ledc_set_duty(speed_mode, (ledc_channel_t)channel, duty);
    ret |= ledc_update_duty(speed_mode, (ledc_channel_t)channel);
    SERVO_CHECK(ESP_OK == ret, "write servo angle failed", ESP_FAIL);
    return ESP_OK;
}

esp_err_t iot_servo_read_angle(ledc_mode_t speed_mode, uint8_t channel, float *angle)
{
    SERVO_CHECK(speed_mode < LEDC_SPEED_MODE_MAX, "LEDC speed mode invalid", ESP_ERR_INVALID_ARG);
    SERVO_CHECK(channel < LEDC_CHANNEL_MAX, "LEDC channel number too large", ESP_ERR_INVALID_ARG);
    uint32_t duty = ledc_get_duty(speed_mode, (ledc_channel_t)channel);
    float a = calculate_angle(speed_mode, duty);
    *angle = a;
    return ESP_OK;
}

esp_err_t camera_axis_init(gpio_num_t pin1, gpio_num_t pin2, ledc_channel_t pin1_chan, ledc_channel_t pin2_chan) {
    servo_channel_t servo_chan;
    servo_chan.servo_pin[0] = pin1;
    servo_chan.servo_pin[1] = pin2;
    servo_chan.ch[0] = pin1_chan;
    servo_chan.ch[1] = pin2_chan;

    servo_config_t cam_conf = {
        .max_angle = 180,
        .min_width_us = 500,
        .max_width_us = 2500,
        .freq = 50,
        .timer_number = LEDC_TIMER_0,
        .channels = servo_chan,
        .channel_number = 2,
    };

    iot_servo_init(LEDC_LOW_SPEED_MODE, &cam_conf);
    return ESP_OK;
}

// #include <servo_handle.h>

// inline uint32_t camera_axis::angle_to_compare(int angle) {
//     return (angle - SERVO_MIN_ANGLE) * (SERVO_MAX_PULSEWIDTH_US - SERVO_MIN_PULSEWIDTH_US) / 
//         (SERVO_MAX_ANGLE - SERVO_MIN_ANGLE) + SERVO_MIN_PULSEWIDTH_US;
// }

// camera_axis::camera_axis(gpio_num_t pin) { 
//     //Creating timer for PWM servo
//     camera_servo_timer_conf = {
//         .group_id = 0, // try incrementing this if doesn't work
//         .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
//         .resolution_hz = SERVO_TIMEBASE_RESOLUTION_HZ,
//         .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
//         .period_ticks = SERVO_TIMEBASE_PERIOD,
//     };
//     ESP_ERROR_CHECK(mcpwm_new_timer(&camera_servo_timer_conf, &camera_servo_timer));

//     camera_servo_operator_config = {
//         .group_id = 0,
//     };

//     ESP_ERROR_CHECK(mcpwm_new_operator(&camera_servo_operator_config, &camera_servo_oper));

//     // Connecting timer and operator

//     ESP_ERROR_CHECK(mcpwm_operator_connect_timer(camera_servo_oper, camera_servo_timer));

//     // Creating comparator and generator from the operator
//     camera_servo_comparator_config.flags.update_cmp_on_tez = true;

//     ESP_ERROR_CHECK(mcpwm_new_comparator(camera_servo_oper, &camera_servo_comparator_config, &camera_servo_comparator));
//     ESP_LOGI("Servo", "comparator successful init");
//     // Generating a pulse on pin 19, camera x axis
//     camera_servo_gen_config = {
//         .gen_gpio_num = pin,
//     };
//     ESP_LOGI("Servo", "Pin successfully assigned");
//     ESP_ERROR_CHECK(mcpwm_new_generator(camera_servo_oper, &camera_servo_gen_config, &camera_servo_gen_handle));
//     ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(camera_servo_comparator, angle_to_compare(cur_angle)));
//     ESP_LOGI("Servo", "compare value set");
//     // Set generator action on timer and compare event

//     // Go high on counter empty
//     ESP_ERROR_CHECK(mcpwm_generator_set_actions_on_timer_event(camera_servo_gen_handle, 
//     MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
//     ESP_LOGI("Servo", "set actions on timer event done");
//     // Go low on compare threshold
//     ESP_ERROR_CHECK(mcpwm_generator_set_actions_on_compare_event(camera_servo_gen_handle, 
//     MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, camera_servo_comparator, MCPWM_GEN_ACTION_LOW)));

//     // Enable and start timer
//     ESP_ERROR_CHECK(mcpwm_timer_enable(camera_servo_timer));
//     ESP_ERROR_CHECK(mcpwm_timer_start_stop(camera_servo_timer, MCPWM_TIMER_START_NO_STOP));
//     ESP_LOGI("Servo", "Timer started");
// }

// void camera_axis::cam_rotate(bool clockwise) {
//     if (clockwise) {
//         if (cur_angle + step <= 90) {
//         ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(camera_servo_comparator, angle_to_compare(cur_angle + step)));
//         cur_angle += step;
//         }
//     }
//     else {
//         if (cur_angle - step >= -90) {
//             ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(camera_servo_comparator, angle_to_compare(cur_angle - step)));
//             cur_angle -= step;
//         }
//     }
// }