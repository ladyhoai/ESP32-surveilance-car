// Copyright 2020-2021 Espressif Systems (Shanghai) Co. Ltd.
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

#ifndef _IOT_SERVO_H_
#define _IOT_SERVO_H_

#include "esp_err.h"
#include "driver/ledc.h"
#include "driver/gpio.h"

/**
 * @brief Configuration of servo motor channel
 * 
 */
typedef struct {
    gpio_num_t servo_pin[LEDC_CHANNEL_MAX];     /**< Pin number of pwm output */
    ledc_channel_t ch[LEDC_CHANNEL_MAX];    /**< The ledc channel which used */
} servo_channel_t;

/**
 * @brief Configuration of servo motor
 * 
 */
typedef struct {
    uint16_t max_angle;        /**< Servo max angle */
    uint16_t min_width_us;     /**< Pulse width corresponding to minimum angle, which is usually 500us */
    uint16_t max_width_us;     /**< Pulse width corresponding to maximum angle, which is usually 2500us */
    uint32_t freq;             /**< PWM frequency */
    ledc_timer_t timer_number; /**< Timer number of ledc */
    servo_channel_t channels;  /**< Channels to use */
    uint8_t channel_number;    /**< Total channel number */
} servo_config_t;

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize ledc to control the servo
 * 
 * @param speed_mode Select the LEDC channel group with specified speed mode. Note that not all targets support high speed mode.
 * @param config Pointer of servo configure struct
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_ERR_INVALID_ARG Parameter error
 *     - ESP_FAIL Configure ledc failed
 */
esp_err_t iot_servo_init(ledc_mode_t speed_mode, const servo_config_t *config);

/**
 * @brief Deinitialize ledc for servo
 * 
 * @param speed_mode Select the LEDC channel group with specified speed mode.
 * 
 * @return
 *     - ESP_OK Success
 */
esp_err_t iot_servo_deinit(ledc_mode_t speed_mode);

/**
 * @brief Set the servo motor to a certain angle
 * 
 * @note This API is not thread-safe
 * 
 * @param speed_mode Select the LEDC channel group with specified speed mode.
 * @param channel LEDC channel, select from ledc_channel_t
 * @param angle The angle to go
 * 
 * @return
 *     - ESP_OK Success
 *     - ESP_ERR_INVALID_ARG Parameter error
 */
esp_err_t iot_servo_write_angle(ledc_mode_t speed_mode, uint8_t channel, float angle);

/**
 * @brief Read current angle of one channel 
 * 
 * @param speed_mode Select the LEDC channel group with specified speed mode.
 * @param channel LEDC channel, select from ledc_channel_t
 * @param angle Current angle of the channel
 * 
 * @return
 *     - ESP_OK Success
 *     - ESP_ERR_INVALID_ARG Parameter error
 */
esp_err_t iot_servo_read_angle(ledc_mode_t speed_mode, uint8_t channel, float *angle);

esp_err_t camera_axis_init(gpio_num_t pin1, gpio_num_t pin2, ledc_channel_t pin1_chan, ledc_channel_t pin2_chan);
#ifdef __cplusplus
}
#endif

#endif /* _IOT_SERVO_H_ */


// #pragma once
// #include <driver/mcpwm_prelude.h>
// #include <driver/gpio.h>
// #include <esp_log.h>
// #include <car_pin.h>

// #define SERVO_MIN_PULSEWIDTH_US       500 // Minimum pulse width im microseconds
// #define SERVO_MAX_PULSEWIDTH_US       2500 // Maximum pulse width for servo
// #define SERVO_MIN_ANGLE              -90
// #define SERVO_MAX_ANGLE               90
// #define SERVO_TIMEBASE_RESOLUTION_HZ  1000000 //1Mz, 1us per tick
// #define SERVO_TIMEBASE_PERIOD         20000   // 20000 ticks, 20ms

// class camera_axis {
//     private:
//     int step = 3;
//     int cur_angle = 0;

//     mcpwm_timer_handle_t camera_servo_timer = NULL;
//     mcpwm_timer_config_t camera_servo_timer_conf = {};
//     mcpwm_oper_handle_t camera_servo_oper = NULL;
//     mcpwm_operator_config_t camera_servo_operator_config = {};
//     mcpwm_cmpr_handle_t camera_servo_comparator = NULL;
//     mcpwm_comparator_config_t camera_servo_comparator_config = {};
//     mcpwm_gen_handle_t camera_servo_gen_handle = NULL;
//     mcpwm_generator_config_t camera_servo_gen_config = {};

//     static inline uint32_t angle_to_compare(int angle);
//     public:

//     camera_axis(gpio_num_t pin);
//     void cam_rotate(bool clockwise);
// };