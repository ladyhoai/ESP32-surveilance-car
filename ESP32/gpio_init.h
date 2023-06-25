#pragma once
#ifndef GPIO_INCLUDED
#define GPIO_INCLUDED

#include <car_pin.h>

// I don't know whether we need to initialize sda and scl pin
#define OUTPUT_PIN ( (1ULL << SPEAKER_EN) |  (1ULL << STEPPER_EN) | (1ULL << STEPPER_DIR) /*| (1ULL << CAM_X)*/ | (1ULL << 2))

void init_gpio() {
gpio_config_t io_conf = {
    .pin_bit_mask = OUTPUT_PIN,
    .mode = GPIO_MODE_OUTPUT,
    .pull_up_en = GPIO_PULLUP_DISABLE,
    .pull_down_en = GPIO_PULLDOWN_DISABLE,
    .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);
}

#endif
