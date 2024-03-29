#pragma once

#ifndef CARPIN_INCLUDED
#define CARPIN_INCLUDED

#include <driver/gpio.h>

#define    I2S_WS  GPIO_NUM_27
#define    I2S_SD  GPIO_NUM_33
#define    I2S_SCK GPIO_NUM_32

#define I2S_WS_SPEAK GPIO_NUM_13
#define I2S_SD_SPEAK GPIO_NUM_4
#define I2S_CLK_SPEAK GPIO_NUM_14 

#define    HCSR501 GPIO_NUM_35

#define    CAM_X   GPIO_NUM_18
#define    CAM_Y   GPIO_NUM_19

#define    STEPPER_DIR    GPIO_NUM_26

#define    SDA            GPIO_NUM_21
#define    SCL            GPIO_NUM_22

#define SPEAKER_EN GPIO_NUM_23

#define SDA_LCD GPIO_NUM_16
#define SCL_LCD GPIO_NUM_17

#define CONFIG_TFT_DISPLAY_WIDTH 480
#define CONFIG_TFT_DISPLAY_HEIGHT 320


#define I2S_NETWORK_PORT  4000
#define COMMAND_PORT      4001

#endif
