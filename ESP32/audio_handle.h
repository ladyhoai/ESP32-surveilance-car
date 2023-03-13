#pragma once
#ifndef I2S_INCLUDED
#define I2S_INCLUDED

#include <driver/i2s.h>
#include <car_pin.h>
#include <esp_websocket.h>

#define I2S_PORT I2S_NUM_0 
    
void init_i2s();
void stream_to_server(esp_websocket_client_handle_t audio);

#endif
