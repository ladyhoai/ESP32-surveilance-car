#include <audio_handle.h>

const uint16_t audio_buffer_size = 512;
int16_t audio_buffer[audio_buffer_size];

size_t byte_read = 0;

void init_i2s() {
    // i2s_config_t i2s_conf = {
    //     .mode = i2s_mode_t(I2S_MODE_MASTER | I2S_MODE_RX),
    //     .sample_rate = 22000,
    //     .bits_per_sample = i2s_bits_per_sample_t(16),
    //     .channel_format = /*I2S_CHANNEL_FMT_RIGHT_LEFT,*/ I2S_CHANNEL_FMT_ONLY_LEFT,
    //     .communication_format = i2s_comm_format_t(I2S_COMM_FORMAT_I2S),
    //     .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    //     .dma_buf_count = 4,
    //     .dma_buf_len = 512,
    //     .use_apll = false,
    // };
    // i2s_driver_install(I2S_PORT, &i2s_conf, 0, NULL);
    
    // i2s_pin_config_t pin_conf = {
    //     .bck_io_num = I2S_SCK,
    //     .ws_io_num = I2S_WS,
    //     .data_out_num = -1,
    //     .data_in_num = I2S_SD,
    // };
    // i2s_set_pin(I2S_PORT, &pin_conf);
    
    i2s_config_t i2s_speaker = {
        .mode = i2s_mode_t(I2S_MODE_MASTER | I2S_MODE_TX),
        .sample_rate = 16000,
        .bits_per_sample = i2s_bits_per_sample_t(16),
        .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
        .communication_format = i2s_comm_format_t(I2S_COMM_FORMAT_I2S),
        .intr_alloc_flags = (ESP_INTR_FLAG_LOWMED | ESP_INTR_FLAG_SHARED | ESP_INTR_FLAG_INTRDISABLED),
        .dma_buf_count = 4,
        .dma_buf_len = 512,
        .use_apll = false,
        .tx_desc_auto_clear = true
    };
    i2s_driver_install(I2S_SPEAK, &i2s_speaker, 0, NULL);


    i2s_pin_config_t pin_speak = {
        .bck_io_num = I2S_CLK_SPEAK,
        .ws_io_num = I2S_WS_SPEAK,
        .data_out_num = I2S_SD_SPEAK,
        .data_in_num = -1,
    };
    i2s_set_pin(I2S_SPEAK, &pin_speak);

    //Set header for audio buffer
    //audio_buffer[0] = 0x7561; // the reverse of "au", represented as ASCII code in hexadecimal form
    // We reverse the byte because esp web socket client send byte as little endian (low byte, then high byte)
}

void stream_to_server(esp_websocket_client_handle_t audio) {
    // i2s_read, offset by 1 (First 2 bytes is for the header)
    i2s_read(I2S_PORT, audio_buffer, sizeof(audio_buffer), &byte_read, 10000 / portTICK_PERIOD_MS);
    //ESP_LOGI("Audio", "%d", byte_read);
    esp_websocket_client_send_bin(audio, (const char*) audio_buffer, sizeof(audio_buffer), 10000 / portTICK_PERIOD_MS);    
}

void output_to_speaker(const char* data, int len) {
    size_t byte_written = 0;
    i2s_write(I2S_SPEAK, data, len, &byte_written, 10000 / portTICK_PERIOD_MS);
    //i2s_write(I2S_SPEAK, 0x00, 1, &byte_written, 10000 / portTICK_PERIOD_MS);
    //ESP_LOGI("audio", "%d", byte_written);
}
