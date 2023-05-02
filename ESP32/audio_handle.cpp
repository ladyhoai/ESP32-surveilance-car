#include <audio_handle.h>

const uint16_t audio_buffer_size = 512;
uint8_t audio_buffer_asByte[2 * audio_buffer_size];
size_t byte_read = 0;

struct wave_header {
    char riff_tag[4]          {'R', 'I', 'F', 'F'};
    uint32_t riff_length      {0xFFFFFFFF};
    char wave_tag[4]          {'W', 'A', 'V', 'E'};
    char format_tag[4]        {'f', 'm', 't', ' '};
    uint32_t format_length    {16};
    uint16_t format_code      {1};
    uint16_t num_channel      {1};
    uint32_t sample_rate      {16000};
    uint32_t byte_rate        {32000};
    uint16_t block_align      {2};
    uint16_t bit_depth        {16};
    char data_tag[4]          {'d', 'a', 't', 'a'};
    uint32_t data_length      {0xFFFFFFFF};
} header_wav;

void init_i2s() {
    // Reading from INMP441 microphone
    i2s_config_t i2s_conf = {
        .mode = i2s_mode_t(I2S_MODE_MASTER | I2S_MODE_RX),
        .sample_rate = 22000,
        .bits_per_sample = i2s_bits_per_sample_t(16),
        .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT, //I2S_CHANNEL_FMT_ONLY_LEFT,
        .communication_format = i2s_comm_format_t(I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB),
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count = 4,
        .dma_buf_len = 512,
        .use_apll = false,
    };
    i2s_driver_install(I2S_PORT, &i2s_conf, 0, NULL);
    
    i2s_pin_config_t pin_conf = {
        .bck_io_num = I2S_SCK,
        .ws_io_num = I2S_WS,
        .data_out_num = -1,
        .data_in_num = I2S_SD,
    };
    i2s_set_pin(I2S_PORT, &pin_conf);
    
    // Outputting PCM data to the speaker
    i2s_config_t i2s_speaker = {
        .mode = i2s_mode_t(I2S_MODE_MASTER | I2S_MODE_TX),
        .sample_rate = 16000,
        .bits_per_sample = i2s_bits_per_sample_t(16),
        .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
        .communication_format = i2s_comm_format_t(I2S_COMM_FORMAT_I2S),
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count = 4,
        .dma_buf_len = 512,
        .use_apll = false,
    };
    i2s_driver_install(I2S_SPEAK, &i2s_speaker, 0, NULL);

    i2s_pin_config_t pin_speak = {
        .bck_io_num = I2S_CLK_SPEAK,
        .ws_io_num = I2S_WS_SPEAK,
        .data_out_num = I2S_SD_SPEAK,
        .data_in_num = -1,
    };
    i2s_set_pin(I2S_SPEAK, &pin_speak);
}

void stream_to_server(esp_websocket_client_handle_t audio) {
    uint16_t audio_buffer[audio_buffer_size];
    i2s_read(I2S_PORT, audio_buffer, audio_buffer_size, &byte_read, 10000 / portTICK_PERIOD_MS);
    esp_websocket_client_send_bin(audio, (const char*) audio_buffer, audio_buffer_size, 10000/portTICK_PERIOD_MS);
}

void output_to_speaker(const char* data, int len) {
    size_t byte_written = 0;
    i2s_write(I2S_SPEAK, data, len, &byte_written, 1000 / portTICK_PERIOD_MS);
}
