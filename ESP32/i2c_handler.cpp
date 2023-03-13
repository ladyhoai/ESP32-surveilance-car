#include <i2c_handler.h>

i2c_config_t i2c_conf = {};

int I2C_clearBus(gpio_num_t sda, gpio_num_t scl) {
    #if defined(TWCR) && defined(TWEN)
        TWCR &= ~(_BV(TWEN));
    #endif

    gpio_set_direction(sda, GPIO_MODE_INPUT);
    gpio_pullup_en(sda);
    gpio_set_direction(scl, GPIO_MODE_INPUT);
    gpio_pullup_en(scl);

    bool SCL_LOW = (gpio_get_level(scl) == 0);
    if (SCL_LOW) {
        return 1;
    }

    bool SDA_LOW = (gpio_get_level(sda) == 0);
    int clockCount = 20;

    while (SDA_LOW && clockCount > 0) {
        clockCount--;
        gpio_pullup_dis(scl);
        gpio_set_direction(scl, GPIO_MODE_OUTPUT);
        vTaskDelay(1 / portTICK_PERIOD_MS);
        gpio_set_direction(scl, GPIO_MODE_INPUT);
        gpio_pullup_en(scl);

        vTaskDelay(1 / portTICK_PERIOD_MS);
        SCL_LOW = (gpio_get_level(scl) == 0);
        int counter = 20;

        while (SCL_LOW && counter > 0) {
            counter--;
            vTaskDelay(100 /portTICK_PERIOD_MS);
            SCL_LOW = (gpio_get_level(scl) == 0);
        }

        if (SCL_LOW) {
            gpio_set_direction(scl, GPIO_MODE_INPUT);
            gpio_pullup_en(scl);
            gpio_set_direction(sda, GPIO_MODE_INPUT);
            gpio_pullup_en(sda);
            return 2;
        }
        SDA_LOW = (gpio_get_level(sda) == 0);
    }

    if (SDA_LOW) {
        gpio_set_direction(scl, GPIO_MODE_INPUT);
        gpio_pullup_en(scl);
        gpio_set_direction(sda, GPIO_MODE_INPUT);
        gpio_pullup_en(sda);
        return 3;
    }

    gpio_pullup_dis(sda);
    gpio_set_direction(sda, GPIO_MODE_OUTPUT);

    vTaskDelay(1 / portTICK_PERIOD_MS);
    gpio_set_direction(sda, GPIO_MODE_INPUT);
    gpio_pullup_en(sda);
    vTaskDelay(1 / portTICK_PERIOD_MS);
    gpio_pullup_en(sda);
    gpio_pullup_en(scl);

    return 0;
}

void init_i2c() {
 i2c_conf = {
    .mode = I2C_MODE_MASTER,
    .sda_io_num = SDA,
    .scl_io_num = SCL,
    .sda_pullup_en = false,
    .scl_pullup_en = false, 
};
i2c_conf.master.clk_speed = 100000;

i2c_driver_install(I2C_PORT, i2c_conf.mode, 0, 0, 0);
i2c_param_config(I2C_PORT, &i2c_conf);
}

void send_command_to_uno(uint8_t byte, uint8_t slave_address, int num_bytes) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ((slave_address << 1) | I2C_MASTER_WRITE), true);
    i2c_master_write_byte(cmd, byte, true);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_PORT, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
}

void received_data_from_uno(uint8_t* data_buffer, uint8_t slave_address, uint8_t num_byte) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ((slave_address << 1) | I2C_MASTER_READ), true);
    i2c_master_read(cmd, data_buffer, num_byte, I2C_MASTER_ACK);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_PORT, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
}
