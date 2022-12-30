#include "pins_arduino.h"
#include "Arduino.h"
#include "driver/i2c.h"

#define I2C_NUM (0)
#define I2C_TIMEOUT_MS (100)

void I2CSetup() {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = SDA,  // select GPIO specific to your project
        .scl_io_num = SCL,  // select GPIO specific to your project
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master = {
            .clk_speed = 100000UL,  // select frequency specific to your project
        },
        .clk_flags = 0,  // you can use I2C_SCLK_SRC_FLAG_* flags to choose i2c source clock here
    };

    i2c_param_config(I2C_NUM, &conf);
    i2c_driver_install(I2C_NUM, conf.mode, 0, 0, 0);
}

int I2CWrite(uint8_t addr, uint8_t *data, uint8_t len, bool stop) {
    return i2c_master_write_to_device(I2C_NUM, addr, data, len, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
}

int I2CRead(uint8_t addr, uint8_t *writeData, uint8_t writeLen, uint8_t *readData, uint8_t readLen) {
    if (writeLen > 0) {
        return i2c_master_write_read_device(I2C_NUM, addr, writeData, writeLen, readData, readLen, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
    }

    return i2c_master_read_from_device(I2C_NUM, addr, readData, readLen, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
}

int I2CMemWrite(uint8_t addr, uint8_t reg, uint8_t *data, uint8_t len) {
    uint8_t write_buf[len + 1];
    write_buf[0] = reg;
    memcpy(&write_buf[1], data, len);
    return i2c_master_write_to_device(I2C_NUM, addr, write_buf, len + 1, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
}

int I2CMemRead(uint8_t addr, uint8_t reg, uint8_t *data, uint8_t len) {
    return i2c_master_write_read_device(I2C_NUM, addr, &reg, 1, data, len, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
}

int I2CMemWrite(uint8_t addr, uint8_t reg, uint8_t data) {
    return I2CMemWrite(addr, reg, &data, 1);
}

int I2CMemRead(uint8_t addr, uint8_t reg, uint8_t *data) {
    return I2CMemRead(addr, reg, data, 1);
}

int I2CAddressCStringToInt(const char *str) {
    unsigned int addr = 0;
    if (str && strlen(str) > 2) {
        sscanf(str, "0x%X", &addr);
    }
    return addr;
}

bool CheckI2CDevice(int addr) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_NUM, cmd, 50 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret == ESP_OK;
}
