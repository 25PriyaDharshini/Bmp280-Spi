#include "BMP280.h"
#include <stdio.h>

BMP280::BMP280(spi_inst_t* spi_port, uint cs_pin) {
    this->spi_port = spi_port;
    this->cs_pin = cs_pin;
}

void BMP280::init() {
    spi_init(spi_port, 1000 * 1000); // 1 MHz
    gpio_set_function(18, GPIO_FUNC_SPI);
    gpio_set_function(19, GPIO_FUNC_SPI);
    gpio_set_function(16, GPIO_FUNC_SPI);

    gpio_init(cs_pin);
    gpio_set_dir(cs_pin, GPIO_OUT);
    gpio_put(cs_pin, 1); // CS inactive

    // Soft reset
    writeRegister(0xE0, 0xB6);
    sleep_ms(100);

    // Read calibration data
    readCalibrationData();

    // Set mode and config
    writeRegister(0xF4, 0x27); // Temp and Pressure oversampling, Normal mode
    writeRegister(0xF5, 0xA0); // Standby and filter
}

void BMP280::writeRegister(uint8_t reg, uint8_t value) {
    gpio_put(cs_pin, 0);
    uint8_t buf[2] = { reg & 0x7F, value };
    spi_write_blocking(spi_port, buf, 2);
    gpio_put(cs_pin, 1);
}

uint8_t BMP280::readRegister(uint8_t reg) {
    gpio_put(cs_pin, 0);
    uint8_t tx = reg | 0x80;
    uint8_t rx[2];
    spi_write_read_blocking(spi_port, &tx, rx, 2);
    gpio_put(cs_pin, 1);
    return rx[1];
}

uint16_t BMP280::read16(uint8_t reg) {
    uint8_t msb = readRegister(reg);
    uint8_t lsb = readRegister(reg + 1);
    return (msb << 8) | lsb;
}

uint32_t BMP280::read24(uint8_t reg) {
    uint8_t msb = readRegister(reg);
    uint8_t lsb = readRegister(reg + 1);
    uint8_t xlsb = readRegister(reg + 2);
    return ((uint32_t)msb << 16) | ((uint32_t)lsb << 8) | xlsb;
}

void BMP280::readCalibrationData() {
    dig_T1 = read16(0x88);
    dig_T2 = (int16_t)read16(0x8A);
    dig_T3 = (int16_t)read16(0x8C);
    dig_P1 = read16(0x8E);
    dig_P2 = (int16_t)read16(0x90);
    dig_P3 = (int16_t)read16(0x92);
    dig_P4 = (int16_t)read16(0x94);
    dig_P5 = (int16_t)read16(0x96);
    dig_P6 = (int16_t)read16(0x98);
    dig_P7 = (int16_t)read16(0x9A);
    dig_P8 = (int16_t)read16(0x9C);
    dig_P9 = (int16_t)read16(0x9E);
}

float BMP280::readTemperature() {
    int32_t adc_T = read24(0xFA) >> 4;
    int32_t var1 = ((((adc_T >> 3) - ((int32_t)dig_T1 << 1))) * ((int32_t)dig_T2)) >> 11;
    int32_t var2 = (((((adc_T >> 4) - ((int32_t)dig_T1)) *
                     ((adc_T >> 4) - ((int32_t)dig_T1))) >> 12) *
                   ((int32_t)dig_T3)) >> 14;
    t_fine = var1 + var2;
    float T = (t_fine * 5 + 128) >> 8;
    return T / 100.0;
}

float BMP280::readPressure() {
    int32_t adc_P = read24(0xF7) >> 4;

    int64_t var1 = ((int64_t)t_fine) - 128000;
    int64_t var2 = var1 * var1 * (int64_t)dig_P6;
    var2 = var2 + ((var1 * (int64_t)dig_P5) << 17);
    var2 = var2 + (((int64_t)dig_P4) << 35);
    var1 = ((var1 * var1 * (int64_t)dig_P3) >> 8) +
           ((var1 * (int64_t)dig_P2) << 12);
    var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)dig_P1) >> 33;

    if (var1 == 0) return 0; // avoid div by zero

    int64_t p = 1048576 - adc_P;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = (((int64_t)dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((int64_t)dig_P8) * p) >> 19;

    p = ((p + var1 + var2) >> 8) + (((int64_t)dig_P7) << 4);
    return (float)p / 256;
}