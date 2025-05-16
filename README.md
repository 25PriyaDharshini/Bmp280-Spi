#ifndef BMP280_H
#define BMP280_H

#include "hardware/spi.h"
#include "pico/stdlib.h"
#include <stdint.h>

class BMP280 {
public:
    BMP280(spi_inst_t* spi_port, uint cs_pin);
    void init();
    float readTemperature();
    float readPressure();

private:
    spi_inst_t* spi_port;
    uint cs_pin;

    void writeRegister(uint8_t reg, uint8_t value);
    uint8_t readRegister(uint8_t reg);
    void readCalibrationData();
    uint16_t read16(uint8_t reg);
    uint32_t read24(uint8_t reg);

    // Calibration coefficients
    uint16_t dig_T1;
    int16_t dig_T2, dig_T3;
    uint16_t dig_P1;
    int16_t dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;

    int32_t t_fine;
};

#endif
