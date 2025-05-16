#include "pico/stdlib.h"
#include "BMP280.h"
#include <stdio.h>

int main() {
    stdio_init_all();

    BMP280 bmp(spi0, 17); // CS on GPIO17
    bmp.init();

    while (true) {
        float temp = bmp.readTemperature();
        float pressure = bmp.readPressure();
        printf("Temp: %.2f °C, Pressure: %.2f Pa\n", temp, pressure);
        sleep_ms(2000);
    }
}