#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "../../lib/pico-eeprom-i2c/src/eeprom.h"

#define I2C_SDA 2
#define I2C_SCL 3

uint32_t hlog = 0;

uint8_t addr_eeprom = 0x50;

int main(void) {
    stdio_init_all();
    sleep_ms(3000);
    printf("\n\nhello, this is pico!\n");

    i2c_init(i2c1, 400 * 1000);
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);

    static uint8_t rbuf[500] = {'0'};
    uint8_t abuf[] = {0x00, 0x00};
    int16_t i;
    int16_t ret;

    while (hlog < 64000) {
        ret = eeprom_read(i2c1, addr_eeprom, hlog, rbuf, 500);
        hlog += 500;
        for (i = 0; i < 500; i++) printf("%x", rbuf[i]);
        sleep_ms(100);
    }
    
    printf("ret: %d\n", ret);
}
