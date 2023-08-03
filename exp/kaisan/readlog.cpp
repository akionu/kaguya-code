#include <stdio.h>
extern "C" {
    #include "../../lib/kaguya-pin/kaguya-pin.h"
    #include "../../lib/pico-eeprom-i2c/src/eeprom.h"
}

#define READ_UNTIL 10000

uint8_t rbuf[500] = {0};
uint16_t head;
int main(void) {
    stdio_init_all();
    sleep_ms(1000);
    printf("read_log:\n");

    i2c_init(i2c1, 1000 * 1000);
    gpio_set_function(i2c1_scl_pin, GPIO_FUNC_I2C);
    gpio_set_function(i2c1_sda_pin, GPIO_FUNC_I2C);

    while (head < READ_UNTIL) {
        eeprom_read(i2c1, 0x50, head, rbuf, 500);
        for (int i = 0; i < 500; i++) {
            printf("%c", rbuf[i]);
        }
        head+=500;
    }
}