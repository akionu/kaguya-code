#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "../../lib/pico-eeprom-i2c/src/eeprom.h"
#include "../../lib/kaguya-pin/kaguya-pin.h"

#define MAX_INDEX 20000
// eeprom
uint8_t eeprom_addr = 0x50;
uint32_t head = 0;

uint8_t buf[MAX_INDEX];

int main (void){
    stdio_init_all();
    sleep_ms(2000);
    printf("hello, this is parachute_read!\n");

    i2c_init(i2c1, 1000 * 1000);
    gpio_set_function(i2c1_sda_pin, GPIO_FUNC_I2C);
    gpio_set_function(i2c1_scl_pin, GPIO_FUNC_I2C);

    eeprom_read(i2c1, eeprom_addr, 0x0000, buf, MAX_INDEX);
    
#if 0
    for (int32_t i = 0; i < MAX_INDEX; i++) printf("%x ", buf[i]);
    printf("\n");
#endif

    uint32_t time;
    int16_t alt;
    for (int32_t i = 0; i <= MAX_INDEX-8; i+=8) {
        //printf("buf: %x %x\n", buf[i], buf[i+1]);
        if (buf[i+8] == '\0') {
            time = (uint32_t)((buf[i] << 24) | (buf[i+1] << 16) | (buf[i+2] << 8) | buf[i+3]);
            alt  = (int16_t)(buf[i+4] << 8 | buf[i+5]);
            printf("%d,%d,%c\n", time, alt, buf[i+6]);
        } else {
            break;
        }
    }

    printf("end of file\n");
}