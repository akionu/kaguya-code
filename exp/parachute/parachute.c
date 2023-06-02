#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "../../lib/pico-eeprom-i2c/src/eeprom.h"
#include "../../lib/pico-spl06-001/src/pico-spl06-001.h"
#include "../../lib/kaguya-pin/kaguya-pin.h"

// cfg
int16_t th = 195;

// eeprom
uint8_t eeprom_addr = 0x50;
uint32_t head = 0;

// spl06-001
spl06_config_t config = {0x77, i2c0, BGD_PRS_TEMP, {2, 64}, {2, 8}};
spl06_coef_t coef;
float prs;
int16_t alt;

int main (void){
    stdio_init_all();
    sleep_ms(2000);

    gpio_init(nichrome_pin);
    gpio_set_dir(nichrome_pin, GPIO_OUT);

    i2c_init(i2c0, 400 * 1000);
    gpio_set_function(i2c0_sda_pin, GPIO_FUNC_I2C);
    gpio_set_function(i2c0_scl_pin, GPIO_FUNC_I2C);

    i2c_init(i2c1, 1000 * 1000);
    gpio_set_function(i2c1_sda_pin, GPIO_FUNC_I2C);
    gpio_set_function(i2c1_scl_pin, GPIO_FUNC_I2C);

    spl06_init(&config, &coef);

    uint32_t time_ms;
    uint8_t wbuf[10] = {'_'};
    int16_t cnt = 0;
    bool isfirst = true;
    while (1) {
        spl06_read_press_cal(&config, &coef, &prs);
        alt = (int16_t)(spl06_calc_alt(prs) * 10);

        time_ms = time_us_32() / 1000;
        wbuf[0] = (uint8_t)(time_ms >> 24);
        wbuf[1] = (uint8_t)(time_ms >> 16);
        wbuf[2] = (uint8_t)(time_ms >> 8);
        wbuf[3] = (uint8_t)(time_ms & 0xff);
        printf("%d ms\n", time_ms);

        wbuf[4] = (uint8_t)(alt >> 8);
        wbuf[5] = (uint8_t)(alt & 0xff);
        printf("%d * 10 cm\n", alt);
        printf("wbuf: %x %x %x %x, %x %x\n", wbuf[0], wbuf[1], wbuf[2], wbuf[3], wbuf[4], wbuf[5]);

#if 0
        if (cnt == 13) gpio_put(nichrome_pin, 0);
        if (alt < th) {
            if (isfirst) {
                isfirst = false;
                wbuf[6] = 'T';
                gpio_put(nichrome_pin, 1);
            }
            if (cnt <= 13) cnt++;
        } else {
            wbuf[6] = 'F';
        }
        wbuf[7] = '\n';
#endif

        printf("%s\n", wbuf);
        eeprom_write_multi(i2c1, eeprom_addr, head, wbuf, 8);
        head += 8;
        sleep_ms(150);
    }

}