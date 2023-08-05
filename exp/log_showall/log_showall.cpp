#include <stdio.h>
#include "pico/stdlib.h"
#include "../../lib/kaguya-pin/kaguya-pin.h"
#include "../../ulib/ulog/ulog.h"

Log logging('X');

int main(void) {
    stdio_init_all();
    sleep_ms(2000);
    printf("hello this is log_showall\n");
    logging.head = 64000;
    // i2c1: eeprom, tof
    i2c_init(i2c1, 1000 * 1000);
    gpio_set_function(i2c1_sda_pin, GPIO_FUNC_I2C);
    gpio_set_function(i2c1_scl_pin, GPIO_FUNC_I2C);

    logging.showAll();
    while (1) {
        tight_loop_contents();
    }
}