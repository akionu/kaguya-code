#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "../../lib/kaguya-pin/kaguya-pin.h"

void nichrome_onoff(uint8_t pin, uint8_t seconds) {
    printf("ON for %ds\n", seconds);
    gpio_put(pin, 1);
    sleep_ms(seconds * 1000);
    gpio_put(pin, 0);
    printf("OFF\n");
    printf("wait a few seconds for the next heat...\n");
    sleep_ms(2000);
}

int main(void) {
    stdio_init_all();
    sleep_ms(2000);
    printf("hi, this is KAGUYA-EM!\n");

    gpio_init(nichrome_pin);
    gpio_set_dir(nichrome_pin, GPIO_OUT);

    printf("nichrome ready.\npress \"1\" to heat.\n");
    uint8_t c = 63;
    while (1) {
        c = getchar_timeout_us(10 * 1000);
        //printf("%c\n", c);
        if (c == 0 || c < 0) continue;
        if (c == '1') {
            nichrome_onoff(nichrome_pin, 1);
        } else if (c == '2') {
            nichrome_onoff(nichrome_pin, 2);
        } else if (c == '3') {
            nichrome_onoff(nichrome_pin, 3);
        } 
        //sleep_ms(10);
    }
}