#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "../../lib/kaguya-pin/kaguya-pin.h"

int main(void) {
    stdio_init_all();
    sleep_ms(2000);
    printf("hi, this is KAGUYA-EM!\n");

    gpio_init(nichrome_pin);
    gpio_set_dir(nichrome_pin, GPIO_OUT);

    sleep_ms(10000);
    gpio_put(nichrome_pin, 1);
    sleep_ms(2000);
    gpio_put(nichrome_pin, 0);
}