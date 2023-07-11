#include <stdio.h>
#include "pico/stdlib.h"
#include "../../lib/kaguya-pin/kaguya-pin.h"
#include "../../lib/motor_encoder/src/motor.h"

int main(void) {
    stdio_init_all();
    sleep_ms(2000);
    printf("hello, this is rotate\n");

    uint slice_l = motor_init(21);
    uint slice_r = motor_init(23);

    //printf("motor test:");
    motor_rotate(slice_r, 1023);
    sleep_ms(1000);    
    motor_rotate(slice_l, -1023);
    sleep_ms(1000);
    int pwm = 500;
    while (1) {
#if 0
        if (pwm > 1023) pwm = 500;
        motor_rotate(slice_r, pwm);
        motor_rotate(slice_l, -pwm);
        pwm++;
#endif
        printf("still alive!: %u ms\n", time_us_32()/1000);
        sleep_ms(50);
    }

}