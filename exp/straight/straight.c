#include <stdio.h>
#include "pico/stdlib.h"
#include "../../lib/kaguya-pin/kaguya-pin.h"

int main(void) {
    stdio_init_all();
    sleep_ms(2000);
    printf("hello, this is rotate\n");

    uint slice_l = motor_init(motor_left_a_pin);
    uint slice_r = motor_init(motor_right_a_pin);

    //printf("motor test:");
    motor_rotate(slice_r, 1023);
    sleep_ms(1000);    
    motor_rotate(slice_l, -1023);
    sleep_ms(1000);
    int pwm = 500;
    while (1) {
        if (pwm > 1023) pwm = 500;
        motor_rotate(slice_r, pwm);
        motor_rotate(slice_l, -pwm);
        pwm++;
        sleep_ms(50);
    }

}