#include <stdio.h>
#include "pico/stdlib.h"
#include "../../lib/kaguya-pin/kaguya-pin.h"
//#include "../../lib/motor_encoder/src/motor.h"
#include "../../ulib/umotor/umotor.h"

Motor motor;

int main(void) {
    stdio_init_all();
    sleep_ms(2000);
    printf("hello, this is rotate\n");

    motor.init(motor_left_a_pin, motor_right_a_pin);

    sleep_ms(10000);
    motor.backward(1023);
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