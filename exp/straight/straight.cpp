#include <stdio.h>
#include "pico/stdlib.h"
#include "../../lib/kaguya-pin/kaguya-pin.h"
#include "../../ulib/umotor/umotor.h"

Motor motor;

int main(void) {
    stdio_init_all();
    sleep_ms(2000);
    printf("hello, this is rotate\n");

    //uint left = motor_init(22);
    //uint right = motor_init(24);
    motor.init(motor_left_a_pin, motor_right_a_pin);
    //motor.init(22, 24);

    //sleep_ms(10000);
    //motor.backward(1023);
    motor.forward(1023);
    //motor_rotate(left, 1000);
    //motor_rotate(right, 600);

    while (1) {

        printf("still alive!: %u ms\n", time_us_32()/1000);
        sleep_ms(50);
    }

}