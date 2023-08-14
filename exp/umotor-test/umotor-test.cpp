#include <stdio.h>
#include "pico/stdlib.h"
#include "../../ulib/umotor/umotor.h"
#include "../../lib/kaguya-pin/kaguya-pin.h"

Motor motor;

int main(void) {
    stdio_init_all();
    sleep_ms(2000);
    printf("hello, pico!\n");

    motor.init(motor_left_a_pin, motor_right_a_pin);

    while (1) {
        printf("forward\n");
        motor.forward(1023);
        sleep_ms(3000);

        printf("backward\n");
        motor.backward(1023);
        sleep_ms(3000);

        printf("left move\n");
        motor.leftM();
        sleep_ms(3000);

        printf("right move\n");
        motor.rightM();
        sleep_ms(3000);

        printf("left here\n");
        motor.leftH();
        sleep_ms(3000);

        printf("right here\n");
        motor.rightH();
        sleep_ms(3000);

        printf("stop\n");
        motor.stop();
        sleep_ms(3000);
    }
    
}