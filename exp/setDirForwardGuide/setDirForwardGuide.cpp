#include <stdio.h>
#include "pico/stdlib.h"
#include "../../lib/kaguya-pin/kaguya-pin.h"
#include "../../ulib/umotor/umotor.h"

Motor motor;

void rotateWithDir(int8_t l, int8_t r) {
    printf("forward: L %d, R %d\n", l, r);
    motor.setDirForward(l, r);
    motor.forward(1023);
    sleep_ms(3000);
    motor.stop();
    sleep_ms(500);
}

int main(void) {
    stdio_init_all();
    sleep_ms(2000);
    printf("hello, this is setDirForwardGuide\n");

    int8_t l, r;

    motor.init(motor_left_a_pin, motor_right_a_pin);

    while (1) {
        l = 1; r = 1;
        rotateWithDir(l, r);

        l = 1; r = -1;
        rotateWithDir(l, r);

        l = -1; r = 1;
        rotateWithDir(l, r);

        l = -1; r = -1;
        rotateWithDir(l, r);
    }
}