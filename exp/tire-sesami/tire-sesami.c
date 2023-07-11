#include <stdio.h>
#include "pico/stdlib.h"
#include "../../lib/kaguya-pin/kaguya-pin.h"
#include "../../lib/motor_encoder/src/motor.h"

void waitHuman(uint slice, int16_t pwm) {
    char c;
    printf("press 's' to set slice %d to pwm %d>\n", slice, pwm);
    while (1) {
        if ((c = getchar_timeout_us(1000)) == 's') {
            printf("motor_rotate: %d %d\n", slice, pwm);
            motor_rotate(slice, pwm);
            break;
        }
        sleep_ms(1);
    }
}

int main(void) {
    stdio_init_all();
    sleep_ms(2000);
    printf("TIRE SESAMI\n--\n");

    uint slice_left = motor_init(motor_left_a_pin);
    uint slice_right = motor_init(motor_right_a_pin);

    motor_rotate(slice_left, -1023);
    sleep_ms(1000);
    motor_rotate(slice_left, 1023);
    sleep_ms(1000);
    motor_rotate(slice_left, 0);

    PIO pio = pio0;
    const uint sm = 0;
    uint32_t delta[2];

    quadrature_encoder_two_pio_init(pio, sm, encoder_left_a_pin, encoder_right_a_pin);

    printf("press 's' to rotate, press 's' again to stop.>\n");

    int16_t pwm = 1023;
    uint32_t tbefore = time_us_32();
    uint32_t tafter;
    while (1) {
        quadrature_encoder_update_delta(pio, sm, delta);
        waitHuman(slice_right, pwm);
        printf("before: ");
        printf("M1: d %6d  M2: d %6d\n", delta[0], delta[1]);
        waitHuman(slice_right, 0);
        tafter = time_us_32();
        quadrature_encoder_update_delta(pio, sm, delta);
        printf("after: ");
        printf("M1: d %6d  M2: d %6d\n", delta[0], delta[1]);
        printf("elapsed time: %d us == %d ms\n", tafter-tbefore, (tafter-tbefore)/1000);
        pwm = -pwm;
    }
}