#include "../src/motor.h"

int main() {
    stdio_init_all();

    uint8_t slice_left = motor_init(20);
    uint8_t slice_right = motor_init(18);

    // Base pin to connect the A phase of the encoder.
    // The B phase must be connected to the next pin
    const uint PIN_AB[2] = {22, 24}; // 22: left, 24: right

    PIO pio = pio0;
    const uint sm = 0;
    uint32_t delta[2];

    quadrature_encoder_two_pio_init(pio, sm, PIN_AB[0], PIN_AB[1]);

    int16_t pwml = 1023, pwmr = 800;
    motor_rotate(slice_left, pwml);
    motor_rotate(slice_right, pwmr);

    while (1) {
        // note: thanks to two's complement arithmetic delta will always
        // be correct even when new_value wraps around MAXINT / MININT
        quadrature_encoder_update_delta(pio, sm, delta);

        printf("M1: d %6d, pwm %6d  M2: d %6d, pwm %6d\n", delta[0], pwml, delta[1], pwmr);
        //printf("M1: pos %8d, d %6d, pwm %6d  M2: pos %8d, d %6d, pwm %6d\n", new_value[0], -delta[0], pwml, new_value[1], delta[1], pwmr);
        //printf("position %8d, delta %6d, pwm %6d\n", new_value, delta, pwm);

        //if (pwm >= 1023) pwm = -1023;
        //pwm++;
        sleep_ms(10);
    }
}