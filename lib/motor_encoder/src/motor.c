#include "motor.h"

uint8_t motor_init(uint8_t pin) {
    uint8_t slice = pwm_gpio_to_slice_num(pin);
    gpio_set_function(pin, GPIO_FUNC_PWM);
    gpio_set_function((pin + 1), GPIO_FUNC_PWM);

    pwm_set_clkdiv(slice, 1.0614809); // clkdiv = sysclock / ((wrap + 1) * f)
    pwm_set_wrap(slice, 1023);        // pwm resolution = 1024, 115kHz
    pwm_set_chan_level(slice, PWM_CHAN_A, 0);
    pwm_set_chan_level(slice, PWM_CHAN_B, 0);
    return slice;
}

void motor_brake(uint8_t slice) {
    pwm_set_chan_level(slice, PWM_CHAN_A, 1023);
    pwm_set_chan_level(slice, PWM_CHAN_B, 1023);
    pwm_set_enabled(slice, true);
}

void motor_rotate(uint8_t slice, int16_t pwm) {
    // abs(pwm) < 500 (at 7.4V) means no rotation because of high(1:380) gear rates
    if (pwm > 0) {
        // forward
        pwm_set_chan_level(slice, PWM_CHAN_A, 0);
        pwm_set_chan_level(slice, PWM_CHAN_B, pwm);
    } else {
        // backward
        pwm_set_chan_level(slice, PWM_CHAN_A, -pwm);
        pwm_set_chan_level(slice, PWM_CHAN_B, 0);
    }
    
    pwm_set_enabled(slice, true);
}

void quadrature_encoder_two_pio_init(PIO pio, uint sm, uint8_t pin_ab_left, uint8_t pin_ab_right) {
    uint offset = pio_add_program(pio, &quadrature_encoder_program);
    quadrature_encoder_program_init(pio, sm, offset, pin_ab_left, 0);
    quadrature_encoder_program_init(pio, sm+1, offset, pin_ab_right, 0);
}

void quadrature_encoder_update_delta(PIO pio, uint sm, uint32_t delta[2]) {
    static uint32_t new_value[2] = {0};
    static uint32_t old_value[2] = {0};
    quadrature_encoder_request_count(pio, sm);
    quadrature_encoder_request_count(pio, sm+1);
    new_value[0] = quadrature_encoder_fetch_count(pio, sm);
    new_value[1] = quadrature_encoder_fetch_count(pio, sm+1);
    delta[0] = new_value[0] - old_value[0];
    delta[1] = new_value[1] - old_value[1];
    old_value[0] = new_value[0];
    old_value[1] = new_value[1];
}
