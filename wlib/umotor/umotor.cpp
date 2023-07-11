#include "umotor.h"

void Motor::init(int8_t left, int8_t right) {
    slice_left = motor_init(left);
    slice_right = motor_init(right);
    
    //quadrature_encoder_two_pio_init(pio, 0, encoder_left_a_pin, encoder_right_a_pin);
    motor_rotate(slice_left, 0);
    motor_rotate(slice_right, 0);
}

void Motor::forward(int16_t pwm) {
    motor_rotate(slice_left, -pwm);
    motor_rotate(slice_right, pwm);
}

void Motor::stop() {
    motor_rotate(slice_left, 0);
    motor_rotate(slice_right, 0);
}

void Motor::leftH() {
    motor_rotate(slice_left, -1023);
    motor_rotate(slice_right, 0);
}

void Motor::rightH() {
    motor_rotate(slice_left, 0);
    motor_rotate(slice_right, 1023);
}

void Motor::leftM() {
    motor_rotate(slice_left, -800);
    motor_rotate(slice_right, 1023);
}

void Motor::rightM() {
    motor_rotate(slice_right, 800);
    motor_rotate(slice_left, -1023);
}