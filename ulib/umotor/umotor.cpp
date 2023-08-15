#include "umotor.h"

Motor::Motor() {
    is_dir_set = false;
}

void Motor::init(int8_t left, int8_t right) {
    slice_left = motor_init(left);
    slice_right = motor_init(right);
    
    //quadrature_encoder_two_pio_init(pio, 0, encoder_left_a_pin, encoder_right_a_pin);
    motor_rotate(slice_left, 0);
    motor_rotate(slice_right, 0);
}

void Motor::forward(int16_t pwm) {
    if (is_dir_set) {
        motor_rotate(slice_left, dir_left*pwm);
        motor_rotate(slice_right, dir_right*pwm);
    } else {
        motor_rotate(slice_left, -pwm);
        motor_rotate(slice_right, pwm);
    }
    
}

void Motor::backward(int16_t pwm) {
    if (is_dir_set) {
        motor_rotate(slice_left, -dir_left*pwm);
        motor_rotate(slice_right, -dir_right*pwm);
    } else {
        motor_rotate(slice_left, pwm);
        motor_rotate(slice_right, -pwm);
    }
}

void Motor::stop() {
    motor_rotate(slice_left, 0);
    motor_rotate(slice_right, 0);
}

void Motor::rightH() {
    if (is_dir_set) {
        motor_rotate(slice_left, dir_left*1023);
        motor_rotate(slice_right, 0);
    } else {
        motor_rotate(slice_left, -1023);
        motor_rotate(slice_right, 0);
    }
}

void Motor::leftH() {
    if (is_dir_set) {
        motor_rotate(slice_left, 0);
        motor_rotate(slice_right, dir_right*1023);
    } else {
        motor_rotate(slice_left, 0);
        motor_rotate(slice_right, 1023);
    }
}

void Motor::leftM() {
    if (is_dir_set) {
        motor_rotate(slice_left, dir_left*500);
        motor_rotate(slice_right, dir_right*1023);
    } else {
        motor_rotate(slice_left, -500);
        motor_rotate(slice_right, 1023);
    }
}

void Motor::rightM() {
    if (is_dir_set) {
        motor_rotate(slice_left, dir_left*1023);
        motor_rotate(slice_right, dir_right*500);
    } else {
        motor_rotate(slice_left, -1023);
        motor_rotate(slice_right, 500);
    }
}

// 1: seiten, -1: gyakuten
void Motor::setDirForward(int8_t left, int8_t right) {
    int8_t ltmp, rtmp;
    ltmp = (left > 0) ? left : -left;
    rtmp = (right > 0) ? right : -right;
    if ((ltmp != 1) || (rtmp != 1)) {
        printf("err: setDirForward; left: %d, right: %d\n");
    }
    dir_left = left;
    dir_right = right;
    is_dir_set = true;
}