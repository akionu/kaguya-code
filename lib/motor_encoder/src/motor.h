#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/pio.h"
#include "hardware/timer.h"

#include "quadrature_encoder.pio.h"

// control motor

/// @brief initialize pwm channels for motor
/// @param pin gpio num. pin and pin+1 will be initialied
/// @return slice number. must handle this to rotate/stop motor
uint8_t motor_init(uint8_t pin);

/// @brief rotate motor
/// @param slice slice number to be manipulated, ret value of motor_init
/// @param pwm -1023 to 1023. - counterclockwise, 0 stop, + clockwise
void motor_rotate(uint8_t slice, int16_t pwm);

void motor_brake(uint8_t slice);

// reading encoder

/// @brief initialize pio for reading encoder
/// @param pio pio0 or pio1
/// @param sm state machine number. sm and sm+1 will be used for reading encoder
/// @param pin_ab_left phase A pin of left motor
/// @param pin_ab_right phase A pin of right motor
void quadrature_encoder_two_pio_init(PIO pio, uint sm, uint8_t pin_ab_left, uint8_t pin_ab_right);

/// @brief update/get encoder delta rotation, 一定の周期で呼び出すことを推奨
/// @param pio pio0 or pio1
/// @param sm state machine number. sm and sm+1 will be updated
/// @param delta store calcurated value
void quadrature_encoder_update_delta(PIO pio, uint sm, uint32_t delta[2]);
