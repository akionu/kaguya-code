#include <stdio.h>
#include <math.h>

/*
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/pwm.h"
#include "hardware/uart.h"
#include "hardware/adc.h"
#include "hardware/pio.h"
#include "hardware/gpio.h"
*/

#include "../lib/kaguya-pin/kaguya-pin.h"
#include "../lib/gnss/nmeap-0.3/inc/nmeap.h"
#include "../lib/health_monitor/src/health.h"
//#include "../lib/motor_encoder/src/motor.h"
#include "../lib/pico-bno08x/src/pico-bno08x.h"
#include "../wlib/umotor/umotor.h"
#include "../lib/pico-eeprom-i2c/src/eeprom.h"
#include "../wlib/uprs/uprs.h"
#include "config.h"

extern "C" {
    #include "../lib/pico-e220/src/pico-e220.h"
    //#include "../lib/pico-spl06-001/src/pico-spl06-001.h"
    #include "../lib/pico-vl53l5cx/api/vl53l5cx_api.h"
    #include "util.h"
}

Motor motor;
Press prs;

void modeWait() {
    float alt = prs.getAlt();
    printf("%f\n", alt);
}

void modeFowardLanding() {
    motor.init(motor_left_a_pin, motor_right_a_pin);
    motor.back(1023);
    sleep_ms(12000);
    motor.stop();
}


int main(void) {
    stdio_init_all();
    sleep_ms(2000);

    i2c_init(i2c0, 400 * 1000);
    gpio_set_function(12, GPIO_FUNC_I2C);
    gpio_set_function(13, GPIO_FUNC_I2C);

    prs.init();
    while (1) {
        modeWait();
        sleep_ms(50);
    }
    
}