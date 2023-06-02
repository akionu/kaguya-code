#include "../src/health.h"
#include "../../prometheus-pin/prometheus-pin.h"
//#include "../../motor_encoder/src/motor.h"

int main(void) {
    stdio_init_all();
    sleep_ms(3000);

    float current, voltage;
    health_init(current_sense_pin, voltage_sense_pin);

    //uint slice_left = motor_init(motor_right_a_pin);
    //motor_rotate(slice_left, 800);

    while (1) {
        current = health_current_read(current_sense_pin);
        voltage = health_voltage_read(voltage_sense_pin);
        printf("health: %f V, %f A\n", voltage, current);
        sleep_ms(1000);
    }
}