#include "health.h"

void health_init(const uint isense_pin, const uint vsense_pin) {
    adc_init();

    adc_gpio_init(isense_pin);
    adc_gpio_init(vsense_pin);
}

float health_current_read(const uint isense_pin) {
    adc_select_input(isense_pin-26);
    uint16_t res = adc_read();
    return ((float)(res * (3.3f / (1 << 12)) * 0.15f));
}

float health_voltage_read(const uint vsense_pin) {
    adc_select_input(vsense_pin-26);
    uint16_t res = adc_read();
    return ((float)(res * (3.3f / (1 << 12)) * 4.0f));
}