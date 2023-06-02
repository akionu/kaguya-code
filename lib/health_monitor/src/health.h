#pragma once
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"

void health_init(const uint isense_pin, const uint vsense_pin) ;

float health_current_read(const uint isense_pin);

float health_voltage_read(const uint vsense_pin);