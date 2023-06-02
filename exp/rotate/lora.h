#pragma once
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/uart.h"

typedef struct e220_config {
    uint16_t addr;
    uint8_t reg0; // baudrate, air data rate
    uint8_t reg1; // subpacket len, rssi, transmit power
    uint8_t reg2; // channel
    uint8_t reg3; // rssi byte, transmit mode, wor cycle
    uint16_t enckey;
} e220_config_t;

typedef struct e220_pin {
    uint8_t txpin;
    uint8_t m0pin;
    uint8_t m1pin;
    uint8_t auxpin;
} e220_pin_t;

void lora_init(uart_inst_t *uart, e220_pin_t *pin);
void lora_mode(e220_pin_t *pin, int8_t mode);
void lora_config(uart_inst_t *uart, e220_config_t *config, e220_pin_t *pin);
void lora_send(uart_inst_t *uart, uint8_t *src, int8_t len);
void lora_receive(uart_inst_t *uart, uint8_t *dst);