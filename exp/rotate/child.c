#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "lora.h"
#include "../../lib/kaguya-pin/kaguya-pin.h"

e220_pin_t lora_pin = {lora_tx_pin, lora_m0_pin, lora_m1_pin, lora_aux_pin};
// addr, datarate, 
e220_config_t lora_cfg = {0xffff, 0x62, 0xc2, 0, 0x00, 0x0000};

int main(void) {
    stdio_init_all();
    sleep_ms(2000);
    printf("hello, this is rotate\n");

    uint slice_l = motor_init(motor_left_a_pin);
    uint slice_r = motor_init(motor_right_a_pin);

    //printf("motor test:");
    motor_rotate(slice_r, 1023);
    sleep_ms(1000);
    motor_rotate(slice_r, 0);
    motor_rotate(slice_l, -1023);
    sleep_ms(1000);
    motor_rotate(slice_l, 0);

    lora_init(uart0, &lora_pin);
    lora_config(uart0, &lora_cfg, &lora_pin);

    uint8_t rsv[50];
    int16_t cnt = 0;
    while (1) {
        lora_receive(uart0, rsv);
        if (cnt > 50) {
            motor_rotate(slice_r, 0);
            motor_rotate(slice_l, 0);
            cnt = 0;
        }

        switch (rsv[0])
        {
        case 'a':
            motor_rotate(slice_l, -1023);
            break;
        case 'd':
            motor_rotate(slice_r, 1023);
            break;
        case 'w':
            motor_rotate(slice_r, 1023);
            motor_rotate(slice_l, -1023);
            break;
        case 'v':
            motor_rotate(slice_r, 800);
            motor_rotate(slice_l, -800);
            break;
        case 'b':
            motor_rotate(slice_r, -800);
            motor_rotate(slice_l, 800);
            break;
        case 'l':
            motor_rotate(slice_l, -800);
            break;
        case 'r':
            motor_rotate(slice_r, 800);
            break;
        case 'x':
            motor_rotate(slice_r, 600);
            break;
        case 'y':
            motor_rotate(slice_r, 600);
            break;


        default:
            cnt++;
            break;
        }

        rsv[0] = 's';
        sleep_ms(10);
    }
    return 0;
}