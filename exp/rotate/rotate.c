#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "lora.h"
#include "../../lib/kaguya-pin/kaguya-pin.h"

#define DELAY 1

e220_pin_t lora_pin = {lora_tx_pin, lora_m0_pin, lora_m1_pin, lora_aux_pin};
// addr, datarate, 
e220_config_t lora_cfg = {0xffff, 0x62, 0xc2, 0, 0x00, 0x0000};

int main(void) {
    stdio_init_all();
    sleep_ms(2000);
    printf("hello, this is rotate\n");
    lora_init(uart0, &lora_pin);
    lora_config(uart0, &lora_cfg, &lora_pin);

    uint8_t buf[4] = {'t'};
    uint8_t rsv[50];
    uint8_t ch;
    while (1) {
        ch = getchar_timeout_us(10 * 1000);
        //printf("ch: %c\n", ch);

        switch (ch)
        {
        case 'a':
            buf[0] = 'a';
            //sleep_ms(DELAY);
            lora_send(uart0, buf, 1);
            break;
        case 'd':
            buf[0] = 'd';
            lora_send(uart0, buf, 1);
            break;
        case 'w':
            buf[0] = 'w';
            //sleep_ms(DELAY);
            lora_send(uart0, buf, 1);
            break;
        case 'l':
            buf[0] = 'l';
            //sleep_ms(DELAY);
            lora_send(uart0, buf, 1);
            break;
        case 'r':
            buf[0] = 'r';
            //sleep_ms(DELAY);
            lora_send(uart0, buf, 1);
            break;
        case 'x':
            buf[0] = 'x';
            //sleep_ms(DELAY);
            lora_send(uart0, buf, 1);
            break;
        case 'y':
            buf[0] = 'y';
            //sleep_ms(DELAY);
            lora_send(uart0, buf, 1);
            break;
        case 'v':
            buf[0] = 'v';
            //sleep_ms(DELAY);
            lora_send(uart0, buf, 1);
            break;
        case 'b':
            buf[0] = 'b';
            //sleep_ms(DELAY);
            lora_send(uart0, buf, 1);
            break;
        
        default:
            break;
        }
        
        sleep_ms(200);
    }
    return 0;
}