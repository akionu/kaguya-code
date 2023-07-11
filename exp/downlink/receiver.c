#include <stdio.h>
#include "pico/stdlib.h"
#include "../../lib/gnss/nmeap-0.3/inc/nmeap.h"
#include "../../lib/pico-e220/src/pico-e220.h"
#include "../../lib/kaguya-pin/kaguya-pin.h"

e220_pin_t lora_pin = {lora_tx_pin, lora_m0_pin, lora_m1_pin, lora_aux_pin};

e220_config_t lora_cfg = {
    0xffff, // addr: all(broadcast)
    96,     // baudrate: 9600 bps
    12509,  // air data rate: bw: 125, sf: 9
    32,     // subpacket length: 32 byte
    false,  // rssi: off
    13,     // transmitting power: 13
    0,      // channel: 0
    false,  // rssi output: off
    true,   // mode: transparent
    20,     // wor cycle: 2 sec
    0x0000  // encryption key: 0x0000
};

int main(void) {
    uint8_t rbuf[200] = {0};

    stdio_init_all();
    sleep_ms(2000);
    printf("hello, this is receiver\n");
    
    // lora
    lora_init(uart0, &lora_pin);
    lora_config(uart0, &lora_cfg, &lora_pin);

    while (1) {
        lora_receive(uart0, rbuf);
        for (int i = 0; i < 32; i++) printf("%c", rbuf[i]);
        printf("\n");
        sleep_ms(500);
    }

}