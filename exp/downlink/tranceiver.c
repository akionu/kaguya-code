#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/rtc.h"
#include "pico/util/datetime.h"
#include "../../lib/gnss/nmeap-0.3/inc/nmeap.h"
#include "../../lib/pico-e220/src/pico-e220.h"
#include "../../lib/kaguya-pin/kaguya-pin.h"

datetime_t time = {
            .year  = 2023,
            .month = 06,
            .day   = 17,
            .dotw  = 6, // 0 is Sunday, so 5 is Friday
            .hour  = 00,
            .min   = 00,
            .sec   = 00
};

static repeating_timer_t timer;

// gps
nmeap_context_t nmea;
nmeap_gga_t gga;
static bool gps_isready = false;

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

void on_uart_rx() {
    while (uart_is_readable(uart1)) {
        uint8_t ch = uart_getc(uart1);
        nmeap_parse(&nmea, ch);
    }
}

static void gpgga_callout(nmeap_context_t *context, void *data, void *user_data) {
    gps_isready = true;
}

bool send(repeating_timer_t *rt) {
    static uint8_t sbuf[32];
    for (uint8_t i = 0; i < 32; i++) sbuf[i] = 0;
    if (gps_isready) {
        gps_isready = false;
        printf("gps_isready: true, %03.6f,%03.6f, sat %d\n", gga.latitude, gga.longitude, gga.satellites);

        if (gga.satellites > 0) {
            rtc_get_datetime(&time);
            snprintf(sbuf, 32, "%02d%02d%02d,%3.6f,%3.6f\n", time.hour, time.min, time.sec, gga.latitude, gga.longitude);

            printf("sbuf: %s\n");
            lora_send(uart0, sbuf, 32);
        }
        
    }
}

int main(void) {
    uint8_t sbuf[32] = {0};

    stdio_init_all();
    sleep_ms(2000);
    printf("hello, this is tranceiver\n");
    
    // lora
    lora_init(uart0, &lora_pin);
    lora_config(uart0, &lora_cfg, &lora_pin);

    // gnss
    uart_init(uart1, 9600);
    gpio_set_function(gnss_tx_pin, GPIO_FUNC_UART);
    gpio_set_function(gnss_rx_pin, GPIO_FUNC_UART);
    uart_set_baudrate(uart1, 9600);
    uart_set_hw_flow(uart1, false, false);
    uart_set_format(uart1, 8, 1, UART_PARITY_NONE);
    //uart_set_fifo_enabled(uart1, true);
    uart_set_fifo_enabled(uart1, false);
    irq_set_exclusive_handler(UART1_IRQ, on_uart_rx);
    irq_set_enabled(UART1_IRQ, true);
    uart_set_irq_enables(uart1, true, false);

    nmeap_init(&nmea, NULL);
    nmeap_addParser(&nmea, "GNGGA", nmeap_gpgga, gpgga_callout, &gga);

    // rtc
    rtc_init();
    rtc_set_datetime(&time);
    sleep_us(64);

    add_repeating_timer_ms(-5000, &send, NULL, &timer);

    while (1) {
        tight_loop_contents();
    }

}