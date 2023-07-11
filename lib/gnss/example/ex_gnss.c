#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "../nmeap-0.3/inc/nmeap.h"

static bool is_ready = false;

static void print_gga(nmeap_gga_t *gga) {
    is_ready = false;
    printf("lat %.10f, long %.10f, alt %.0f, time %lu, sat %d, qu %d, dop %f, gh %f\n",
            gga->latitude  ,
            gga->longitude, 
            gga->altitude , 
            gga->time     , 
            gga->satellites,
            gga->quality   ,
            gga->hdop      ,
            gga->geoid     
            );
}

/** called when a gpgga message is received and parsed */
static void gpgga_callout(nmeap_context_t *context,void *data,void *user_data) {
    is_ready = true;
}

int main(void) {
    stdio_init_all();

    uart_init(uart1, 9600);
    gpio_set_function(4, GPIO_FUNC_UART);
    gpio_set_function(5, GPIO_FUNC_UART);

    uart_set_baudrate(uart1, 9600);
    uart_set_hw_flow(uart1, false, false);
    uart_set_format(uart1, 8, 1, UART_PARITY_NONE);
    uart_set_fifo_enabled(uart1, true);

    gpio_init(7);
    gpio_set_dir(7, GPIO_OUT);
    gpio_put(7, 0);

    nmeap_context_t nmea;
    nmeap_gga_t gga;
    int8_t status;

    status = nmeap_init(&nmea, NULL);
    if (status != 0) printf("err: nmea_init %d\n", status);
    nmeap_addParser(&nmea, "GNGGA", nmeap_gpgga, gpgga_callout, &gga);
    if (status != 0) printf("err: nmea_addParser %d\n", status);

    uint8_t buf[100];
    while (1) {
        if (uart_is_readable(uart1)) {
            uart_read_blocking(uart1, buf,  10);
            for (uint i = 0; buf[i] != '\0'; i++) {
                status = nmeap_parse(&nmea, buf[i]);
            }
            //printf("%s", buf);
        }
        for (uint i = 0; i < 100; i++) buf[i] = '\0';

        if (is_ready == true) print_gga(&gga);
        sleep_ms(10);
    }

    return 0;
}