//とりあえずprometheus-codeからコピペしてきただけ

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/pwm.h"
#include "hardware/uart.h"
#include "hardware/adc.h"
#include "hardware/pio.h"
#include "hardware/gpio.h"
#include "../../lib/prometheus-pin/prometheus-pin.h"
#include "../../lib/motor_encoder/src/motor.h"
#include "../../lib/icm20948/src/pico-icm20948.h"
#include "../../lib/vl53l5cx/api/vl53l5cx_api.h"
#include "../../lib/health_monitor/src/health.h"
#include "../../lib/pico-eeprom-i2c/src/eeprom.h"

int main(void) {
    stdio_init_all();
    sleep_ms(2000);
    printf("\n\nbegin selftest:\n");

    // i2c0
    // vl53l5cx
    i2c_inst_t vl53l5cx_i2c = {i2c0_hw, false};
    gpio_init(tof_vcc_pin);
    gpio_set_dir(tof_vcc_pin, GPIO_OUT);
    gpio_put(tof_vcc_pin, 0);
    sleep_ms(100);

    i2c_init(&vl53l5cx_i2c, 400 * 1000);
    gpio_set_function(i2c0_scl_pin, GPIO_FUNC_I2C);
    gpio_set_function(i2c0_sda_pin, GPIO_FUNC_I2C);

    uint8_t status, is_ready, is_alive;
    VL53L5CX_Configuration dev;
    VL53L5CX_ResultsData res;

    dev.platform.address = 0x29;
    dev.platform.i2c     = &vl53l5cx_i2c;
    status = vl53l5cx_is_alive(&dev, &is_alive);
    status = vl53l5cx_init(&dev);
    if (status) printf("vl53l5cx: ULD load failed\n");
    if (!status) printf("vl53l5cx: OK\n");
    else printf("vl53l5cx: NG\n");

    // i2c1
    // icm-20948
    icm20948_config_t config = {0x68, 0x0C, i2c1};
    i2c_init(i2c1, 400 * 1000);
    gpio_set_function(i2c1_sda_pin, GPIO_FUNC_I2C);
    gpio_set_function(i2c1_scl_pin, GPIO_FUNC_I2C);
    if (icm20948_init(&config) == 0) printf("icm20948: OK\n");
    else printf("icm20948: NG\n");

    // eeprom
    uint8_t wbuf[10] = {0};
    uint8_t rbuf[10] = {'0'};
    for (uint8_t i = 0; i < 10; i++) wbuf[i] = 33 + i;
    printf("write:\n");
    for (uint8_t i = 0; i < 10; i++) printf("%c", wbuf[i]);
    printf("\n");
    int16_t ret = eeprom_write_multi(i2c1, 0x50, 0x00000, wbuf, 10);
    if (ret > 0) printf("eeprom write: OK\n");
    else printf("eeprom write: NG\n");
    ret = eeprom_read(i2c1, 0x50, 0x0000, rbuf, 10);
    if (ret > 0) printf("eeprom read: OK\n");
    else printf("eeprom read: NG\n");
    for (int8_t i = 0; i < 10; i++) printf("%c", rbuf[i]);
    printf("\n");

    // gnss
    gpio_init(gnss_vcc_pin);
    gpio_set_dir(gnss_vcc_pin, GPIO_OUT);
    gpio_put(gnss_vcc_pin, 0);

    uart_init(uart1, 9600);
    gpio_set_function(gnss_tx_pin, GPIO_FUNC_UART);
    gpio_set_function(gnss_rx_pin, GPIO_FUNC_UART);
    uart_set_baudrate(uart1, 9600);
    uart_set_hw_flow(uart1, false, false);
    uart_set_format(uart1, 8, 1, UART_PARITY_NONE);
    uart_set_fifo_enabled(uart1, true);

    
    sleep_ms(5000);

    uint8_t buf[100] = {'\0'};
    int16_t n;
    if (n = uart_is_readable(uart1)) {
        uart_read_blocking(uart1, buf, 50);
        printf("%s\n", buf);
    }
    if (n > 0) printf("gnss: OK, n: %d\n", n);
    else printf("gnss: NG, n: %d\n", n);

    // motor, encoder
    uint8_t sleft = motor_init(motor_left_a_pin);
    uint8_t sright = motor_init(motor_right_a_pin);
    PIO pio = pio0;
    uint32_t delta[2];
    motor_rotate(sleft, 800);
    motor_rotate(sright, 800);
    sleep_ms(1000);
    quadrature_encoder_two_pio_init(pio, 0, encoder_left_a_pin, encoder_right_a_pin);
    sleep_ms(10);
    quadrature_encoder_update_delta(pio, 0, delta);
    if (delta[0] > 30) printf("motor_left: OK, delta: %d\n", delta[0]);
    else printf("motor_left: NG\n");
    if (delta[1] > 30) printf("motor_right: OK, delta: %d\n", delta[1]);
    else printf("motor_right: NG\n");
    motor_rotate(sleft, 0);
    motor_rotate(sright, 0);

    // health check
    float current, voltage;
    health_init(current_sense_pin, voltage_sense_pin);
    current = health_current_read(current_sense_pin);
    voltage = health_voltage_read(voltage_sense_pin);
    printf("health: %f V, %f A\n", voltage, current);

    // nichrome
    printf("check nichrome\n");
    sleep_ms(1000);
    gpio_init(nichrome_pin);
    gpio_set_dir(nichrome_pin, GPIO_OUT);
    gpio_put(nichrome_pin, 1);
    sleep_ms(500);
    gpio_put(nichrome_pin, 0);
    sleep_ms(2000);

    // led
    gpio_init(led_pin);
    gpio_set_dir(led_pin, GPIO_OUT);
    
    while (1) {
        gpio_put(led_pin, 1);
        sleep_ms(500);
        gpio_put(led_pin, 0);
        sleep_ms(500);
    }

    return 0;

}