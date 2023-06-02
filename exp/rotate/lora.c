#include "lora.h"

void lora_init(uart_inst_t *uart, e220_pin_t *pin) {
    gpio_init(pin->m0pin);
    gpio_set_dir(pin->m0pin, GPIO_OUT);

    gpio_init(pin->m1pin);
    gpio_set_dir(pin->m1pin, GPIO_OUT);

    gpio_init(pin->auxpin);
    gpio_set_dir(pin->auxpin, GPIO_IN);

    uart_init(uart, 9600);
    gpio_set_function(pin->txpin, GPIO_FUNC_UART);
    gpio_set_function((pin->txpin)+1, GPIO_FUNC_UART);

    uart_set_baudrate(uart, 9600);
    uart_set_hw_flow(uart, false, false);
    uart_set_format(uart, 8, 1, UART_PARITY_NONE);
    uart_set_fifo_enabled(uart, true);

#if 0
    int UART_IRQ = uart == uart0 ? UART0_IRQ : UART1_IRQ;
    irq_set_exclusive_handler(UART_IRQ, lora_on_uart_rx);
    irq_set_enabled(UART_IRQ, true);
    uart_set_irq_enables(uart, true, false);
#endif

    printf("lora: init done\n");
}

void lora_on_uart_rx() {
    while (uart_is_readable(uart0)) {
        printf("%c\n", uart_getc(uart0));
    }
}

void lora_mode(e220_pin_t *pin, int8_t mode) {
    while (gpio_get(pin->auxpin) == 0) sleep_ms(1);
    printf("lora: mode %d\n", ((mode >= 0) && (mode <= 3)) ? mode : 999);
    switch (mode)
    {
    case 0: // normal mode
        gpio_put(pin->m0pin, 0);
        gpio_put(pin->m1pin, 0);
        break;
    case 1: // WOR send mode
        gpio_put(pin->m0pin, 1);
        gpio_put(pin->m1pin, 0);
        break;
    case 2:
        gpio_put(pin->m0pin, 0);
        gpio_put(pin->m1pin, 1);
        break;
    case 3: 
        gpio_put(pin->m0pin, 1);
        gpio_put(pin->m1pin, 1);
        break;
    default:
        printf("lora: invaild mode\n");
        break;
    }
}

void lora_config(uart_inst_t *uart, e220_config_t *config, e220_pin_t *pin) {
    uint8_t buf[15];

    buf[0] = 0xc0; // set register
    buf[1] = 0x00;
    buf[2] = 8;
    buf[3] = (config->addr) >> 8;
    buf[4] = (config->addr) & 0xff;
    buf[5] = (config->reg0);
    buf[6] = (config->reg1);
    buf[7] = (config->reg2);
    buf[8] = (config->reg3);
    buf[9] = (config->enckey) >> 8;
    buf[10] = (config->enckey) & 0xff;
    lora_mode(pin, 3);
    sleep_ms(200);
    while (uart_is_readable(uart)) uart_getc(uart);
    while (gpio_get(pin->auxpin) == 0) sleep_ms(1);
    uart_write_blocking(uart, buf, 11);
    for (int8_t i = 0; i <= 11; i++) printf("%x ", buf[i]);
    printf("\n");

    sleep_ms(1000);
    while (gpio_get(pin->auxpin) == 0) sleep_ms(1);
    while (uart_is_readable(uart)) printf("%c", uart_getc(uart));
    lora_mode(pin, 0);

    printf("lora: done config\n");
}  

void lora_send(uart_inst_t *uart, uint8_t *src, int8_t len) {
    // only transparent mode
#if 0
    uint16_t target = 0xffff;
    uint8_t chan = 0;
    uart_putc(uart, target >> 8);
    uart_putc(uart, target & 0xff);
    uart_putc(uart, chan);
#endif
    for (int8_t i = 0; i < len; i++) uart_puts(uart, &src[i]);
    printf("send:\n");
    for (int8_t i = 0; i < len; i++) printf("%c", src[i]);
    printf("\n");
}

void lora_receive(uart_inst_t *uart, uint8_t *dst) {
    //printf("recieve: \n");
    int8_t i = 0;
    while (uart_is_readable(uart)) {
        //printf("receive!");
        //uart_read_blocking(uart, &dst[i], 1);
        printf("%c", dst[i] = uart_getc(uart0));
        i++;
    }
    
    //for (int8_t j = 0; j < i; j++) printf("%c", dst[i]);
    //printf("\n");
}