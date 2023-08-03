#include "ugps.h"

GPS::GPS(int32_t goal_lat, int32_t goal_long,
         int16_t lat_per_res, int16_t long_per_res,
         double(*fx)(double)) {
    // 単位：cm
    this->goal_latitude = goal_lat;
    this->goal_longitude = goal_long;
    this->long_per_res = long_per_res;
    this->lat_per_res = lat_per_res;
    this->fx = fx;
}

void GPS::setFx(double(*fx)(double)) {
    this->fx = fx;
}

static bool calc_flag = false;
static bool is_ready = false;
static void callout(nmeap_context_t *context, void *data, void *user_data) {
    is_ready = true;
    calc_flag = false;
}

static nmeap_context_t nmea;
static void on_uart1_rx() {
    while (uart_is_readable(uart1)) {
        uint8_t ch = uart_getc(uart1);
        nmeap_parse(&nmea, ch);
    }
}

void GPS::init(uint tx_pin, uint rx_pin) {
    // uart setting
    uart_init(uart1, 9600);
    gpio_set_function(tx_pin, GPIO_FUNC_UART);
    gpio_set_function(rx_pin, GPIO_FUNC_UART);
    uart_set_baudrate(uart1, 9600);
    uart_set_hw_flow(uart1, false, false);
    uart_set_format(uart1, 8, 1, UART_PARITY_NONE);
    //uart_set_fifo_enabled(uart1, true);
    uart_set_fifo_enabled(uart1, false);
    irq_set_exclusive_handler(UART1_IRQ, on_uart1_rx);
    irq_set_enabled(UART1_IRQ, true);
    uart_set_irq_enables(uart1, true, false);

    // nmeap setting
    nmeap_init(&nmea, NULL);
    nmeap_addParser(&nmea, "GNGGA", nmeap_gpgga, callout, &gga);
}

bool GPS::isReady() {
    bool ret = is_ready;
    is_ready = false;
    return ret;
}

int32_t GPS::getLongitude() {
    int32_t ret;
    ret = (int32_t)(gga.longitude * 1000000.0f);
    return ret;
}

int32_t GPS::getLatitude() {
    int32_t ret;
    ret = (int32_t)(gga.latitude * 1000000.0f);
    return ret;
}

void GPS::calc() {
    long_now = getLongitude();
    lat_now  = getLatitude();
    dx = ((goal_longitude - long_now) * long_per_res);//0.000001度で0.092m(京田辺)，0.085m(能代)より，単位メートル
    dy = ((goal_latitude - lat_now) * lat_per_res);//0.000001度で0.111m(111)より0.1

    //printf("lat, long: %d, %d\n", lat_now, long_now);
    printf("dx, dy: %d, %d\n", dx, dy);
    calc_flag = true;
}

float GPS::getDirection() {
    double direction;
    if (!calc_flag) calc();
    dy = (int32_t)((double)dy*fx(0.8));
    
    if (dx == 0 && dy == 0) direction = 0.0f;
    else direction = atan2(dx, dy);

    return (float)direction;
}

// ゴールまでの距離(m)を返す
float GPS::getDistance() {
    double distance;
    if (!calc_flag) calc();
    
    distance = (double)approxDistance(dx, dy) / 1000.0f;
    distance = ((distance < 0) ? (-distance) : distance);
    return (float)distance;
    //return (1.5f);

}

/*平方根を使わず2点間の距離を近似*/
//参考
//https://nowokay.hatenablog.com/entry/20120604/1338773843
//https://dora.bk.tsukuba.ac.jp/~takeuchi/?%E3%83%97%E3%83%AD%E3%82%B0%E3%83%A9%E3%83%9F%E3%83%B3%E3%82%B0%2F%E5%B9%B3%E6%96%B9%E6%A0%B9%E3%82%92%E4%BD%BF%E3%82%8F%E3%81%9A%E3%81%AB%E8%B7%9D%E9%9B%A2%E3%82%92%E6%B1%82%E3%82%81%E3%82%8B
int32_t GPS::approxDistance(int32_t dx, int32_t dy) {
    int32_t min, max, approx;

    if (dx < 0) dx = -dx;
    if (dy < 0) dy = -dy;

    if (dx < dy) {
        min = dx;
        max = dy;
    } else {
        min = dy;
        max = dx;
    }

    approx = (max * 983) + (min * 407);
    if (max < (min << 4)) approx -= ( max * 40 );

    // add 512 for proper rounding
    return ((approx + 512) >> 10);
} 
