#include <stdio.h>
#include <math.h>

#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/pwm.h"
#include "hardware/uart.h"
#include "hardware/adc.h"
#include "hardware/pio.h"
#include "hardware/gpio.h"

#include "../lib/kaguya-pin/kaguya-pin.h"
#include "../lib/gnss/nmeap-0.3/inc/nmeap.h"
#include "../lib/health_monitor/src/health.h"
//#include "../lib/motor_encoder/src/motor.h"
#include "../lib/pico-bno08x/pico-bno08x.cpp"
#include "../lib/pico-e220/src/pico-e220.h"
#include "../lib/pico-eeprom-i2c/src/eeprom.h"
#include "../lib/pico-spl06-001/src/pico-spl06-001.h"
#include "../lib/vl53l5cx/api/vl53l5cx_api.h"
#include "util.c"
#include "umotor.h"

float current, voltage;
static repeating_timer_t timer;
static uint32_t logh = 0;

enum {
    MODE_WAIT = 1,
    MODE_LANDING = 2,
    MODE_GNSS = 3,
    MODE_TOF = 4,
    MODE_GOAL = 5,
    MODE_FORWARD_LANDING = 6,
    MODE_FORWARD_TOF = 7
};

// SETTINGS
// 白黒 COM5
const int32_t goal_longitude = 130960119;
const int32_t goal_latitude  =  30374279;

// 初期モード
static uint mode_now = MODE_WAIT;

const int32_t wait_count = 3 * 100; // 30秒
const float angle_th = 0.39; //前45度，GPS誘導
const uint16_t long_per_res = 111; // 種子島
const uint16_t lat_per_les = 96;

// motor
Motor motor;

// imu
BNO080 bno08x;

// tof
VL53L5CX_Configuration tof_dev;
VL53L5CX_ResultsData tof_res;
static uint8_t tof_isready;

// prs
spl06_config_t config = {0x77, i2c0, BGD_PRS_TEMP, {2, 64}, {2, 8}};
spl06_coef_t coef;

// gps
nmeap_context_t nmea;
nmeap_gga_t gga;
static bool gps_isready = false;

// motor
PIO pio = pio0;
uint32_t delta[2];

bool update_all(repeating_timer_t *rt);
bool is_goal();
void calc_gps(float *direction, int32_t *distance, int32_t long_now, int32_t lat_now);
//void calc_gps(float *dir, float *dist, double lon, double lat);

#if 1
//int16_t elog_ret;
void elog(uint8_t *src, int16_t len) {
    if (logh+8+len > 3000) return;
    uint8_t t[100] = {'\0'};
    snprintf(t, 8+len, "%06d,%s", gga.time, src);
    //printf("%s", t);
    for (int16_t i = 0; i < (len+8); i++) {
        logbuf[i+logh] = t[i];
    }
    //elog_ret = eeprom_write_multi(i2c1, 0x50, logh, t, len+7);
    logh += 7;
    logh += len;
}
#endif

void on_uart_rx() {
    while (uart_is_readable(uart1)) {
        uint8_t ch = uart_getc(uart1);
        nmeap_parse(&nmea, ch);
    }
}

// motor
int16_t limit_pwm_1023(int16_t pwm) {
    pwm = constrain(pwm, -1023, 1023);
    return pwm;
}

// gps
static void gpgga_callout(nmeap_context_t *context, void *data, void *user_data) {
    gps_isready = true;
}

#if 1
void calc_gps(float *direction, int32_t *distance, int32_t long_now, int32_t lat_now) {
    int32_t dx = ((goal_longitude - long_now) * long_per_res);//0.000001度で0.092m(京田辺)，0.085m(能代)より，単位メートル
    int32_t dy = ((goal_latitude - lat_now) * lat_per_les);//0.000001度で0.111m(111)より0.1
    
    if (dx == 0 && dy == 0) *direction = 0;
    else *direction = atan2(dx, dy);//意図的にdx, dyの順，というのも，北基準だから．
    *distance = approx_distance(dx, dy) / 10;//単位:cm
    *distance = abs(*distance);
    printf("calc_gps: dx %d, dy %d, dir %f, dist %d\n", dx, dy, *direction, *distance);
    printf("long_now %d, lat_now %d, long_goal %d, lat_goal %d\n", long_now, lat_now, goal_longitude, goal_latitude);
}
#endif

void mode_wait() {
    static bool is_removed = false;
    static int32_t cnt = 0;
    static bool isfirst = true;
    static uint8_t str[] = "fpin removed\n";
    static uint8_t str2[] = "wait end\n";

    if (cnt > wait_count) {
        mode_now = MODE_LANDING;
        printf("cnt > wait_count, cnt: %d\n", cnt);
        elog(str2, 9);
        return;
    }
    
    // 気圧が高くなっていっていく→一定の範囲内に落ち着いたら落下判定
}

void mode_landing() {
    static bool isfirst = true;
    static uint8_t str[]  = "nichr heat st\n";
    static uint8_t str2[] = "nichr heat end\n";
    if (isfirst) {
        isfirst = false;
        elog(str, 14);
    }
    printf("mode_landing\n");
    static int32_t count = 0;
    gpio_put(nichrome_pin, 1);

    count++;
    if (count > 150) {
        gpio_put(nichrome_pin, 0);
        printf("nichrome end\n");
        elog(str2, 15);
        mode_now = MODE_FORWARD_LANDING;
    }

}

void mode_forward_landing() {
    static uint8_t str[] = "mf 2s\n";
    static int32_t cnt = 0;

    motor_rotate(slice_left, 1023);
    motor_rotate(slice_right, -1023);

    if (cnt > 200) {
        elog(str, 6);
        motor_rotate(slice_left, 0);
        motor_rotate(slice_right, 0);
        mode_now = MODE_GNSS;
    }
    cnt++;
}

void mode_gnss() {
    static uint8_t str[] = "gnss\n";
    static uint8_t str2[20];
    static uint8_t str3[11];
    static bool isfirst = true;
    printf("mode_gnss\n");
    static float dir, yaw;
    static int32_t dist;
    static int16_t times = 0;
    static int16_t ttimes = 0;
    static float error[2] = {0}, integral = 0;

    if (isfirst) {
        isfirst = false;
        elog(str, 5);
    }

    if (gps_isready) {
        gps_isready = false;
        printf("gps_isready: true, %.6f,%.6f, sat %d\n", gga.latitude, gga.longitude, gga.satellites);
        int32_t lon = (int32_t)(gga.longitude * 1000000);
        int32_t lat = (int32_t)(gga.latitude * 1000000);
        calc_gps(&dir, &dist, lon, lat);
        if (dist < 70) {
            mode_now = MODE_FORWARD_TOF;
        }
        snprintf(str2, 19, "%d,%d\n", lat, lon);
        elog(str2, 18);
        //calc_gps(&dir, &dist, gga.longitude, gga.latitude);
        //if (gga.longitude == 0.0) is_valid = false;
    }
    if ((times == 10)) {
        times = 0;
        yaw = icm20948_calc_yawmag();
        int16_t l, r;
        //printf("yaw %f dir %f dir-th %f dir+t %f\n", yaw, dir, dir-angle_th, dir+angle_th);
        if ((yaw > dir-angle_th) && (yaw < dir+angle_th)) {
            l = 900; r = 900;
            printf("straight\n");
            motor.forward();
        } else if (yaw < (dir-angle_th)) {
            l = 900; r = 650;
            // 右旋回
            printf("right\n");
            motor.rightM();
        } else if (yaw > (dir+angle_th)) {
            // 左旋回
            //pwm = -pwm;
            printf("left\n");
            l = 650; r = 900;
            motor_rotate(slice_left, 650);
            motor_rotate(slice_right, 900);
        } else {
            printf("sikatanaku right\n");
            motor_rotate(slice_left, 900);
            motor_rotate(slice_right, 650);
        }
#if 1
        if (ttimes == 10) {
            ttimes = 0;
            snprintf(str3, 11, "mr,%d%d\n", l, r);
            elog(str3, 10);
        }
        ttimes++;
#endif
    }
    times++;
}

void mode_forward_tof() {
    static int32_t cnt = 0;

    motor_rotate(slice_left, 800);
    motor_rotate(slice_right, 800);

    if (cnt > 200) {
        motor_rotate(slice_left, 0);
        motor_rotate(slice_right, 0);
        mode_now = MODE_TOF;
    }
    cnt++;
}

void mode_tof() {

}

void mode_goal() {
    cancel_repeating_timer(&timer);
    static bool isfirst = true;
    static uint8_t str[] = "goal!\n";
    if (isfirst) {
        isfirst = false;
        elog(str, 6);
    }

    eeprom_write_multi(i2c1, 0x50, 0x0000, logbuf, logh);
    printf("mode_goal\n");
    while(1) tight_loop_contents();
}

// overall
bool update_all(repeating_timer_t *rt) {
    static float accel_g[3] = {0}, gyro_dps[3] = {0};
    uint8_t buf[11];
    icm20948_read_all();
    icm20948_calc_data(accel_g, gyro_dps);
    MadgwickAHRSupdateIMU(&filter, gyro_dps[0], gyro_dps[1], gyro_dps[2], accel_g[0] * 9.8, accel_g[1] * 9.8, accel_g[2] * 9.8);
    q2e(&filter, euler);

    switch (mode_now)
    {
    case MODE_WAIT:
        mode_wait();
        break;
    case MODE_LANDING:
        mode_landing();
        break;
    case MODE_FORWARD_LANDING:
        mode_forward_landing();
        break;
    case MODE_GNSS:
        mode_gnss();
        break;
    case MODE_FORWARD_TOF:
        mode_forward_tof();
        break;
    case MODE_TOF:
        mode_tof();
        break;
    case MODE_GOAL:
        mode_goal();
        break;
    }
}

int main(void) {
    stdio_init_all();

    // gnss on
    gpio_init(gnss_vcc_pin);
    gpio_set_dir(gnss_vcc_pin, GPIO_OUT);
    gpio_put(gnss_vcc_pin, 0);

    // i2c0: pres, imu
    // BNO085
    bool bno_status = false;
    bno_status = bno08x.begin(0x4a, i2c0, 255);
    if (!bno_status) printf("failed to initialize BNO085\n");
    bno08x.enableRotationVector(INTERVAL_MS);
    if (bno08x.getFeatureResponseAvailable()) {
        if (bno08x.checkReportEnable(SENSOR_REPORTID_ROTATION_VECTOR, INTERVAL_MS))
            bno08x.printGetFeatureResponse();
        else printf("fail: checkReportEnable\n");

    } else printf("fail: getFeatureResponseAvailable\n");

    // SPL06-001
    spl06_init(&prs_config, &prs_coef);

    // i2c1: tof, eeprom
    // vl53l5cx
    gpio_init(tof_vcc_pin);
    gpio_set_dir(tof_vcc_pin, GPIO_OUT);
    gpio_put(tof_vcc_pin, 0);
    sleep_ms(100);

    i2c_init(&i2c1, 1000 * 1000);
    gpio_set_function(i2c1_scl_pin, GPIO_FUNC_I2C);
    gpio_set_function(i2c1_sda_pin, GPIO_FUNC_I2C);

    uint8_t tof_status, is_ready, is_alive;

    tof_dev.platform.address = 0x29;
    tof_dev.platform.i2c     = &vl53l5cx_i2c;
    tof_status = vl53l5cx_is_alive(&tof_dev, &is_alive);
    tof_status = vl53l5cx_init(&tof_dev);
    if (tof_status) printf("vl53l5cx: ULD load failed\n");
    if (!tof_status) printf("vl53l5cx: OK\n");
    else printf("vl53l5cx: NG\n");
    // 8x8
    tof_status = vl53l5cx_set_resolution(&tof_dev, VL53L5CX_RESOLUTION_8X8);
    if (tof_status) printf("err: set_resolution\n");
    // 5Hz
    tof_status = vl53l5cx_set_ranging_frequency_hz(&tof_dev, 15);
    if (tof_status) printf("err: set_ranging_frequency_hz\n");
    // strongest -> closest
    tof_status = vl53l5cx_set_target_order(&tof_dev, VL53L5CX_TARGET_ORDER_CLOSEST);
    if (tof_status) printf("err: set_target_order\n");
    // continuous
    tof_status = vl53l5cx_set_ranging_mode(&tof_dev, VL53L5CX_RANGING_MODE_CONTINUOUS);
    if (tof_status) printf("err: set_ranging_mode\n");
    tof_status = vl53l5cx_start_ranging(&tof_dev);
    if (tof_status) printf("err: start_ranging\n");

    // health check
    health_init(current_sense_pin, voltage_sense_pin);
    current = health_current_read(current_sense_pin);
    voltage = health_voltage_read(voltage_sense_pin);
    printf("health: %f V, %f A\n", voltage, current);

    // nichrome
    gpio_init(nichrome_pin);
    gpio_set_dir(nichrome_pin, GPIO_OUT);

    // led
    gpio_init(led_pin);

    // motor, encoder
    motor.init();

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

    add_repeating_timer_ms(-10, &update_all, NULL, &timer);

    while (1) {
        tight_loop_contents();
    }
}