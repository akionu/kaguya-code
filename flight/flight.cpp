#include <stdio.h>
#include <math.h>
#include <stdarg.h>

/*
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/pwm.h"
#include "hardware/uart.h"
#include "hardware/adc.h"
#include "hardware/pio.h"
#include "hardware/gpio.h"
*/

#include "../lib/kaguya-pin/kaguya-pin.h"
//#include "../lib/gnss/nmeap-0.3/inc/nmeap.h"
//#include "../lib/motor_encoder/src/motor.h"
#include "../lib/pico-bno08x/src/pico-bno08x.h"
//#include "../lib/pico-eeprom-i2c/src/eeprom.h"
#include "../ulib/umotor/umotor.h"
#include "../ulib/uprs/uprs.h"
#include "../ulib/ugps/ugps.h"
#include "../ulib/utof/utof.h"
#include "../ulib/ulog/ulog.h"

#include "config.h"

extern "C" {
    #include "../lib/health_monitor/src/health.h"
    //#include "../lib/pico-e220/src/pico-e220.h"
    //#include "../lib/pico-spl06-001/src/pico-spl06-001.h"
    //#include "../lib/pico-vl53l5cx/api/vl53l5cx_api.h"
    #include "util.h"
}
#define CONST_180_DIVIDED_BY_PI 57.2957795130823

repeating_timer_t timer;

Motor motor;
Press prs;
BNO080 bno08x;
// 34.801604, long: 135.770988
GPS gps(
    39716491, // goal_lat
    140128503, // goal_long
    111, 85, // noshiro
    [](double x) {return (x*x);}
    );
Tof tof;
Log logging('A');

uint8_t logbuf[12];
bool rt_flag = false;

bool bnoIsEnoughAccuracy() {
    double accq = bno08x.getQuatRadianAccuracy();
    double accd = accq * CONST_180_DIVIDED_BY_PI;
    printf("acc: rad: %f, deg: %f\n", accq, accd);
    if (accd < 15.0f) return true;
    else return false;
}

void addLogBuf(const char* fmt, ...) {
    va_list arg;
    va_start(arg, fmt);
    vsnprintf((char *)logbuf, 12, fmt, arg);
    va_end(arg);
}

class Mode {
    public:
        Mode() {
            // landing
            for (int i = 0; i < 10; i++) alt_change[i] = 3776.0f;
            isDetectRise = false; isDetectFall = false;
            landingCnt = 0;
            // expansion, forwardLanding
            expansionCnt = 0;
            isOnlyGnss = false;
        }
        int8_t landing() {
            bno08x.dataAvailable();
            for (int8_t i = 1; i < 10; i++) {
                alt_change[i-1] = alt_change[i];
            }
            alt_change[9] = prs.getAlt();
            printf("alt: %f\n", alt_change[9]);
            printf("rise: %d, fall: %d\n", isDetectRise, isDetectFall);
            for (int i = 0; i < 10; i++) printf("%3.2f ", alt_change[i]);
            printf("\n");

            if (!isDetectRise) {
                if (alt_change[0] >= 3000) {
                    addLogBuf("altNotReady");
                    return MODE_LANDING;
                }
                int8_t cnt = 0;
                for (int8_t i = 1; i < 10; i++) {
                    // さっきより高度が高い=より上にいる
                    if ((alt_change[i] - alt_change[i-1]) > 0.05) cnt++;
                    //printf("isDetectRise: %d\n", cnt);
                }
                if (cnt > 5) {
                    addLogBuf("c:%1d %3f rise", cnt, alt_change[9]);
                    isDetectRise = true;
                }
            }
            if (isDetectRise && !isDetectFall) {
                if (alt_change[0] >= 1000) return MODE_LANDING;
                int8_t cnt = 0;
                for (int8_t i = 1; i < 10; i++) {
                    // さっきより高度が低い=より下にいる（地面に近い）
                    if ((alt_change[i-1] - alt_change[i]) > 0.03) cnt++;
                }
                if (cnt > 5) {
                    addLogBuf("cnt:%1d %3f fall", cnt, alt_change[9]);
                    isDetectFall = true;
                }
                //return MODE_LANDING;
            } 
            if (isDetectRise && isDetectFall) {

                // 分散
                double avg = 0, s = 0;
                for (int8_t i = 0; i < 10; i++) {
                    avg += alt_change[i];
                }
                avg /= 10;
                for (int8_t i = 0; i < 10; i++) {
                    s += (alt_change[i]-avg)*(alt_change[i]-avg);
                }
                s /= 10;
                printf("alt: %f, s: %f\n", alt_change[9], s);
                if (s < 0.06) {
                    addLogBuf("s:%1.2f a:%.0f land", s, avg);
                    return MODE_NICHROME;
                }
            } else {
                //landingCnt = 0;
                // 300s
                if (landingCnt > (300*SEC2CNT)) {
                    if (abs(alt_change[9]-alt_ref) < 10) {
                        addLogBuf("la:%4d");
                        return MODE_NICHROME;
                    } else if (landingCnt > (500*SEC2CNT)) {
                        return MODE_NICHROME;
                    }
                } else {
                    landingCnt++;
                }
            }
            return MODE_LANDING;
        }
        float alt_ref;

        int8_t nichrome() {
            bno08x.dataAvailable();
            if (expansionCnt > (2*SEC2CNT)) {
                gpio_put(nichrome_pin, 0);
                addLogBuf("cnt:%d d", expansionCnt);
                printf("nichrome done\n");
                return MODE_EXPANSION;
            } else {
                gpio_put(nichrome_pin, 1);
                expansionCnt++;
                return MODE_NICHROME;
            }
        }

        int8_t expansion() {
            //motor.backward(1023);
            motor.forward(1023);
            bno08x.dataAvailable();
            if (expansionCnt > (40*SEC2CNT)) {
                // left: 317348 ms, right: 398766 ms
                addLogBuf("cnt:%d d", expansionCnt);
                printf("enpansion done\n");
                motor.stop();
                expansionCnt = 0;
                return MODE_FORWARD_LANDING;
            } else {
                addLogBuf("cnt:%d u", expansionCnt);
                expansionCnt++;
                return MODE_EXPANSION;
            }
        }

        int8_t forwardLanding() {
            motor.forward(1023);
            bno08x.dataAvailable();
            //printf("cnt: %d\n", cnt);
            if (expansionCnt > (5*SEC2CNT)) {
                addLogBuf("cnt:%d d", expansionCnt);
                printf("forwardLanding done\n");
                motor.stop();
                expansionCnt = 0;
                return MODE_GNSS;
            } else {
                addLogBuf("cnt:%d u", expansionCnt);
                expansionCnt++;
                return MODE_FORWARD_LANDING;
            }
        }

        int8_t gnss() {
            static int8_t cnt = 10;
            static float distlog[5] = {0};
            if (gps.isReady() && bno08x.dataAvailable()) {
                
                if (!bnoIsEnoughAccuracy()) {
                    printf("low acc: left\n");
                    motor.leftM();
                    return MODE_GNSS;
                }
                gps.calc();
                //printf("gps ready\n");
                float dist = gps.getDistance();
                for (int i = 0; i < 5-1; i++) distlog[i] = distlog[i+1];
                distlog[9] = dist;
                int8_t dcnt = 0;
                for (int i = 0; i < 5-1; i++) {
                    if (abs(distlog[i]-distlog[i+1])) dcnt++;
                }
                if (dcnt > 3) {
                    motor.backward(1023);
                    return MODE_GNSS;
                }
                if (dist < 2.5f) {
                    return MODE_FORWARD_TOF;
                }
                if (dist < 5.0f) {
                    gps.setFx([](double x) {return x;});
                }
                float dir = gps.getDirection();
                float yaw = -bno08x.getYaw();
                printf("dist: %f, dir: %f, yaw: %f\n", dist, dir, yaw);
                if ((yaw > dir-angle_th) && (yaw < dir+angle_th)) {
                    printf("forward\n");
                    addLogBuf("%2.1f %3.0f f", dist, dir);
                    motor.forward(1023);
                } 
                else if (yaw < (dir-angle_th)) {
                    //printf("rightM\n");
                    addLogBuf("%2.1f %3.0f r", dist, dir);
                    motor.rightM();
                } else if (yaw > (dir+angle_th)) {
                    addLogBuf("%2.1f %3.0f l", dist, dir);
                    printf("leftM\n");
                    motor.leftM(); 
                } else {
                    printf("sikatanaku rightM\n");
                    motor.rightM();
                }
            }
            return MODE_GNSS;
#if 0
            static int8_t cnt = 10;
            static float distlog[5] = {0};
            if (gps.isReady() && bno08x.dataAvailable()) {
                
                if (!bnoIsEnoughAccuracy()) {
                    printf("low acc: left\n");
                    motor.leftM();
                    return MODE_GNSS;
                }
                gps.calc();
                //printf("gps ready\n");
                float dist = gps.getDistance();
                for (int i = 0; i < 5-1; i++) distlog[i] = distlog[i+1];
                distlog[9] = dist;
                int8_t dcnt = 0;
                for (int i = 0; i < 5-1; i++) {
                    if (abs(distlog[i]-distlog[i+1])) dcnt++;
                }
                if (dcnt > 3) {
                    motor.backward(1023);
                    return MODE_GNSS;
                }
                if (dist < 2.5f) {
                    return MODE_FORWARD_TOF;
                }
                if (dist < 5.0f) {
                    gps.setFx([](double x) {return x;});
                }
                float dir = -gps.getDirection();
                float yaw = bno08x.getYaw();
                double ytmp = (double)dir - (double)yaw;
                printf("dist: %f, dir: %f, yaw: %f ytmp: %d\n", dist, dir, yaw, ytmp);
                if ((-angle_th < ytmp) && (ytmp < angle_th)) {
                    printf("forward\n");
                    addLogBuf("%2.1f %3.0f f", dist, dir);
                    motor.forward(1023);
                } 
                else if (ytmp > 0) {
                    printf("rightM\n");
                    addLogBuf("%2.1f %3.0f r", dist, dir);
                    motor.rightM();
                } else if (ytmp < 0) {
                    addLogBuf("%2.1f %3.0f l", dist, dir);
                    printf("leftM\n");
                    motor.leftM(); 
                } else {
                    printf("sikatanaku rightM\n");
                    motor.rightM();
                }
            }
            return MODE_GNSS;
#endif
        }
        
        int8_t forwardTof() {
#if 1
            static bool is_first = true;
            static float dir = 0.0f;
            float th = 0.2;

            if (is_first && gps.isReady()) {
                is_first = false;
                gps.calc();
                dir = -gps.getDirection();
            }
            if (expansionCnt > (10*SEC2CNT)) {
                printf("forwardTof done\n");
                addLogBuf("cnt:%d d", expansionCnt);
                motor.stop();
                expansionCnt = 0;
                if (isOnlyGnss) {
                    return MODE_GOAL;
                } else {
                    return MODE_TOF;
                }
            } else if (bno08x.dataAvailable()) {
                float yaw = bno08x.getYaw();
                float ytmp = dir - yaw;

                if ((-th < ytmp) && (ytmp < th)) {
                    // forward
                    motor.forward(1023);
                    printf("forward");
                } else if (ytmp > 0) {
                    motor.rightM();
                    printf("rightM");
                } else if (ytmp < 0) {
                    motor.leftM();
                    printf("leftM");
                }
                expansionCnt++;
                return MODE_FORWARD_TOF;
            }
            return MODE_FORWARD_TOF;
#endif
#if 0
            bno08x.dataAvailable();
            motor.forward(1023);
            if (expansionCnt > (6*SEC2CNT)) {
                printf("forwardTof done\n");
                addLogBuf("cnt:%d d", expansionCnt);
                motor.stop();
                expansionCnt = 0;
                if (isOnlyGnss) {
                    return MODE_GOAL;
                } else {
                    return MODE_TOF;
                }
            } else {
                expansionCnt++;
                return MODE_FORWARD_TOF;
            }
#endif
        }

        int8_t tofm() {
            static bool is_first = true;
            static int16_t trycnt = 0;
            bool is_cone = false;
            printf("tofm\n");

            if (trycnt > (300*SEC2CNT)) {
                addLogBuf("tcnt:%d d", trycnt);
                printf("trycnt over th\n");
                return MODE_GOAL;
            } else {
                trycnt++;
            }

            //return MODE_TOF; // ok
            if (is_first) {
                printf("start tof ranging\n");
                is_first = false;
                tof.start();
            } else {
                motor.stop();
                //return MODE_TOF; // ok
#if 1
                if (bno08x.dataAvailable()) {
                    printf("bno av\n");
                    bool needmove = false;
                    float pitch = bno08x.getPitch() * CONST_180_DIVIDED_BY_PI;
                    if (pitch < 2.0f) {
                        motor.backward(1023);
                        needmove = true;
                    } else if (pitch > 20.0f) {
                        motor.forward(1023);
                        needmove = true;
                    }
                    
                    if (needmove) {
                        //sleep_ms(200);
                        return MODE_TOF;
                    }
                }
#endif
                //return MODE_TOF; // NG
                // about 200ms
                int8_t cnt = 0;
                while (cnt < 3) {
                    if (tof.isCaptureReady()) {
                        printf("isCaptureReady!\n");
                        tof.capture();
                        
                        if (tof.isConeReady()) {
                            printf("tof isConeReady\n");
                            if (tof.isCone()) {
                                printf("cone!\n");
                                uint8_t cm = 0;
                                if ((cm = tof.getCenterCm()) < 15) {
                                    addLogBuf("c!c:%dcm");
                                    printf("close to cone!\n");
                                    motor.stop();
                                    return MODE_GOAL;
                                } else {
                                    addLogBuf("c!c:%dcm f");
                                    printf("forward\n");
                                    motor.forward(900);
                                }
                            } else {
                                addLogBuf("no cone l");
                                printf("no cone...left\n");
                                motor.leftH();
                            }
                        }
                        cnt++;
                    }
                    sleep_ms(10);
                }
                //return MODE_TOF;
                
            }
            return MODE_TOF;
        }

        int8_t goal() {
            static bool is_first = true;
            static bool is_goal = false;
            static uint8_t cnt = 0;

            bno08x.dataAvailable();
            if (cnt > 20*SEC2CNT) {
                return MODE_SHOWLOG;
            }
            if (is_first) {
                is_first = false;
            }
            if (is_goal) {
                motor.stop();
                return MODE_GOAL;
            }
            if (gps.isReady()) {
                float dist = 0;
                if ((dist = gps.getDistance()) < 5.0f) {
                    printf("d:%2.0fm ok", dist);
                    printf("goal\n");
                    is_goal = true;
                } else {
                    printf("onlyGnss");
                    printf("onlyGnss\n");
                    isOnlyGnss = true;
                    return MODE_GNSS;
                }
            }
            return MODE_GOAL;
        }

        int8_t showlog() {
            char c;
            printf("press s to show\n");
            while (1) {
                if ((c = getchar_timeout_us(1000)) == 's') {
                    logging.showAll();
                }
                sleep_ms(10);
            }
        }
    private:
        // landing
        float alt_change[10];
        bool isDetectRise;
        bool isDetectFall;
        uint16_t landingCnt;
        // expansion, fowardLanding
        uint16_t expansionCnt;
        // tof dame
        bool isOnlyGnss;
 } mode;

bool rtCallback(repeating_timer_t* rt) {
    rt_flag = true;
    return true;
}

void update() {
    uint32_t before = time_us_32();
    printf("mode_now: %d\n", mode_now);
    static int8_t cnt = 0;
    static bool islog = false;
    int ret = mode_now;
    switch (mode_now)
    {
    case MODE_WAIT:
        cnt++;
        if (cnt > 50) ret = MODE_LANDING;
        break;
    case MODE_LANDING:
        ret = mode.landing();
        break;
    case MODE_NICHROME:
        gpio_put(led_pin, 1);
        ret = mode.nichrome();
        break;
    case MODE_EXPANSION:
        ret = mode.expansion();
        break;
    case MODE_FORWARD_LANDING:
        ret = mode.forwardLanding();
        break;
    case MODE_GNSS:
        gpio_put(led_pin, 0);
        ret = mode.gnss();
        break;
    case MODE_FORWARD_TOF:
        ret = mode.forwardTof();
        break;
    case MODE_TOF:
        ret = mode.tofm();
        break;
    case MODE_GOAL:
        ret = mode.goal();
        break;
    case MODE_SHOWLOG:
        ret = mode.showlog();
        break;
    }
    mode_now = ret;
    if (islog) {
        logging.addLog(gps.getLatitude(), gps.getLongitude(),
                        bno08x.getYaw(), bno08x.getRoll(), bno08x.getPitch(),
                        mode_now,
                        logbuf);
    }
    islog = (islog ? (false) : (true));
    uint32_t et = time_us_32() - before;
    printf("elapsed: %d ms\n", et / 1000);
}

int main(void) {
    stdio_init_all();
    sleep_ms(2000);

    // gnss 1on
    gpio_init(gnss_vcc_pin);
    gpio_set_dir(gnss_vcc_pin, GPIO_OUT);
    gpio_put(gnss_vcc_pin, 0);
    
    // motor
    motor.init(motor_left_a_pin, motor_right_a_pin);
    //
    //
    motor.setDirForward(-1, 1);
    //
    //

    // i2c0: prs, imu
    i2c_init(i2c0, 400 * 1000);
    gpio_set_function(i2c0_sda_pin, GPIO_FUNC_I2C);
    gpio_set_function(i2c0_scl_pin, GPIO_FUNC_I2C);

    // bno085
    bool bno_status = false;
    bno_status = bno08x.begin(0x4a, i2c0, 255);
    if (!bno_status) printf("failed to initialize BNO085\n");
    bno08x.enableRotationVector(50);
    if (bno08x.getFeatureResponseAvailable()) {
        if (bno08x.checkReportEnable(SENSOR_REPORTID_ROTATION_VECTOR, 50))
            bno08x.printGetFeatureResponse();
        else printf("fail: checkReportEnable\n");

    } else printf("fail: getFeatureResponseAvailable\n");

    // prs
    prs.init();
    for (int8_t i = 0; i < 10; i++) {
        prs.getAlt();
        sleep_ms(50);
    }
    mode.alt_ref = prs.getAlt();

    // i2c1: eeprom, tof
    i2c_init(i2c1, 1000 * 1000);
    gpio_set_function(i2c1_sda_pin, GPIO_FUNC_I2C);
    gpio_set_function(i2c1_scl_pin, GPIO_FUNC_I2C);
    
    // tof
    tof.on(tof_vcc_pin);
    tof.init();

    // health check
    health_init(current_sense_pin, voltage_sense_pin);
    float current = health_current_read(current_sense_pin);
    float voltage = health_voltage_read(voltage_sense_pin);
    printf("health: %f V, %f A\n", voltage, current);

    // nichrome
    gpio_init(nichrome_pin);
    gpio_set_dir(nichrome_pin, GPIO_OUT);
    gpio_put(nichrome_pin, 0);

    // led
    gpio_init(led_pin);
    gpio_put(led_pin, 0);

    // gnss
    gps.init(gnss_tx_pin, gnss_rx_pin);

    printf("calibrate!\n");
    while (1) {
        if (bno08x.dataAvailable()) {
            if (bnoIsEnoughAccuracy()) break;
            //if (acc > 0) break;
            sleep_ms(10);
        }
    } 

    add_repeating_timer_ms(-500, &rtCallback, NULL, &timer);
    while (1) {
        if (rt_flag) {
            rt_flag = false;
            update();
        }   
    }
}