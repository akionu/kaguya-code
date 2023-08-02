// tofのテスト

#include <stdio.h>
#include <math.h>

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
#include "../lib/pico-eeprom-i2c/src/eeprom.h"
#include "../ulib/umotor/umotor.h"
#include "../ulib/uprs/uprs.h"
#include "../ulib/ugps/ugps.h"
//#include "../ulib/utof/utof.h"

#include "config.h"

extern "C" {
    #include "../lib/health_monitor/src/health.h"
    #include "../lib/pico-e220/src/pico-e220.h"
    //#include "../lib/pico-spl06-001/src/pico-spl06-001.h"
    #include "../lib/pico-vl53l5cx/api/vl53l5cx_api.h"
    #include "util.h"
}
#define CONST_180_DIVIDED_BY_PI 57.2957795130823

repeating_timer_t timer;

Motor motor;
Press prs;
BNO080 bno08x;
// 34.801604, long: 135.770988
GPS gps(
    34801613, // goal_lat
    135770988, // goal_long
    111, 92, // kyotanabe
    [](double x) {return (x*x);}
    );
//Tof tof;
VL53L5CX_Configuration dev;
VL53L5CX_ResultsData res;

bool bnoIsEnoughAccuracy() {
    double accq = bno08x.getQuatRadianAccuracy();
    double accd = accq * CONST_180_DIVIDED_BY_PI;
    printf("acc: rad: %f, deg: %f\n", accq, accd);
    if (accd < 15.0f) return true;
    else return false;
}

void gradient(uint8_t in[64], uint8_t out[64], double amp) {
    static float cx[9] = {0, 0, 0,
                            0, 1, 0,
                            0, 0, -1};
    static float cy[9] = {0, 0, 0,
                            0, 0, 1,
                            0, -1, 0};

    int16_t d[9];
    float xx, yy, zz;

    for (int8_t i = 8; i < 8; i+=8) {
	    for (int8_t j = 1; j < 7; j++) {
			d[0] = in[(i-8)+(j-1)];
			d[1] = in[(i-8)+(j)];
			d[2] = in[(i-8)+(j+1)];
			d[3] = in[(i)+(j-1)];
			d[4] = in[(i)+(j)];
			d[5] = in[(i)+(j+1)];
			d[6] = in[(i+8)+(j-1)];
			d[7] = in[(i+8)+(j)];
			d[8] = in[(i+8)+(j+1)];
		    xx = (float)(cx[0]*d[0] + cx[1]*d[1] + cx[2]*d[2]
					   + cx[3]*d[3] + cx[4]*d[4] + cx[5]*d[5]
					   + cx[6]*d[6] + cx[7]*d[7] + cx[8]*d[8]);
			yy = (float)(cy[0]*d[0] + cy[1]*d[1] + cy[2]*d[2]
					   + cy[3]*d[3] + cy[4]*d[4] + cy[5]*d[5]
					   + cy[6]*d[6] + cy[7]*d[7] + cy[8]*d[8]);
			zz = (float)(amp*sqrt(xx*xx+yy*yy));
			double dat = (int)zz;
			if(dat > 255) dat = 255;
			out[i+j]= (uint8_t)dat;
		}
	}
}

uint8_t median_value(uint8_t d[9])
{
	uint8_t     i, j, tmp;
    
	for (j = 0; j < 8; j++) {
		for (i = 0; i < 8; i++) {
			if (d[i+1] < d[i]) {
				tmp = d[i+1];
				d[i+1] = d[i];
				d[i] = tmp;
			}
		}
	}
	return d[4];
}

void median(uint8_t in[64], uint8_t out[64])
{
	int             i, j;
  	uint8_t   d[9];

    for (int8_t i = 8; i < 8; i+=8) {
	    for (int8_t j = 1; j < 7; j++) {
			d[0] = in[(i-8)+(j-1)];
			d[1] = in[(i-8)+(j)];
			d[2] = in[(i-8)+(j+1)];
			d[3] = in[(i)+(j-1)];
			d[4] = in[(i)+(j)];
			d[5] = in[(i)+(j+1)];
			d[6] = in[(i+8)+(j-1)];
			d[7] = in[(i+8)+(j)];
			d[8] = in[(i+8)+(j+1)];
			out[i+j] = median_value(d);
		}
	}
}

void expand(uint8_t in[64], uint8_t out[64]) {
    uint8_t max = in[0], min = in[0];
    double d;
    for (int i = 0; i < 64; i++) {
        if (in[i] > max) max = in[i];
        if (in[i] < min) min = in[i];
    }
    for (int i = 0; i < 64; i++) {
        d = 255.0 / ((double)max - (double)min) * ((double)in[i] - (double)min);
        if (d > 255.0) out[i] = 255;
        else if (d < 0.0) out[i] = 0;
        else out[i] = (int)d;
    }
}

void showval(uint8_t mat[64]) {
    for (uint8_t i = 1; i <= 64; i++) {
        printf("%4d", mat[i-1]);
        if (i % 8 == 0) printf("\n");
    }
}

bool isCone(uint8_t in[64]) {
    uint8_t cnt = 0;
    uint16_t avg = 0;
    double s = 0;
    for (int8_t i = 0; i < 64; i++) {
        avg += in[i];
    }
    avg /= 64;

    for (int8_t i = 0; i < 64; i++) {
        s += (double)((in[i]-avg)*(in[i]-avg));
    }
    s /= 64.0f;

    printf("avg: %d, s: %f\n", avg, s);
    if (s < 600) return true;
    else return false;
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
            for (int8_t i = 1; i < 10; i++) {
                alt_change[i-1] = alt_change[i];
            }
            alt_change[9] = prs.getAlt();
            printf("alt: %f\n", alt_change[9]);
            printf("rise: %d, fall: %d\n", isDetectRise, isDetectFall);
            for (int i = 0; i < 10; i++) printf("%3.2f ", alt_change[i]);
            printf("\n");

            if (!isDetectRise) {
                if (alt_change[0] >= 3000) return MODE_LANDING;
                int8_t cnt = 0;
                for (int8_t i = 1; i < 10; i++) {
                    // さっきより高度が高い=より上にいる
                    if ((alt_change[i] - alt_change[i-1]) > 0.05) cnt++;
                    //printf("isDetectRise: %d\n", cnt);
                }
                if (cnt > 5) isDetectRise = true;
            }
            if (isDetectRise && !isDetectFall) {
                if (alt_change[0] >= 1000) return MODE_LANDING;
                int8_t cnt = 0;
                for (int8_t i = 1; i < 10; i++) {
                    // さっきより高度が低い=より下にいる（地面に近い）
                    if ((alt_change[i-1] - alt_change[i]) > 0.05) cnt++;
                }
                if (cnt > 5) isDetectFall = true;
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
                if (s < 0.08) return MODE_FORWARD_LANDING;
            } else {
                landingCnt = 0;
                // 300s
                if (landingCnt > (300*SEC2CNT)) {
                    if (abs(alt_change[9]-alt_ref) < 10) {
                        return MODE_FORWARD_LANDING;
                    } else if (landingCnt > (500*SEC2CNT)) {
                        return MODE_FORWARD_LANDING;
                    }
                } else {
                    landingCnt++;
                }
            }
            return MODE_LANDING;
        }
        float alt_ref;

        int8_t expansion() {
            motor.backward(1023);
            if (expansionCnt > (12*SEC2CNT)) {
                printf("enpansion done\n");
                motor.stop();
                expansionCnt = 0;
                return MODE_FORWARD_LANDING;
            } else {
                expansionCnt++;
                return MODE_EXPANSION;
            }
        }

        int8_t forwardLanding() {
            motor.forward(1023);
            //printf("cnt: %d\n", cnt);
            if (expansionCnt > (5*SEC2CNT)) {
                printf("forwardLanding done\n");
                motor.stop();
                expansionCnt = 0;
                return MODE_GNSS;
            } else {
                expansionCnt++;
                return MODE_FORWARD_LANDING;
            }
        }

        int8_t gnss() {
            static int8_t cnt = 10;
            if (gps.isReady() && bno08x.dataAvailable()) {
                
                if (!bnoIsEnoughAccuracy()) {
                    printf("low acc: left\n");
                    motor.leftM();
                    return MODE_GNSS;
                }
                gps.calc();
                //printf("gps ready\n");
                float dist = gps.getDistance();
                if (dist < 1.0f) {
                    return MODE_FORWARD_TOF;
                }
                if (dist < 3.0f) {
                    gps.setFx([](double x) {return x;});
                }
                float dir = gps.getDirection();
                float yaw = -bno08x.getYaw();
                printf("dist: %f, dir: %f, yaw: %f\n", dist, dir, yaw);
                if ((yaw > dir-angle_th) && (yaw < dir+angle_th)) {
                    printf("forward\n");
                    motor.forward(1023);
                } 
                else if (yaw < (dir-angle_th)) {
                    //printf("rightM\n");
                    motor.rightM();
                } else if (yaw > (dir+angle_th)) {
                    printf("leftM\n");
                    motor.leftM(); 
                } else {
                    printf("sikatanaku rightM\n");
                    motor.rightM();
                }
            }
            return MODE_GNSS;
        }
        
        int8_t forwardTof() {
            motor.forward(1023);
            if (expansionCnt > (3*SEC2CNT)) {
                printf("forwardTof done\n");
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
        }

        int8_t tofm() {
            static bool is_first = true;
            static int16_t trycnt = 0;
            bool is_cone = false;
                static uint8_t mat[64] = {0};
    static uint8_t mat2[64] = {0};
    static uint16_t matbuf[64] = {0};

            printf("tofm\n");

            if (trycnt > (300*SEC2CNT)) {
                printf("trycnt over th\n");
                return MODE_GOAL;
            } else {
                trycnt++;
            }

            //return MODE_TOF; // ok
            if (is_first) {
                printf("start tof ranging\n");
                is_first = false;
                uint8_t status = vl53l5cx_start_ranging(&dev);
                if (status) printf("err: start_ranging\n");
            } else {
                motor.stop();
                //return MODE_TOF; // ok
#if 1
                if (bno08x.dataAvailable()) {
                    printf("bno av\n");
                    
                    float pitch = bno08x.getPitch() * CONST_180_DIVIDED_BY_PI;
                    if ((pitch > 0.0f) && (pitch < 20.0f)) motor.forward(1023);
                    else if (pitch > 20.0f) motor.backward(1023);
                    //sleep_ms(200);
                    
                    //return MODE_TOF;
                }

                int8_t cnt = 0;
                while (cnt < 3) {
                uint8_t is_ready;
                uint8_t status = vl53l5cx_check_data_ready(&dev, &is_ready);
                if (is_ready) {
                    printf("ready\n");
                    vl53l5cx_get_ranging_data(&dev,&res);
                    for (int i = 0; i < 64; i++) matbuf[i] += (uint8_t)(res.distance_mm[i] / 8);

                    for (uint8_t i = 0; i < 64; i++) mat[i] = (uint8_t)(matbuf[i]);
                    expand(mat, mat2);
                    memcpy(mat2, mat, 64);
                    // median
                    median(mat, mat2);
                    // 1次微分によるエッジ抽出
                    gradient(mat2, mat, 1.1);
                    //memcpy(mat2, mat, 64);
                    showval(mat);
                    printf("isCone: %d\n", isCone(mat));
                    for (int i = 0; i < 64; i++) matbuf[i] = 0;
                    for (int i = 0; i < 64; i++) mat[i] = 0;
                    cnt++;
                }
                sleep_ms(10);
                }
            
#endif
                
            }
            return MODE_TOF;
        }

        int8_t goal() {
            static bool is_goal = false;
            if (is_goal) {
                motor.stop();
                return MODE_GOAL;
            }
            if (gps.isReady()) {
                if (gps.getDistance() < 5.0f) {
                    printf("goal\n");
                    is_goal = true;
                } else {
                    printf("onlyGnss\n");
                    isOnlyGnss = true;
                    return MODE_GNSS;
                }
            }
            return MODE_GOAL;
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

bool update() {
    uint32_t before = time_us_32();
    printf("mode_now: %d\n", mode_now);
    static int8_t cnt = 0;
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
    case MODE_EXPANSION:
        ret = mode.expansion();
        break;
    case MODE_FORWARD_LANDING:
        ret = mode.forwardLanding();
        break;
    case MODE_GNSS:
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
    }
    mode_now = ret;
    uint32_t et = time_us_32() - before;
    printf("elapsed: %d ms\n", et / 1000);
    return true;
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
    //tof.on(tof_vcc_pin);
    //tof.init();
    // power on
    gpio_init(tof_vcc_pin);
    gpio_set_dir(tof_vcc_pin, GPIO_OUT);
    gpio_put(tof_vcc_pin, 0);
    uint8_t status, is_ready, is_alive;
    dev.platform.address = 0x29;
    dev.platform.i2c     = i2c1;

    status = vl53l5cx_is_alive(&dev, &is_alive);
    if (!is_alive || status) printf("err: sensor not detected\n");

    status = vl53l5cx_init(&dev);
    if (status) printf("ULD load failed\n");

    // 8x8
    status = vl53l5cx_set_resolution(&dev, VL53L5CX_RESOLUTION_8X8);
    if (status) printf("err: set_resolution\n");
    // 5Hz
    status = vl53l5cx_set_ranging_frequency_hz(&dev, 15);
    if (status) printf("err: set_ranging_frequency_hz\n");
    // strongest
    status = vl53l5cx_set_target_order(&dev, VL53L5CX_TARGET_ORDER_STRONGEST);
    if (status) printf("err: set_target_order\n");
    // continuous
    status = vl53l5cx_set_ranging_mode(&dev, VL53L5CX_RANGING_MODE_CONTINUOUS);
    if (status) printf("err: set_ranging_mode\n");

    // health check
    health_init(current_sense_pin, voltage_sense_pin);
    float current = health_current_read(current_sense_pin);
    float voltage = health_voltage_read(voltage_sense_pin);
    printf("health: %f V, %f A\n", voltage, current);

    // nichrome
    gpio_init(nichrome_pin);
    gpio_set_dir(nichrome_pin, GPIO_OUT);

    // led
    gpio_init(led_pin);

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

    //add_repeating_timer_ms(-500, &update, NULL, &timer);
    while (1) {
        update();
        sleep_ms(200);
    }   
}