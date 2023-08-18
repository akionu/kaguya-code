#include <stdio.h>
#include "pico/stdlib.h"
#include "../../lib/kaguya-pin/kaguya-pin.h"
#include "../../ulib/umotor/umotor.h"
#include "../../lib/pico-bno08x/src/pico-bno08x.h"
#define CONST_180_DIVIDED_BY_PI 57.2957795130823
#define mabs(x) ((x<0)?(-x):(x))
BNO080 bno08x;
Motor motor;

repeating_timer_t timer;
bool rt_flag = false;

bool rtCallback(repeating_timer_t* rt) {
    rt_flag = true;
    return true;
}

void control(double dir) {
    float th = 0.2;
    if (bno08x.dataAvailable()) {
        double yaw = -bno08x.getYaw();
        double ytmp = 0;
        //double ytmp = (dir/(double)M_PI) - (yaw/(double)M_PI);
#if 0
        ytmp = (dir) - (yaw);
        if ((-th < ytmp) && (ytmp < th)) {
            // forward
            //motor.forward(1023);
            motor.stop();
        }
        else if ((dir > 0) && (yaw > 0)) {
            if (dir > yaw) {
                printf("dir > yaw, right ");
                motor.rightH();
            }
            else {
                printf("dir < yaw, left ");
                motor.leftH();
            }
        } else if ((dir > 0) && (yaw < 0)) {
            if (mabs(M_PI-ytmp) < mabs(ytmp)) motor.rightH();
            else motor.leftH();
        } else if ((dir < 0) && (yaw > 0)) {
            if (mabs(M_PI+ytmp) < mabs(ytmp)) motor.leftH();
            else motor.rightH();
        } else if ((dir < 0) && (yaw < 0)) {
            if (dir < yaw) motor.leftH();
            else motor.rightH();
        }
#endif

#if 0
        ytmp = (dir) - (yaw);
        if (ytmp < 2*M_PI) ytmp += M_PI;
        else if (ytmp > 2*M_PI) ytmp -= M_PI;
        if ((-th+M_PI < ytmp) && (ytmp < th+M_PI)) {
            // forward
            //motor.forward(1023);
            motor.stop();
            printf("forward ");
        } else if (ytmp > M_PI) {
            motor.rightH();
            printf("right   ");
        } else if (ytmp < M_PI) {
            motor.leftH();
            printf("left    ");
        }
#endif

        if ((yaw > dir-th) && (yaw < dir+th)) {
            printf("straight ");
            motor.stop();
        } else if (yaw < (dir-th)) {
            // 右旋回
            printf("right    ");
            motor.rightM();
            sleep_ms(200);
            motor.stop();
        } else if (yaw > (dir+th)) {
            // 左旋回
            //pwm = -pwm;
            printf("left     ");
            motor.leftM();
            sleep_ms(200);
            motor.stop();
        } else {
            printf("s-right  ");
            motor.rightM();
            sleep_ms(200);
            motor.stop();
        }

        printf("yaw: %+03.2f, dir: %+03.2f, ytmp: %+03.2f, %s\n", 
            yaw * CONST_180_DIVIDED_BY_PI,
            dir * CONST_180_DIVIDED_BY_PI,
            ytmp * CONST_180_DIVIDED_BY_PI,
            (((-th < ytmp) && (ytmp < th)) ? "straight" : ((ytmp > 0) ? "right" : "left")));
    }
}

int main(void) {
    stdio_init_all();
    sleep_ms(2000);
    printf("hello, dir_forward!\n");

    // motor
    motor.init(motor_left_a_pin, motor_right_a_pin);
    //
    //
    motor.setDirForward(-1, +1);
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

    add_repeating_timer_ms(-500, &rtCallback, NULL, &timer);

    double yawtobe = -3.14;
    int16_t cnt = 0;
    while (1) {
        if (rt_flag) {
            rt_flag = false;
            if (cnt == 10*2) {
                // each 10 sec
                cnt = 0;
                yawtobe += 0.2; // about 10 deg
            } else {
                cnt++;
            }
            if (yawtobe > 3.14) yawtobe = -3.14;
            control(yawtobe);
        }
    }
}