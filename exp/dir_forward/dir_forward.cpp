#include <stdio.h>
#include "pico/stdlib.h"
#include "../../lib/kaguya-pin/kaguya-pin.h"
#include "../../ulib/umotor/umotor.h"
#include "../../lib/pico-bno08x/src/pico-bno08x.h"
#define CONST_180_DIVIDED_BY_PI 57.2957795130823

BNO080 bno08x;
Motor motor;

repeating_timer_t timer;
bool rt_flag = false;

bool rtCallback(repeating_timer_t* rt) {
    rt_flag = true;
    return true;
}

void control(float yawtobe) {
    float yaw;
    float th = 0.2;
    if (bno08x.dataAvailable()) {
        yaw = bno08x.getYaw();
        float ytmp = yawtobe - yaw;

        if ((-th < ytmp) && (ytmp < th)) {
            // forward
            motor.forward(1023);
        } else if (ytmp > 0) {
            motor.rightH();
        } else if (ytmp < 0) {
            motor.leftH();
        }
        printf("yaw: %3.2f, dir: %3.2f, ytmp: %3.2f, %s\n", 
                    yaw * CONST_180_DIVIDED_BY_PI,
                    yawtobe * CONST_180_DIVIDED_BY_PI,
                    ytmp * CONST_180_DIVIDED_BY_PI,
                    ((ytmp < 0.2) ? "straight" : ((ytmp > 0) ? "right" : "left")));
                
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
    motor.setDirForward(1, -1);
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

    float yawtobe = -1.0f;
    while (1) {
        if (rt_flag) {
            control(yawtobe);
        }
    }
}