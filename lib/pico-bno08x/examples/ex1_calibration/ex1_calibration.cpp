#include <stdio.h>
#include "pico/stdlib.h"
#include "../../src/pico-bno08x.h"

#define I2C_BUS i2c0
#define SDA 12
#define SCL 13

#define INTERVAL_MS 90
#define CONST_180_DIVIDED_BY_PI 57.2957795130823

BNO080 bno08x;
float rot_euler[3]; // yaw, roll, pitch
float mag[3]; // x, y, z

int main(void) {
    stdio_init_all();
    sleep_ms(2000);
    printf("\nhello, this is ex0_rotation_vector\n");

    i2c_init(I2C_BUS, 400 * 1000);
    gpio_set_function(SDA, GPIO_FUNC_I2C);
    gpio_set_function(SCL, GPIO_FUNC_I2C);
    //bno08x.enableDebugging();

    bool status = false;
    status = bno08x.begin(0x4a, I2C_BUS, 255);
    if (!status) printf("failed to initialize BNO085\n");

    bno08x.calibrateAll();
    bno08x.enableRotationVector(INTERVAL_MS);
    bno08x.enableMagnetometer(INTERVAL_MS);

    if (bno08x.getFeatureResponseAvailable()) {
        if (bno08x.checkReportEnable(SENSOR_REPORTID_ROTATION_VECTOR, INTERVAL_MS))
            bno08x.printGetFeatureResponse();
        else printf("fail: checkReportEnable\n");

    } else printf("fail: getFeatureResponseAvailable\n");
    
    printf("Press 's' to save calibration data to flash\n");

    uint8_t c;
    while (1) {
        if ((c = getchar_timeout_us(5000)) && (c == 's')) {
            printf("Saving Calibration Data...\n");
            bno08x.saveCalibration();
            bno08x.requestCalibrationStatus();

            for (uint8_t i = 0; i < 100; i++) {
                if (bno08x.dataAvailable()) {
                    if (bno08x.calibrationComplete()) {
                        printf("Calibration data successfully stored\n");
                        sleep_ms(1000);
                        break;
                    }
                }
                sleep_ms(1);
            }
        }
        if (bno08x.dataAvailable()) {
            mag[0] = bno08x.getMagX();
            mag[1] = bno08x.getMagY();
            mag[2] = bno08x.getMagZ();
            uint8_t mag_acc = bno08x.getMagAccuracy();

            rot_euler[0] = (bno08x.getYaw()) * CONST_180_DIVIDED_BY_PI;
            rot_euler[1] = (bno08x.getRoll()) * CONST_180_DIVIDED_BY_PI;
            rot_euler[2] = (bno08x.getPitch()) * CONST_180_DIVIDED_BY_PI;
            uint8_t quat_acc = bno08x.getQuatAccuracy();

            printf("mag  : %.2f %.2f %.2f %d\n", mag[0], mag[1], mag[2], mag_acc);
            printf("euler: %.2f %.2f %.2f %d\n", rot_euler[0], rot_euler[1], rot_euler[2], quat_acc);
        }
    }
}