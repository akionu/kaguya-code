#include <stdio.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "../../src/pico-bno08x.h"

#define I2C_BUS i2c0
#define SDA 12
#define SCL 13

#define INTERVAL_MS 90
#define CONST_180_DIVIDED_BY_PI 57.2957795130823

BNO080 bno08x;
float euler[3]; // yaw, roll, pitch

int main(void) {
    stdio_init_all();
    sleep_ms(2000);
    printf("\nhello, this is ex0_rotation_vector\n");

    i2c_init(I2C_BUS, 400 * 1000);
    gpio_set_function(SDA, GPIO_FUNC_I2C);
    gpio_set_function(SCL, GPIO_FUNC_I2C);
    bno08x.enableDebugging();

#if 0
    gpio_init(14);
    gpio_set_dir(14, GPIO_IN);
    //gpio_pull_up(14);
#endif
    uint8_t buf;
    //if (i2c_read_blocking(I2C_BUS, 0x4a, &buf, 1, false) > 0) printf("detect BNO085!\n");
    //else printf("could not find BNO085...\n");

    bool status = false;
    status = bno08x.begin(0x4a, I2C_BUS, 255);
    if (!status) printf("failed to initialize BNO085\n");

    bno08x.enableRotationVector(INTERVAL_MS);

    if (bno08x.getFeatureResponseAvailable()) {
        if (bno08x.checkReportEnable(SENSOR_REPORTID_ROTATION_VECTOR, INTERVAL_MS))
            bno08x.printGetFeatureResponse();
        else printf("fail: checkReportEnable\n");

    } else printf("fail: getFeatureResponseAvailable\n");

    while (1) {
        if (bno08x.dataAvailable()) {
            euler[0] = (bno08x.getYaw()) * CONST_180_DIVIDED_BY_PI;
            euler[1] = (bno08x.getRoll()) * CONST_180_DIVIDED_BY_PI;
            euler[2] = (bno08x.getPitch()) * CONST_180_DIVIDED_BY_PI;

            printf("%.2f, %.2f, %.2f\n", euler[0], euler[1], euler[2]);
        }
    }
    
}