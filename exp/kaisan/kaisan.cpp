#include <stdio.h>
#include <string.h>
#include "../../lib/pico-bno08x/src/pico-bno08x.h"

extern "C" {
    #include "../../lib/kaguya-pin/kaguya-pin.h"
    #include "../../lib/pico-eeprom-i2c/src/eeprom.h"
}
#define INTERVAL_MS 30
float acc[4];
uint16_t head = 0;

BNO080 bno08x;

int main(void) {
    stdio_init_all();
    sleep_ms(2000);
    printf("\nhello, this is ex0_rotation_vector\n");

    i2c_init(i2c0, 400 * 1000);
    gpio_set_function(i2c0_scl_pin, GPIO_FUNC_I2C);
    gpio_set_function(i2c0_sda_pin, GPIO_FUNC_I2C);

    i2c_init(i2c1, 1000 * 1000);
    gpio_set_function(i2c1_scl_pin, GPIO_FUNC_I2C);
    gpio_set_function(i2c1_sda_pin, GPIO_FUNC_I2C);
    //bno08x.enableDebugging();

    uint8_t buf;
    //if (i2c_read_blocking(I2C_BUS, 0x4a, &buf, 1, false) > 0) printf("detect BNO085!\n");
    //else printf("could not find BNO085...\n");

    bool status = false;
    status = bno08x.begin(0x4a, i2c0, 255);
    if (!status) printf("failed to initialize BNO085\n");

    bno08x.enableAccelerometer(INTERVAL_MS);
    //sleep_ms(20);
    #if 1
    if (bno08x.getFeatureResponseAvailable()) {
        if (bno08x.checkReportEnable(SENSOR_REPORTID_ACCELEROMETER, INTERVAL_MS))
            bno08x.printGetFeatureResponse();
        else printf("fail: checkReportEnable\n");

    } else printf("fail: getFeatureResponseAvailable\n");
#endif
    uint8_t wbuf[50];
    while (1) {
        if (bno08x.dataAvailable()) {
            acc[0] = bno08x.getAccelX();
            acc[1] = bno08x.getAccelY();
            acc[2] = bno08x.getAccelZ();
            acc[3] = '\n';
            //printf("%d %f %f %f\n", time_us_32()/100000, acc[0], acc[1], acc[2]);
            snprintf((char *)wbuf, 50, "%3d,%+2.2f,%+2.2f,%+2.2f\n", time_us_32()/100000,acc[0], acc[1], acc[2]);
            for (int i = 0; i < 25; i++) printf("%c", wbuf[i]);
            eeprom_write_multi(i2c1, 0x50, head, wbuf, 25);
            head += 25;
        }
    }
}