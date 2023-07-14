#include "uprs.h"

void Press::init() {
    config.addr = 0x77;
    config.i2c = i2c0;
    config.mode = BGD_PRS_TEMP;
    config.prs_config[0] = 2;
    config.prs_config[1] = 64;
    config.temp_config[0] = 2;
    config.temp_config[1] = 8;

    spl06_init(&config, &coef);
}

float Press::getAlt() {
    spl06_read_press_cal(&config, &coef, &prs);
    alt = spl06_calc_alt(prs);
    return alt;
}