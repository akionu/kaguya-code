#pragma once
#include <stdio.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"

extern "C" {
    #include "../../lib/pico-vl53l5cx/api/vl53l5cx_api.h"
}

// 流れ
/*
on();
init();
start();
if (isCaptureReady()) capture();
if (isConeReady()) res = isCone();

*/

class Tof {
    public:
        void on(int8_t vcc_pin);
        bool init();
        bool start();
        bool stop();
        bool isCaptureReady();
        void capture();
        bool isConeReady();
        bool isCone();
        uint8_t getCenterCm();
    private:
        void gradient(uint8_t in[64], uint8_t out[64], double amp);
        void median(uint8_t in[64], uint8_t out[64]);
        uint8_t median_value(uint8_t d[9]);
        void expand(uint8_t in[64], uint8_t out[64]);
        bool _isCone(uint8_t in[64]);

        uint8_t status, is_ready, is_alive;
        VL53L5CX_Configuration dev;
        VL53L5CX_ResultsData res;

        int8_t bufcnt;
        uint16_t matbuf[64];
        uint8_t mat[64];
        uint8_t mat2[64];
};