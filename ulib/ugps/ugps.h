#pragma once
#include <stdio.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/uart.h"
//#include "../lib/kaguya-pin/kaguya-pin.h"
//#include "util.cpp"

extern "C" {
    #include "../../lib/gnss/nmeap-0.3/inc/nmeap.h"
}

class GPS {
    public:
        GPS(int32_t goal_lat, int32_t goal_long, 
            int16_t lat_per_res, int16_t long_per_res,
            double(*fx)(double));
        void init(uint tx_pin, uint rx_pin);
        bool isReady();
        float getDirection();
        float getDistance();
        int32_t getLatitude();
        int32_t getLongitude();
        int32_t goal_latitude;
        int32_t goal_longitude;
    private:
        //bool is_ready;
        //void callout(nmeap_context_t *context, void *data, void *user_data);
        //void on_uart1_rx();
        int32_t approxDistance(int32_t dx, int32_t dy);
        nmeap_gga_t gga;
        double (*fx)(double);
        int16_t long_per_res;
        int16_t lat_per_res;
};

