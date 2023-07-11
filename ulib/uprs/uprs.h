#include <stdio.h>
#include "pico/stdlib.h"
extern "C" {
    #include "../../lib/pico-spl06-001/src/pico-spl06-001.h"
}

class Press {
    public:
        void init();
        float getAlt();

    private:
        float prs;
        float alt;
        spl06_config_t config;
        spl06_coef_t coef;
};
