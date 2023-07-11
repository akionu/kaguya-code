#include <stdio.h>
#include "pico/stdlib.h"
//#include "../lib/kaguya-pin/kaguya-pin.h"
//#include "util.cpp"

extern "C" {
    #include "../../lib/motor_encoder/src/motor.h"
}

class Motor {
    public:
        void init(int8_t left, int8_t right);
        void forward(int16_t pwm);
        void back(int16_t pwm);
        void leftM();
        void rightM();
        void leftH(); // only 
        void rightH();
        void stop();
    private:
        uint8_t slice_left;
        uint8_t slice_right;
};

