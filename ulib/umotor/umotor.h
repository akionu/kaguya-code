#include <stdio.h>
#include "pico/stdlib.h"
//#include "../lib/kaguya-pin/kaguya-pin.h"
//#include "util.cpp"

extern "C" {
    #include "../../lib/motor_encoder/src/motor.h"
}

class Motor {
    public:
        Motor();
        void init(int8_t left, int8_t right);
        void forward(int16_t pwm);
        void backward(int16_t pwm);
        void leftM();
        void rightM();
        void leftH(); // only 
        void rightH();
        void stop();
        void setDirForward(int8_t left, int8_t right);
    private:
        uint8_t slice_left;
        uint8_t slice_right;
        bool is_dir_set;
        int8_t dir_left;
        int8_t dir_right;
};

