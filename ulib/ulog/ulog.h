#pragma once
#include <stdio.h>
#include <stdarg.h>
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/rtc.h"
#include "pico/util/datetime.h"
#include "../../lib/kaguya-pin/kaguya-pin.h"

extern "C" {
    #include "../../lib/pico-eeprom-i2c/src/eeprom.h"
    #include "../../lib/pico-e220/src/pico-e220.h"
}

class Log {
    public:
        Log(uint8_t code);
        void init();
        bool addLog(uint32_t lat, uint32_t lon,
                float yaw, float roll, float pitch,
                uint8_t seq, 
                uint8_t buf[12]);
        void showAll();
        uint32_t head;
    private:
        datetime_t time;
        uint8_t code; // 機体コード
        e220_config_t lora_cfg;
        e220_pin_t lora_pin;
};