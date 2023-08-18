#include "ulog.h"

Log::Log(uint8_t code, uint8_t lora_ch) {
    this->code = code;
    this->head = 0;
    this->time.year  = 2023;
    this->time.month = 06;
    this->time.day   = 17;
    this->time.dotw  = 6; // 0 is Sunday, so 5 is Friday
    this->time.hour  = 00;
    this->time.min   = 00;
    this->time.sec   = 00;

    rtc_init();
    rtc_set_datetime(&time);
    sleep_us(64);

    this->lora_cfg.addr = 0xffff;
    this->lora_cfg.baudrate_div100 = 96;
    this->lora_cfg.air_data_rate = 12509;
    this->lora_cfg.subpacket_len = 32;
    this->lora_cfg.rssi = false;
    this->lora_cfg.tx_power = 13;
    this->lora_cfg.channel = lora_ch;
    this->lora_cfg.rssi_byte = false;
    this->lora_cfg.tx_mode_transp = true;
    this->lora_cfg.wor_cycle_div100 = 20;
    this->lora_cfg.enckey = 0x0000;

    this->lora_pin.txpin = lora_tx_pin;
    this->lora_pin.m0pin = lora_m0_pin;
    this->lora_pin.m1pin = lora_m1_pin;
    this->lora_pin.auxpin = lora_aux_pin;
    lora_init(uart0, &lora_pin);
    lora_config(uart0, &lora_cfg, &lora_pin);
}

bool Log::addLog(int32_t lat, int32_t lon,
                float yaw, float roll, float pitch,
                uint8_t seq, 
                uint8_t buf[12]) {
    uint8_t bufw[32] = {0};
    //uint8_t buf[12] = {' '};
    rtc_get_datetime(&time);
    // 機体コード
    bufw[0] = code;
    // rtc
    // time
    bufw[1] = time.hour;
    bufw[2] = time.min;
    bufw[3] = time.sec;
    // gps
    // latitude
    bufw[4] = (lat >> 24) & 0xff;
    bufw[5] = (lat >> 16) & 0xff;
    bufw[6] = (lat >> 8) & 0xff;
    bufw[7] = (lat & 0xff);
    // longitude
    bufw[8] = (lon >> 24) & 0xff;
    bufw[9] = (lon >> 16) & 0xff;
    bufw[10] = (lon >> 8) & 0xff;
    bufw[11] = (lon & 0xff);
    // yaw
    int16_t tmp = (int16_t)(yaw * 100);
    bufw[12] = (tmp << 8);
    bufw[13] = (tmp & 0xff);
    // roll
    tmp = (int16_t)(roll * 100);
    bufw[14] = (tmp << 8);
    bufw[15] = (tmp & 0xff);
    // pitch
    tmp = (int16_t)(pitch * 100);
    bufw[16] = (tmp << 8);
    bufw[17] = (tmp & 0xff);
    // seq number
    bufw[18] = seq;
    // data
    for (int i = 0; i < 12; i++) bufw[i+19] = buf[i];
    // separator
    bufw[31] = '\n';

    // send/write
    int8_t ret = eeprom_write_multi(i2c1, 0x50, head, bufw, 32);
    //printf("eeprom_write: %d\n", ret);
    head += ((ret > 0) ? (ret) : 0);
    lora_send(uart0, bufw, 32);
    return (ret > 0);
}

void Log::showAll() {
    uint32_t now = 0;
    uint8_t buf[32] = {0};
    printf("code,hour.min.sec,lat,lon,yaw,roll,pitch,seq,data\n");
    while (now < head) {
        int8_t ret = eeprom_read(i2c1, 0x50, now, buf, 32);
        now += ((ret > 0) ? ret : 0);
        if (buf[31] != '\n') {
            printf("error reading log: %d\n", now);
        }
        int32_t lat = (buf[4] << 24) | (buf[5] << 16) | (buf[6] << 8) | buf[7];
        int32_t lon = (buf[8] << 24) | (buf[9] << 16) | (buf[10] << 8) | buf[11];
        printf("%c,%d.%d.%d,%d,%d,%3.2f,%3.2f,%3.2f,%d,", 
                    buf[0], // code
                    buf[1], // hour
                    buf[2], // min
                    buf[3], // sec
                    lat,
                    lon,
                    (float)((buf[12] << 8) | buf[13]) / 100.0f,
                    (float)((buf[14] << 8) | buf[15]) / 100.0f,
                    (float)((buf[16] << 8) | buf[17]) / 100.0f,
                    buf[18]
        );
        for (int i = 0; i < 12; i++) printf("%c", buf[i+19]);
        for (int i = 0; i < 32; i++) buf[i] = ' ';
        printf("\n");
    }
}
