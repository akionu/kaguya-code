#pragma once
#include "pico/stdlib.h"

// motor
const uint motor_right_a_pin = 22;
//const uint motor_right_b_pin = 23;
const uint motor_left_a_pin  = 18;
//const uint motor_left_b_pin  = 19;

// encoder
const uint encoder_right_a_pin = 24;
//const uint encoder_right_b = 25;
const uint encoder_left_a_pin  = 20;
//const uint encoder_left_b  = 21;

// i2c0(400kHz): SPL06-001(pressure), BNO085(IMU) 
const uint i2c0_sda_pin       = 12; // prs, imu
const uint i2c0_scl_pin       = 13; // prs, imu
// BNO085
const uint bno085_intn_pin    = 14;

// i2c1(1MHz)  : EEPROM, VL53L5CX(ToF)
const uint i2c1_sda_pin       = 2; // eeprom, tof
const uint i2c1_scl_pin       = 3; // eeprom, tof
// tof vcc
const uint tof_vcc_pin        = 4;

// uart0: LoRa
const uint lora_tx_pin        = 0; // mcu side
const uint lora_rx_pin        = 1; // mcu side
const uint lora_aux_pin       = 6;
const uint lora_m1_pin        = 10;
const uint lora_m0_pin        = 11;

// uart1: gnss
const uint gnss_tx_pin        = 8; // mcu side
const uint gnss_rx_pin        = 9; // mcu side
const uint gnss_vcc_pin       = 5; // intput LOW to ON

// current sense
// I = Imonitor *  (A)
const uint current_sense_pin  = 29;

// voltage sense
// V = Vmonitor * (V)
const uint voltage_sense_pin  = 27;

// status led
const uint led_pin            = 17;

// nichrome
const uint nichrome_pin       = 15;

