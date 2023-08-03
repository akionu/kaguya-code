#pragma once
#include <stdio.h>
#include "pico/stdlib.h"

enum {
    MODE_WAIT = 1,
    MODE_LANDING = 2,
    MODE_GNSS = 3,
    MODE_TOF = 4,
    MODE_GOAL = 5,
    MODE_FORWARD_LANDING = 6,
    MODE_FORWARD_TOF = 7,
    MODE_EXPANSION = 8,
    MODE_SHOWLOG = 9,
    MODE_NICHROME = 10,
};

#define SEC2CNT 2

// 初期モード
static uint mode_now = MODE_LANDING;
//static uint mode_now = MODE_GNSS;
//static uint mode_now = MODE_FORWARD_TOF;

const int32_t wait_count = 3 * 100; // 30秒
const float angle_th = 0.39; //前45度，GPS誘導
const uint16_t long_per_res = 111;
const uint16_t lat_per_les = 96;