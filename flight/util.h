#pragma once
#include <stdio.h>
#include <math.h>
#include "pico/stdlib.h"

#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
#define abs(a) (((a)<0)?(-(a)):(a))
#define CONST_180_DIVIDED_BY_PI 57.2957795130823

/*平方根を使わず2点間の距離を近似*/
//参考
//https://nowokay.hatenablog.com/entry/20120604/1338773843
//https://dora.bk.tsukuba.ac.jp/~takeuchi/?%E3%83%97%E3%83%AD%E3%82%B0%E3%83%A9%E3%83%9F%E3%83%B3%E3%82%B0%2F%E5%B9%B3%E6%96%B9%E6%A0%B9%E3%82%92%E4%BD%BF%E3%82%8F%E3%81%9A%E3%81%AB%E8%B7%9D%E9%9B%A2%E3%82%92%E6%B1%82%E3%82%81%E3%82%8B
int32_t approx_distance(int32_t dx, int32_t dy) {
   int32_t min, max, approx;

   if (dx < 0) dx = -dx;
   if (dy < 0) dy = -dy;

   if (dx < dy) {
      min = dx;
      max = dy;
   } else {
      min = dy;
      max = dx;
   }

   approx = (max * 983) + (min * 407);
   if (max < (min << 4)) approx -= ( max * 40 );

   // add 512 for proper rounding
   return ((approx + 512) >> 10);
} 

