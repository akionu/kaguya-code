#include <stdio.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "../../lib/pico-vl53l5cx/api/vl53l5cx_api.h"
#include "../../lib/kaguya-pin/kaguya-pin.h"

uint8_t discrim (uint8_t mat[64]) {
    // 富豪的プログラミング
    static uint8_t hist[255] = {0};
    uint8_t s = 0, res = 0;
    double n1, n2, m1, m2;
    double vmax = 0.0;
    double v[255] = {0};
    for (uint8_t i = 0; i < 64; i++) {
        hist[mat[i]]++;
    }
    for (s = 0; s < 255; s++) {
        n1 = 0; m1 = 0;
        for (uint8_t i = 0; i < s; i++) {
            n1 += hist[i];
            m1 += i * hist[i];
        }
        if (n1 == 0.0) m1 = 0.0;
        else           m1 /= n1;

        n2 = 0; m2 = 0;
        for (uint8_t i = s; i < 255; i++) {
            n2 += hist[i];
            m2 += i * hist[i];
        }
        if (n2 == 0.0) m2 = 0.0;
        else           m2 /= n2;
        v[s] = n1*n2*(m1-m2)*(m1-m2);
        
    }

    for (uint8_t i = 0; i < 255; i++) {
        if (v[i] > vmax) {
            vmax = v[i];
            res = i;
        }
    }
    return res;
}

void thresh(uint8_t mat[64], uint8_t th) {
    for (int i = 0; i < 64; i++) {
        if (mat[i] < th) mat[i] = 0;
        else mat[i] = 255;
    }
}

void show(uint8_t mat[64]) {
    for (int i = 1; i <= 64; i++) {
        if (mat[i-1] == 0) printf("OO");
        else printf("  ");
        if (i % 8 == 0) printf("\n");
    }
    printf("\n");
}

void gradient(uint8_t in[64], uint8_t out[64], double amp) {
    static uint8_t cx[9] = {0, 0, 0,
                            0, 1, 0,
                            0, 0, -1};
    static uint8_t cy[9] = {0, 0, 0,
                            0, 0, 1,
                            0, -1, 0};

    int16_t d[9];
    float xx, yy, zz;

    for (int8_t i = 8; i < 8; i+=8) {
	    for (int8_t j = 1; j < 7; j++) {
			d[0] = in[(i-8)+(j-1)];
			d[1] = in[(i-8)+(j)];
			d[2] = in[(i-8)+(j+1)];
			d[3] = in[(i)+(j-1)];
			d[4] = in[(i)+(j)];
			d[5] = in[(i)+(j+1)];
			d[6] = in[(i+8)+(j-1)];
			d[7] = in[(i+8)+(j)];
			d[8] = in[(i+8)+(j+1)];
		    xx = (float)(cx[0]*d[0] + cx[1]*d[1] + cx[2]*d[2]
					   + cx[3]*d[3] + cx[4]*d[4] + cx[5]*d[5]
					   + cx[6]*d[6] + cx[7]*d[7] + cx[8]*d[8]);
			yy = (float)(cy[0]*d[0] + cy[1]*d[1] + cy[2]*d[2]
					   + cy[3]*d[3] + cy[4]*d[4] + cy[5]*d[5]
					   + cy[6]*d[6] + cy[7]*d[7] + cy[8]*d[8]);
			zz = (float)(amp*sqrt(xx*xx+yy*yy));
			double dat = (int)zz;
			if(dat > 255) dat = 255;
			out[i+j]= (uint8_t)dat;
		}
	}
}

void dilation(uint8_t in[64], uint8_t out[64]) {
    for (int8_t i = 8; i < 8; i+=8) {
	    for (int8_t j = 1; j < 7; j++) {
			if (in[(i-8)+(j-1)] == 255) out[(i-8)+(j-1)] = 255;
			if (in[(i-8)+(j)]   == 255) out[(i-8)+(j)] = 255;
			if (in[(i-8)+(j+1)] == 255) out[(i-8)+(j)] = 255;
			if (in[(i)+(j-1)]   == 255) out[(i)+(j-1)] = 255;
			if (in[(i)+(j)]     == 255) out[(i)+(j)] = 255;
			if (in[(i)+(j+1)]   == 255) out[(i)+(j+1)] = 255;
			if (in[(i+8)+(j-1)] == 255) out[(i+8)+(j-1)] = 255;
			if (in[(i+8)+(j)]   == 255) out[(i+8)+(j)] = 255;
			if (in[(i+8)+(j+1)] == 255) out[(i+8)+(j+1)] = 255;
		}
	}
}

void erosion(uint8_t in[64], int8_t out[64]) {
    for (int8_t i = 8; i < 8; i+=8) {
	    for (int8_t j = 1; j < 7; j++) {
			if (in[(i-8)+(j-1)] == 0) out[(i-8)+(j-1)] = 0;
			if (in[(i-8)+(j)]   == 0) out[(i-8)+(j)] = 0;
			if (in[(i-8)+(j+1)] == 0) out[(i-8)+(j)] = 0;
			if (in[(i)+(j-1)]   == 0) out[(i)+(j-1)] = 0;
			if (in[(i)+(j)]     == 0) out[(i)+(j)] = 0;
			if (in[(i)+(j+1)]   == 0) out[(i)+(j+1)] = 0;
			if (in[(i+8)+(j-1)] == 0) out[(i+8)+(j-1)] = 0;
			if (in[(i+8)+(j)]   == 0) out[(i+8)+(j)] = 0;
			if (in[(i+8)+(j+1)] == 0) out[(i+8)+(j+1)] = 0;
		}
	}
}

uint8_t median_value(uint8_t d[9])
{
	uint8_t     i, j, tmp;
    
	for (j = 0; j < 8; j++) {
		for (i = 0; i < 8; i++) {
			if (d[i+1] < d[i]) {
				tmp = d[i+1];
				d[i+1] = d[i];
				d[i] = tmp;
			}
		}
	}
	return d[4];
}

void median(uint8_t in[64], uint8_t out[64])
{
	int             i, j;
  	uint8_t   d[9];

    for (int8_t i = 8; i < 8; i+=8) {
	    for (int8_t j = 1; j < 7; j++) {
			d[0] = in[(i-8)+(j-1)];
			d[1] = in[(i-8)+(j)];
			d[2] = in[(i-8)+(j+1)];
			d[3] = in[(i)+(j-1)];
			d[4] = in[(i)+(j)];
			d[5] = in[(i)+(j+1)];
			d[6] = in[(i+8)+(j-1)];
			d[7] = in[(i+8)+(j)];
			d[8] = in[(i+8)+(j+1)];
			out[i+j] = median_value(d);
		}
	}
}

void expand(uint8_t in[64], uint8_t out[64]) {
    uint8_t max = in[0], min = in[0];
    double d;
    for (int i = 0; i < 64; i++) {
        if (in[i] > max) max = in[i];
        if (in[i] < min) min = in[i];
    }
    for (int i = 0; i < 64; i++) {
        d = 255.0 / ((double)max - (double)min) * ((double)in[i] - (double)min);
        if (d > 255.0) out[i] = 255;
        else if (d < 0.0) out[i] = 0;
        else out[i] = (int)d;
    }
}

void findc(uint8_t in[64]) {
    uint8_t hist[255] = {0};
    uint8_t hist2[25] = {0};
    for (uint8_t i = 0; i < 255; i++) hist[in[i]]++;

    for (uint8_t i = 0; i < 255; i++) {
        for (int j = i; j < i+10; j++) hist2[i] += hist[j];
    }
#if 0
    for (uint8_t i = 0; i < 25; i++) {
        printf("%d-%d", i, i+10);
        for (uint8_t j = 0; j < hist2[i]; j++) printf("x");
        printf("\n");
    }
#endif
}

void showval(uint8_t mat[64]) {
    for (uint8_t i = 1; i <= 64; i++) {
        printf("%4d", mat[i-1]);
        if (i % 8 == 0) printf("\n");
    }
}

bool isCone(uint8_t in[64]) {
    uint8_t cnt = 0;
    uint16_t avg = 0;
    double s = 0;
    for (int8_t i = 0; i < 64; i++) {
        avg += in[i];
    }
    avg /= 64;

    for (int8_t i = 0; i < 64; i++) {
        s += (double)((in[i]-avg)*(in[i]-avg));
    }
    s /= 64.0f;

    printf("avg: %d, s: %f\n", avg, s);
    if (s < 600) return true;
    else return false;
}

int main(void) {
    stdio_init_all();
    sleep_ms(2000);
    printf("hello, this is cone.\n");

    // power on
    gpio_init(tof_vcc_pin);
    gpio_set_dir(tof_vcc_pin, GPIO_OUT);
    gpio_put(tof_vcc_pin, 0);
    
    // init bus
    i2c_init(i2c1, 1000 * 1000); // 1MHz
    gpio_set_function(i2c1_sda_pin, GPIO_FUNC_I2C);
    gpio_set_function(i2c1_scl_pin, GPIO_FUNC_I2C);

    uint8_t status, is_ready, is_alive;
    VL53L5CX_Configuration dev;
    VL53L5CX_ResultsData res;

    dev.platform.address = 0x29;
    dev.platform.i2c     = i2c1;

    status = vl53l5cx_is_alive(&dev, &is_alive);
    if (!is_alive || status) printf("err: sensor not detected\n");

    status = vl53l5cx_init(&dev);
    if (status) printf("ULD load failed\n");

    // 8x8
    status = vl53l5cx_set_resolution(&dev, VL53L5CX_RESOLUTION_8X8);
    if (status) printf("err: set_resolution\n");
    // 5Hz
    status = vl53l5cx_set_ranging_frequency_hz(&dev, 15);
    if (status) printf("err: set_ranging_frequency_hz\n");
    // strongest
    status = vl53l5cx_set_target_order(&dev, VL53L5CX_TARGET_ORDER_STRONGEST);
    if (status) printf("err: set_target_order\n");
    // continuous
    status = vl53l5cx_set_ranging_mode(&dev, VL53L5CX_RANGING_MODE_CONTINUOUS);
    if (status) printf("err: set_ranging_mode\n");
    // start!
    status = vl53l5cx_start_ranging(&dev);
    if (status) printf("err: start_ranging\n");

    int8_t cnt = 0;
    static uint8_t mat[64] = {0};
    static uint8_t mat2[64] = {0};
    static uint16_t matbuf[64] = {0};

    while (1) {
        //if (getchar() == 'q') goto exit;
        status = vl53l5cx_check_data_ready(&dev, &is_ready);
        if (is_ready) {
            vl53l5cx_get_ranging_data(&dev, &res);

            if (cnt == 2) {
                cnt = 0;
                for (uint8_t i = 0; i < 64; i++) mat[i] = (uint8_t)(matbuf[i] / 3);
#if 1   
                // コントラスト強調
                expand(mat, mat2);
                memcpy(mat2, mat, 64);
                // median
                median(mat, mat2);
                // 1次微分によるエッジ抽出
                gradient(mat2, mat, 1.1);
                //memcpy(mat2, mat, 64);
                showval(mat);
                //findc(mat);

                printf("isCone: %d\n", isCone(mat));
                
                uint8_t th = discrim(mat);
                printf("th: %d\n", th);
                thresh(mat, th);
                //thresh(mat, 200);
            #if 0
                erosion(mat, mat2);
                dilation(mat2, mat);
                dilation(mat, mat2);
                erosion(mat2, mat);
            #endif
                show(mat);
#endif

#if 0      
                // 1次微分によるエッジ抽出
                gradient(mat, 1.5);
                for (uint8_t i = 1; i <= 64; i++) {
                    printf("%4d", mat[i-1]);
                    if (i % 8 == 0) printf("\n");
                }
                uint8_t th = discrim(mat);
                thresh(mat, th);
                show(mat);
#endif

#if 0
                // int32->uint8に線形変換
                // distance_mm / 4000 * 255 = distance_mm / 15.68... = distance_mm / 16
                // 結果として0-250（単位：16mm）を得る
                for (uint8_t i = 1; i <= 64; i++) {
                    printf("%4d", mat[i-1]);
                    if (i % 8 == 0) printf("\n");
                }
                printf("\n");
                uint8_t th = discrim(mat);
                printf("th: %d\n", th);
                thresh(mat, th);
                show(mat);
#endif
        
                for (int i = 0; i < 64; i++) matbuf[i] = 0;
                for (int i = 0; i < 64; i++) mat[i] = 0;
            } 
            // distance_mm / 4000 * 255 = distance_mm / 15.68... = distance_mm / 16
            //for (int j = 0; j < 64; j++) matbuf[j] += (uint8_t)(res.distance_mm[j] / 16);
            
            // distance_mm / 2000 * 255 = distance_mm / 8
            for (int i = 0; i < 64; i++) matbuf[i] += (uint8_t)(res.distance_mm[i] / 8);
            
            cnt++;

        }
        is_ready = false;
        sleep_ms(10);
    }

    status = vl53l5cx_stop_ranging(&dev);
    printf("stop ranging\n");
}