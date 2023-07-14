#include "utof.h"

void Tof::on(int8_t vcc_pin) {
    gpio_init(vcc_pin);
    gpio_set_dir(vcc_pin, GPIO_OUT);
    gpio_put(vcc_pin, 0);
}

bool Tof::init() {
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

    if (!status) return true;
    else return false;
}

bool Tof::start() {
    status = vl53l5cx_start_ranging(&dev);
    if (status) printf("err: start_ranging\n");

    if (!status) return true;
    else return false;
}

bool Tof::stop() {
    status = vl53l5cx_stop_ranging(&dev);
    if (status) printf("err: stop_ranging\n");

    if (!status) return true;
    else return false;
}

bool Tof::isCaptureReady() {
    status = vl53l5cx_check_data_ready(&dev, &is_ready);
    bool ret = is_ready;
    is_ready = false;
    return ret;
}

void Tof::capture() {
    vl53l5cx_get_ranging_data(&dev, &res);
    for (int i = 0; i < 64; i++) matbuf[i] += (uint8_t)(res.distance_mm[i] / 8);
    bufcnt++;
}

bool Tof::isConeReady() {
    if (bufcnt >= 2) return true;
    else return false;
}

bool Tof::isCone() {
    if (bufcnt == 2) {
        bufcnt = 0;
        for (uint8_t i = 0; i < 64; i++) mat[i] = (uint8_t)(matbuf[i] / 3);
        // コントラスト強調
        expand(mat, mat2);
        // ノイズ除去（メディアンフィルタ）
        median(mat2, mat);
        // エッジ抽出
        gradient(mat, mat2, 1.1);
        // コーン判定
        bool res = _isCone(mat2);

        for (int8_t i = 0; i < 64; i++) matbuf[i] = 0;
        for (int8_t i = 0; i < 64; i++) mat[i] = 0;
        
        return res;
    }
    return false;
}

bool Tof::_isCone(uint8_t in[64]) {
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

uint8_t Tof::getCenterCm() {
    uint16_t avg = 0;
    avg += (res.distance_mm[8*3+3] / 10);
    avg += (res.distance_mm[8*3+4] / 10);
    avg += (res.distance_mm[8*4+3] / 10);
    avg += (res.distance_mm[8*4+4] / 10);
    avg /= 4;
    return ((uint8_t)avg);
}

void Tof::expand(uint8_t in[64], uint8_t out[64]) {
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


uint8_t Tof::median_value(uint8_t d[9]) {
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

void Tof::median(uint8_t in[64], uint8_t out[64]) {
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

void Tof::gradient(uint8_t in[64], uint8_t out[64], double amp) {
    static int8_t cx[9] = {0, 0, 0,
                            0, 1, 0,
                            0, 0, -1};
    static int8_t cy[9] = {0, 0, 0,
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
