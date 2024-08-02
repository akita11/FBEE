#include <M5Unified.h>
#include "M5_IMU_PRO.h"

#define BIM270_SENSOR_ADDR 0x68
#define BMP280_SENSOR_ADDR 0x76

BMI270::BMI270 bmi270;

#define X 320
#define Y 240

void setup() {
	M5.begin();
	M5.Ex_I2C.begin();
	Wire.setClock(400000L); // I2C clock = 400kHz

	// M5Stack IMU-Pro-Mini library:
	// https://github.com/m5stack/M5Unit-IMU-Pro-Mini

	// ODR=400Hz, range=2G
	// * modify M5Unit-IMU-Pro-Mini/src/utilities/BMI270-Sensor-API/BMI270.cpp, L290
	//   sampling rate: 4ms (250Hz)@5sample, 3ms(300Hz)@20sample, 3.5ms(300Hz)@20sample/graphX=100
	//   sens_cfg[0].cfg.acc.odr         = BMI2_ACC_ODR_400HZ;
	//   sens_cfg[0].cfg.acc.range       = BMI2_ACC_RANGE_2G;

  bmi270.init(I2C_NUM_0, BIM270_SENSOR_ADDR);
}

float ax[X][2], ay[X][2], az[X][2];
uint16_t p = 0;

//#define MAX_ACC 1.0
#define MAX_ACC 2.0

uint16_t conv_acc(float x)
{
	int v;
	v = (x + MAX_ACC) / (2 * MAX_ACC) * Y;
	if (v < 0) v = 0;
	else if (v > Y -1) v = Y - 1;
	return(v);
}

uint8_t tm = 0;
uint32_t t1, t0; 
uint16_t to = 0;
float ax0 = 0, ay0 = 0, az0 = 0;
void loop(void) {
	float x, y, z;
	if (bmi270.accelerationAvailable()) {
		bmi270.readAcceleration(x, y, z);
		if (x > ax[p][0]) ax[p][0] = x;
		if (x < ax[p][1]) ax[p][1] = x;
		if (y > ay[p][0]) ay[p][0] = y;
		if (y < ay[p][1]) ay[p][1] = y;
		if (z > az[p][0]) az[p][0] = z;
		if (z < az[p][1]) az[p][1] = z;
		tm++;
		if (tm == 20){
//			t1 = millis(); printf("%d\n", t1 - t0); t0 = t1;
			tm = 0;
#define XMAX 100
			for (uint16_t xx = 0; xx < XMAX; xx++){
				uint32_t dx = (xx + p + 1) % XMAX;
				M5.Display.drawFastVLine(xx, 0, Y, BLACK);
				uint16_t y0, y1;
				y0 = conv_acc(ax[dx][0] - ax0); y1 = conv_acc(ax[dx][1] - ax0);	M5.Display.drawFastVLine(xx, y0, y0 -y1 + 1, BLUE);
				y0 = conv_acc(ay[dx][0] - ay0); y1 = conv_acc(ay[dx][1] - ay0);	M5.Display.drawFastVLine(xx, y0, y0 -y1 + 1, RED);
				y0 = conv_acc(az[dx][0] - az0); y1 = conv_acc(az[dx][1] - az0);	M5.Display.drawFastVLine(xx, y0, y0 -y1 + 1, GREEN);
			}
			to++;
			if (to == 100){
				to = 0;
				ax0 = (ax[p][0] + ax[p][1]) / 2; 
				ay0 = (ay[p][0] + ay[p][1]) / 2; 
				az0 = (az[p][0] + az[p][1]) / 2; 
			}
			p = (p + 1) % XMAX;
			ax[p][0] = -100; ay[p][0] = -100; az[p][0] = -100;
			ax[p][1] =  100; ay[p][1] =  100; az[p][1] =  100;
		}
  }
}
