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

	// ToDo set data rate, filters
	// ref: (library's init()), Acc=BMI270
	// https://github.com/m5stack/M5Unit-IMU-Pro-Mini/blob/main/src/BMI270.cpp
	// Output data rate (ODR) : 1600Hz (max)
	// Bandwidth : 740Hz @ ODR=1600Hz
  bmi270.init(I2C_NUM_0, BIM270_SENSOR_ADDR);
}

uint8_t ax[X], ay[X], az[X];
uint16_t p = 0;

/*
	bmi2_sens_config config;
	config.cfg.acc.odr = BMI2_ACC_ODR_1600HZ;
	config.cfg.acc.range = BMI2_ACC_RANGE_2G;
	config.cfg.aux.manual_en = BMI2_DISABLE;
	bmi270_set_sensor_config(&config, 1, &bmi270);
*/

#define MAX_ACC 1.0

uint16_t conv_acc(float x)
{
	uint16_t v;
	v = (x + MAX_ACC) / (2 * MAX_ACC) * Y;
	if (v < 0) v = 0;
	else if (v > Y -1) v = Y - 1;
	return(v);
}

void loop(void) {
	float x, y, z;
	if (bmi270.accelerationAvailable()) {
		bmi270.readAcceleration(x, y, z);
	//	printf("%d x/y/z = %f / %f / %f\n", p, x, y, z);
		ax[p] = (float)(x + MAX_ACC) / (2 * MAX_ACC) * Y;
		ax[p] = conv_acc(x);
		ay[p] = conv_acc(y);
		az[p] = conv_acc(z);
		for (uint16_t x = 0; x < X; x++){
			uint32_t dx = (x + p) % X;
			M5.Display.drawFastVLine(x, 0, Y, BLACK);
			M5.Lcd.drawPixel(x, ax[dx], BLUE);
			M5.Lcd.drawPixel(x, ay[dx], RED);
			M5.Lcd.drawPixel(x, az[dx], GREEN);
		}
		p = (p + 1) % X;
  }
//  delay(100);
}
