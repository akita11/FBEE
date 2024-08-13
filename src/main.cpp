// FBEE: FutureBody-ElephantEar

#include <M5Unified.h>
#include "M5_IMU_PRO.h"

#define BIM270_SENSOR_ADDR 0x68
#define BMP280_SENSOR_ADDR 0x76

BMI270::BMI270 bmi270;

#define pinBz 26

#define X 320
#define Y 240

// CoreS3 + audio play
// https://qiita.com/suzukiplan/items/ba86610d523a94775665

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

  ledcSetup(0, 4000, 10);
  ledcAttachPin(pinBz, 0);

#if defined(ARDUINO_M5STACK_CORE2)
	// I2S audio for Core2
	i2s_config_t audioConfig = {
		.mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
		.sample_rate = 44100,
		.bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
		.channel_format = I2S_CHANNEL_FMT_ONLY_RIGHT,
		.communication_format = I2S_COMM_FORMAT_STAND_MSB,
		.intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
		.dma_buf_count = 4,
		.dma_buf_len = 1024,
		.use_apll = false,
		.tx_desc_auto_clear = true};
	i2s_driver_install(I2S_NUM_0, &audioConfig, 0, nullptr);
	i2s_set_clk(I2S_NUM_0, 44100, I2S_BITS_PER_SAMPLE_16BIT, I2S_CHANNEL_MONO);
	i2s_zero_dma_buffer(I2S_NUM_0);

	// write PCB data from buf (max 1024 bytes)
	char buf[1024];
	int bufSize = 1024;
	size_t wrote;
	i2s_write(I2S_NUM_0, buf, bufSize, &wrote, portMAX_DELAY);
	vTaskDelay(2);
#elif defined(ARDUINO_M5STACK_CORES3)
	// I2S audio for CoreS3
	// setup AW88298 regisgter
	M5.In_I2C.bitOn(0x36, 0x02, 0b00000100, 400000);
	this->writeRegister(0x61, 0x0673);
	this->writeRegister(0x04, 0x4040);
	this->writeRegister(0x05, 0x0008);
	this->writeRegister(0x06, 0b0001110000000111); // I2SCTL: 44.1kHz, 16bits, monoral (他は全部デフォルト値)
	this->writeRegister(0x0C, 0x0064);
	// I2S config
	i2s_config_t config;
	memset(&config, 0, sizeof(i2s_config_t));
	config.mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX);
	config.sample_rate = 48000; // dummy setting
	config.bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT;
	config.channel_format = I2S_CHANNEL_FMT_ONLY_RIGHT;
	config.communication_format = I2S_COMM_FORMAT_STAND_I2S;
	config.dma_buf_count = 4;
	config.dma_buf_len = 1024;
	config.tx_desc_auto_clear = true;
	// I2S pin config
	i2s_pin_config_t pinConfig;
	memset(&pinConfig, ~0u, sizeof(i2s_pin_config_t));
	pinConfig.bck_io_num = GPIO_NUM_34;
	pinConfig.ws_io_num = GPIO_NUM_33;
	pinConfig.data_out_num = GPIO_NUM_13;
	// Setup I2S
	i2s_driver_install(I2S_NUM_1, &config, 0, nullptr);
	i2s_set_pin(I2S_NUM_1, &pinConfig);
	i2s_zero_dma_buffer(I2S_NUM_1);
	i2s_start(I2S_NUM_1);
	char buf[1024];
	int bufSize = 1024;
	size_t wrote;
	i2s_write(I2S_NUM_1, buf, bufSize, &wrote, portMAX_DELAY);
	vTaskDelay(2);
#else
#error "no Core2 or CoreS3"
#endif


}

void setFreq(int freq){
	if (freq == 0) ledcWrite(0, 0);
	else{
		ledcWriteTone(0, freq);	
	}
}

float ax[X][2], ay[X][2], az[X][2];
uint16_t p = 0;

//#define MAX_ACC 1.0
//#define MAX_ACC 2.0
#define MAX_ACC 0.1

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
