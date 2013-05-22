#include "board.h"

#define MS5611_ADDR             0x77                            // MS5611, Standard address 0x77
#define BMP085_OFF              digitalLo(BARO_GPIO, BARO_PIN); // Autodetect: turn off BMP085 while initializing ms5611 and check PROM crc to confirm device
#define BMP085_ON               digitalHi(BARO_GPIO, BARO_PIN);
#define CMD_RESET               0x1E                            // ADC reset command
#define CMD_ADC_READ            0x00                            // ADC read command
#define CMD_ADC_CONV            0x40                            // ADC conversion command
#define CMD_ADC_D1              0x00                            // ADC D1 conversion
#define CMD_ADC_D2              0x10                            // ADC D2 conversion
#define CMD_ADC_256             0x00                            // ADC OSR=256
#define CMD_ADC_512             0x02                            // ADC OSR=512
#define CMD_ADC_1024            0x04                            // ADC OSR=1024
#define CMD_ADC_2048            0x06                            // ADC OSR=2048
#define CMD_ADC_4096            0x08                            // ADC OSR=4096
#define CMD_PROM_RD             0xA0                            // Prom read command
#define PROM_NB                 8                               // 8*16Bit = 128 Bit

static void ms5611_reset(void);
static uint16_t ms5611_prom(int8_t coef_num);
static int8_t ms5611_crc(uint16_t *prom);
static uint32_t ms5611_read_adc(void);
static void ms5611_start_ut(void);
static void ms5611_get_ut(void);
static void ms5611_start_up(void);
static void ms5611_get_up(void);
static int32_t ms5611_calculate(void);

static uint32_t ms5611_ut;                                       // static result of temperature measurement
static uint32_t ms5611_up;                                       // static result of pressure measurement
static uint16_t ms5611_c[PROM_NB];                               // on-chip ROM
static uint8_t  ms5611_osr = CMD_ADC_4096;

bool ms5611Detect(baro_t *baro){
    GPIO_InitTypeDef GPIO_InitStructure;
    bool ack = false;
    uint8_t sig,i;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;                   // PC13 (BMP085's XCLR reset input, which we use to disable it)
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
    BMP085_OFF;
    delay(50);                                                   // Crashpilot 100ms No idea how long the chip takes to power-up, but let's make it 10ms
    // BMP085 is disabled. If we have a MS5611, it will reply. if no reply, means either we have BMP085 or no baro at all.
    ack = i2cRead(MS5611_ADDR, CMD_PROM_RD, 1, &sig);
    if (!ack) return false;
 	  delay(50);
	  for (i = 0; i < 5; i++) ms5611_reset();                      // Reset the damn thing a few times
    for (i = 0; i < PROM_NB; i++) ms5611_c[i] = ms5611_prom(i);  // read all coefficients // read 8 words word:0 = ID; word:1-6 = C1-C6; word:7 = ChkSum
		if (ms5611_crc(ms5611_c) != 0) return false;                 // check crc, bail out if wrong - we are probably talking to BMP085 w/o XCLR line!
// 	  baro->ut_delay = 10000; //    baro->up_delay = 10000; //    baro->repeat_delay = 4000;
 	  baro->ut_delay = 9500;
    baro->up_delay = 9500;
    baro->repeat_delay = 1;
    baro->start_ut = ms5611_start_ut;
    baro->get_ut = ms5611_get_ut;
    baro->start_up = ms5611_start_up;
    baro->get_up = ms5611_get_up;
    baro->calculate = ms5611_calculate;
    delay(50);
    for (i = 0; i < 10; i++) ms5611_reset();                     // Reset the damn thing a few times
    delay(50);
    return true;
}

static void ms5611_reset(void){
    i2cWrite(MS5611_ADDR, CMD_RESET, 1);
	  delayMicroseconds(5000);                                     // Crashpilot delayMicroseconds(2800);
}

static uint16_t ms5611_prom(int8_t coef_num){
    uint8_t rxbuf[2] = { 0, 0 };
    i2cRead(MS5611_ADDR, CMD_PROM_RD + coef_num * 2, 2, rxbuf); // send PROM READ command
    return rxbuf[0] << 8 | rxbuf[1];
}

static int8_t ms5611_crc(uint16_t *prom){
    int32_t i, j;
    uint32_t res = 0;
    uint8_t zero = 1;
    uint8_t crc = prom[7] & 0xF;
    prom[7] &= 0xFF00;

    for (i = 0; i < 8; i++) {                                   // if eeprom is all zeros, we're probably fucked - BUT this will return valid CRC lol
        if (prom[i] != 0) zero = 0;
    }
    if (zero) return -1;
    for (i = 0; i < 16; i++) {
        if (i & 1) res ^= ((prom[i >> 1]) & 0x00FF);
        else res ^= (prom[i >> 1] >> 8);
        for (j = 8; j > 0; j--) {
            if (res & 0x8000) res ^= 0x1800;
            res <<= 1;
        }
    }
    prom[7] |= crc;
    if (crc == ((res >> 12) & 0xF)) return 0;
    return -1;
}

static uint32_t ms5611_read_adc(void){
    uint8_t rxbuf[3];
    i2cRead(MS5611_ADDR, CMD_ADC_READ, 3, rxbuf);                     // read ADC
    return (rxbuf[0] << 16) | (rxbuf[1] << 8) | rxbuf[2];
}

static void ms5611_start_ut(void){
    i2cWrite(MS5611_ADDR, CMD_ADC_CONV + CMD_ADC_D2 + ms5611_osr, 1); // D2 (temperature) conversion start!
}

static void ms5611_get_ut(void){
	  uint32_t tmp;
    tmp = ms5611_read_adc();
	  if (tmp !=0) ms5611_ut = tmp;   // Keep old on error
}

static void ms5611_start_up(void){
    i2cWrite(MS5611_ADDR, CMD_ADC_CONV + CMD_ADC_D1 + ms5611_osr, 1); // D1 (pressure) conversion start!
}

static void ms5611_get_up(void){
	  uint32_t tmp;
    tmp = ms5611_read_adc();
	  if (tmp !=0) ms5611_up = tmp;   // Keep old on error
}

static int32_t ms5611_calculate(void){
    int32_t temperature, off2 = 0, sens2 = 0, delt;
    int32_t pressure;
	  uint32_t C5;                                                   // Crashpilot perhaps better, doesn't harm anyways
    C5 = ms5611_c[5];                                              // Crashpilot perhaps better, doesn't harm anyways
	  C5 = C5 << 8;                                                  // Crashpilot perhaps better, doesn't harm anyways
    int64_t dT  = (int32_t)ms5611_ut - ((int32_t)C5);              // Crashpilot perhaps better, doesn't harm anyways
	  int64_t off = ((uint32_t)ms5611_c[2] << 16) + ((dT * ms5611_c[4]) >> 7);
    int64_t sens = ((uint32_t)ms5611_c[1] << 15) + ((dT * ms5611_c[3]) >> 8);
    temperature = 2000 + ((dT * ms5611_c[6]) >> 23);
    if (temperature < 2000) {                                      // temperature lower than 20degC 
        delt = temperature - 2000;
        delt = delt * delt;
        off2 = (5 * delt) >> 1;
        sens2 = (5 * delt) >> 2;
        if (temperature < -1500) {                                 // temperature lower than -15degC
            delt = temperature + 1500;
            delt = delt * delt;
            off2  += 7 * delt;
            sens2 += (11 * delt) >> 1;
        }
    }
    off  -= off2; 
    sens -= sens2;
    pressure = (((ms5611_up * sens ) >> 21) - off) >> 15;
    return pressure;
}
/*
ms5611_calculate Alternatives
=============================


  APM WAY
  =======
  int32_t pressure;
  float D1, D2, C1, C2, C3, C4, C5, C6;
	float dT, TEMP, OFF, OFF2, SENS, SENS2, T2, Aux; 
	D1 = ms5611_up;
	D2 = ms5611_ut;
	C1 = ms5611_c[1];
	C2 = ms5611_c[2];
  C3 = ms5611_c[3];
	C4 = ms5611_c[4];
  C5 = ms5611_c[5];
	C6 = ms5611_c[6];
	dT = D2 - (((uint32_t)C5) <<8);
  TEMP = (dT * C6)/8388608;
  OFF = C2 * 65536.0f + (C4 * dT) / 128;
  SENS = C1 * 32768.0f + (C3 * dT) / 256;
  if (TEMP < 0){                                                      // second order temperature compensation when under 20 degrees C
      T2 = (dT * dT) / 0x80000000;
      Aux = TEMP * TEMP;
      OFF2 = 2.5f * Aux;
      SENS2 = 1.25f * Aux;
      TEMP = TEMP - T2;
      OFF = OFF - OFF2;
      SENS = SENS - SENS2;
  }
  pressure = (D1 * SENS / 2097152 - OFF) / 32768;
	return pressure;


  My Attempt Float
  ================
  int32_t pressure;
	float C1,C2,C3,C4,C5,C6;
	float dT,ut,up,baroTemperature,off,off2,sens,sens2,delt;
	ut = ms5611_ut;
	up = ms5611_up;
	C1 = ms5611_c[1] * 32768;
	C2 = ms5611_c[2] * 65536;
  C5 = ms5611_c[5] * 256;
	dT = ut - C5;
  C3 = (dT * (float)ms5611_c[3]) / 256;
	C4 = (dT * (float)ms5611_c[4]) / 128;
	C6 = (dT * (float)ms5611_c[6]) / 8388608;
	baroTemperature = 2000 + C6;
  off             = C2 + C4;
  sens            = C1 + C3;  
  if (baroTemperature < 2000) { // temperature lower than 20st.C 
    delt = baroTemperature - 2000;
    delt  = 5 * delt * delt;
    off2  = delt * 0.5f;
    sens2 = delt * 0.25f;
    if (baroTemperature < -1500) { // temperature lower than -15st.C
      delt   = baroTemperature + 1500;
      delt   = delt * delt;
      off2  += 7.0f * delt;
      sens2 += 11.0f * delt *0.5f;
    }
    off  -= off2; 
    sens -= sens2;
  }
  pressure = (((up * sens ) / 2097152) - off) / 32768;
	return pressure;


  My Attempt MWII Remix
	=====================
  int32_t off2,sens2,delt,baroTemperature,pressure;
  uint32_t C5;

  C5 = ms5611_c[5];
	C5 = C5 << 8;
  int64_t dT       = (int32_t)ms5611_ut - ((int32_t)C5);
  baroTemperature  = 2000 + ((dT * ms5611_c[6])>>23);
  int64_t off      = ((uint32_t)ms5611_c[2] <<16) + ((dT * ms5611_c[4]) >> 7);
  int64_t sens     = ((uint32_t)ms5611_c[1] <<15) + ((dT * ms5611_c[3]) >> 8);

  if (baroTemperature < 2000) { // temperature lower than 20st.C 
    delt = baroTemperature-2000;
    delt  = 5*delt*delt;
    off2  = delt>>1;
    sens2 = delt>>2;
    if (baroTemperature < -1500) { // temperature lower than -15st.C
      delt   = baroTemperature+1500;
      delt   = delt*delt;
      off2  += 7 * delt;
      sens2 += (11 * delt)>>1;
    }
    off  -= off2; 
    sens -= sens2;
  }
  pressure = (((ms5611_up * sens ) >> 21) - off) >> 15;
	return pressure;


    ORIGINAL	BF
		=============
    int32_t temperature, off2 = 0, sens2 = 0, delt;
    int32_t pressure;

    int32_t dT = ms5611_ut - ((uint32_t)ms5611_c[5] << 8);
    int64_t off = ((uint32_t)ms5611_c[2] << 16) + (((int64_t)dT * ms5611_c[4]) >> 7);
    int64_t sens = ((uint32_t)ms5611_c[1] << 15) + (((int64_t)dT * ms5611_c[3]) >> 8);
    temperature = 2000 + (((int64_t)dT * ms5611_c[6]) >> 23);

    if (temperature < 2000) { // temperature lower than 20degC 
        delt = temperature - 2000;
        delt = delt * delt;
        off2 = (5 * delt) >> 1;
        sens2 = (5 * delt) >> 2;
        if (temperature < -1500) { // temperature lower than -15degC
            delt = temperature + 1500;
            delt = delt * delt;
            off2  += 7 * delt;
            sens2 += (11 * delt) >> 1;
        }
    }
    off  -= off2; 
    sens -= sens2;
    pressure = (((ms5611_up * sens ) >> 21) - off) >> 15;
    return pressure; */
