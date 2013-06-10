#include "board.h"

#define MS5611_ADDR    0x77                                      // MS5611, Standard address 0x77
#define CMD_RESET      0x1E                                      // ADC reset command
#define CMD_ADC_READ   0x00                                      // ADC read command
#define CMD_ADC_CONV   0x40                                      // ADC conversion command
#define CMD_ADC_D1     0x00                                      // ADC D1 conversion
#define CMD_ADC_D2     0x10                                      // ADC D2 conversion
#define CMD_ADC_256    0x00                                      // ADC OSR=256
#define CMD_ADC_512    0x02                                      // ADC OSR=512
#define CMD_ADC_1024   0x04                                      // ADC OSR=1024
#define CMD_ADC_2048   0x06                                      // ADC OSR=2048
#define CMD_ADC_4096   0x08                                      // ADC OSR=4096
#define CMD_PROM_RD    0xA0                                      // Prom read command
#define PROM_NB           8                                      // 8*16Bit = 128 Bit

static void     ms5611_reset(void);
static uint16_t ms5611_prom(int8_t coef_num);
static uint32_t ms5611_read_adc(void);
static void     ms5611_start_ut(void);
static void     ms5611_get_ut(void);
static void     ms5611_start_up(void);
static void     ms5611_get_up(void);
static int32_t  ms5611_calculate(void);
static uint32_t ms5611_ut;                                       // static result of temperature measurement
static uint32_t ms5611_up;                                       // static result of pressure measurement
static uint16_t ms5611_c[PROM_NB];                               // on-chip ROM
static uint8_t  ms5611_osr = CMD_ADC_4096;

bool ms5611Detect(baro_t *baro)
{
    uint8_t i;
    if(!i2cRead(MS5611_ADDR, CMD_PROM_RD, 1, &i)) return false;  // If we have a MS5611, it will reply.
    delay(100);
    ms5611_reset();                                              // Reset it
    delay(100);
    for (i = 0; i < PROM_NB; i++) ms5611_c[i] = ms5611_prom(i);  // read all coefficients // read 8 words word:0 = ID; word:1-6 = C1-C6; word:7 = ChkSum
    delay(50);                                                   // CRC check removed, seemed to produce always good results
    baro->ut_delay     = 9500;                                   // baro->ut_delay = 10000;
    baro->up_delay     = 9500;                                   // baro->up_delay = 10000; 
    baro->repeat_delay = 1;                                      // baro->repeat_delay = 4000;
    baro->start_ut     = ms5611_start_ut;
    baro->get_ut       = ms5611_get_ut;
    baro->start_up     = ms5611_start_up;
    baro->get_up       = ms5611_get_up;
    baro->calculate    = ms5611_calculate;
    return true;
}

static void ms5611_reset(void)
{
    i2cWrite(MS5611_ADDR, CMD_RESET, 1);
}

static uint16_t ms5611_prom(int8_t coef_num)
{
    uint8_t  rxbuf[2] = { 0, 0 };
    delay(20);
    i2cRead(MS5611_ADDR, CMD_PROM_RD + coef_num * 2, 2, rxbuf);  // send PROM READ command
    return ((uint16_t)rxbuf[0] << 8) | (uint16_t)rxbuf[1];
}

static uint32_t ms5611_read_adc(void)
{
    uint8_t rxbuf[3];
    i2cRead(MS5611_ADDR, CMD_ADC_READ, 3, rxbuf);                // read ADC
    return ((uint32_t)rxbuf[0] << 16) | ((uint32_t)rxbuf[1] << 8) | (uint32_t)rxbuf[2];
}

static void ms5611_start_ut(void)
{
    i2cWrite(MS5611_ADDR, CMD_ADC_CONV + CMD_ADC_D2 + ms5611_osr, 1); // D2 (temperature) conversion start!
}

static void ms5611_get_ut(void)
{
    uint32_t tmp = ms5611_read_adc();
    if (tmp !=0) ms5611_ut = tmp;                                // Keep old on error
}

static void ms5611_start_up(void)
{
    i2cWrite(MS5611_ADDR, CMD_ADC_CONV + CMD_ADC_D1 + ms5611_osr, 1); // D1 (pressure) conversion start!
}

static void ms5611_get_up(void)
{
    uint32_t tmp = ms5611_read_adc();
    if (tmp !=0) ms5611_up = tmp;                                // Keep old on error
}

static int32_t ms5611_calculate(void)
{
    int32_t  temp, off2 = 0, sens2 = 0, delt, pressure;
    int64_t  dT, off, sens;
    float dTf;
    dTf  = (float)ms5611_ut - ((float)ms5611_c[5] * 256.0f);
    dT   = dTf;
    off  = ((uint32_t)ms5611_c[2] << 16) + ((dTf * (float)ms5611_c[4]) / 128);
    sens = ((uint32_t)ms5611_c[1] << 15) + ((dTf * (float)ms5611_c[3]) / 256);
    temp  = 2000 + ((dT * ms5611_c[6]) >> 23);                   // temperature is not so important not so precise
    if (temp < 2000)                                             // temperature lower than 20degC
    {
        delt  = temp - 2000;
        delt  = 5 * delt * delt;
        off2  = (delt >> 1);
        sens2 = (delt >> 2);
        if (temp < -1500)                                        // temperature lower than -15degC
        {
            delt   = temp + 1500;
            delt   = delt * delt;
            off2  += (  7 * delt);
            sens2 += ((11 * delt) >> 1);
        }
    }
    off     -= off2;
    sens    -= sens2;
    pressure = (((ms5611_up * sens ) >> 21) - off) >> 15;
    return pressure;
}
