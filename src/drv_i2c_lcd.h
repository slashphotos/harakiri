#pragma once

#define OLED_address          0x3C  // OLED at address 0x3C in 7bit

extern void i2c_clear_OLED(void);
extern void i2c_clr_line(uint8_t line);
extern bool i2c_OLED_init(void);
extern bool initI2cLCD(bool cli);
extern void i2cOSD(void);
extern void i2c_OLED_send_char(unsigned char ascii);
extern void i2c_OLED_LCDsetLine(uint8_t line);
 
