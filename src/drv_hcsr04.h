#pragma once

typedef enum
{
    sonar_pwm56,
    sonar_rc78,
    sonar_i2cDW,
} sonar_config_t;

bool hcsr04_init(sonar_config_t config);
void hcsr04_get_distancePWM(volatile int16_t* distance);
void hcsr04_get_i2c_distanceDW(volatile int16_t* distance);
