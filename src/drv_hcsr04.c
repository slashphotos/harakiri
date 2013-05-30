#include "board.h"
#include "mw.h"

#ifdef SONAR
/*
Currently supported Sonars: (the links are just examples, I didn't buy them there)
HC-SR04  *** Warning: HC-SR04 operates at +5V *** (google that, price around 5$ range about 2m)

DaddyWalross did an I2C "pimp" of the HC-SR04 with the idea in mind to connect several modules at once:
http://fpv-community.de/showthread.php?20988-I%B2C-Sonarplatinen

Maxbotics Sonars using PWM readout (expensive stuff, probably some people have them from APM)

ToDo?:
Other I2C Sonars: SRF02 SRF08 SRF10 SRC235 (all the same protocol)
http://www.shop.robotikhardware.de/shop/catalog/product_info.php?products_id=121

SOME INFO:

UncompDivisor:
==============
+35C 352,17 m/s = 56,79 is the divisor
+20C 343,46 m/s = 29,115 us/cm so we measure double time so: 58,23 is the divisor
-10C 325,35 m/s = 61,47 is the divisor

Error chart we measure 5823us:
+35 C  102 CM
+20 C  100 CM
-10 C   95 CM
So no Temp compensation and taking "58" as divisor seems sufficient to me.
But for the precise people, here is further info:

Temp. Compensation (taken from Maxbotics):
==========================================
(Source: http://www.maxbotix.com/documents/Temperature_Compensation.pdf)

40C to +65C (with limited operation to +85C).
Temperature Compensation that uses the time of flight in seconds, and temperature in degrees centigrade and yields the
distance in meters works for all of our products.

Dm = TOF *((20.05*SQRT(Tc+273.15))/2)
TOF is the measured Time Of Flight in seconds,
Tc is the ambient temperature in degrees C,
Dm is the distance in meters.

For 23 degrees C and 0.0058 seconds (or 5.8mS) the
distance calculates to 1.0006 meter.
If using the Serial output, first convert the distance reported by the sensor to TOF by using 147uS per inch
(TOF = inches * 1.47E-4) or 58uS per cm (TOF = cm * 5.8E-5) and then insert the TOF into the above formula.
*/

#define SONARDW_ADDRESS      0x20       // DaddyW I2C SONAR, Standard address 0x20 to 0x27  7bit!!!
#define SONARDW_DISTANCE_OUT 0x32       // NOT USED HERE #define SONARDW_ADC_OUT           0x33
#define SR04Cycle       60              // Cycletime in ms
#define MaxboticsCycle 100              // Cycletime in ms
#define DaddyWCycle     60              // Since DaddyW Sonar uses SR04 we assume that updaterate
#define UncompDivisor   58              // Temp Compensation currently not implemented

static uint16_t  trigger_pin;           // why not 8 bit ?
static uint16_t  echo_pin;
static uint32_t  exti_line;
static uint8_t   exti_pin_source;
static IRQn_Type exti_irqn;

static uint32_t  last_measurement;
static uint16_t  PulseInUs;
static uint16_t  PulseLimitInUs;

void ECHO_EXTI_IRQHandler(void)
{
    static uint32_t timing_start;
    uint32_t        timing_stop;
    uint32_t        pulse_duration;
    if(GPIO_ReadInputDataBit(GPIOB, echo_pin) != 0)
        timing_start = micros();                              // Up flank detected
    else
    {                                                         // Since interrupt is triggered by changes this is probably the downflank or a problem of contact....
        timing_stop = micros();
        if(timing_stop > timing_start)
        {
          pulse_duration = timing_stop - timing_start;
          if (pulse_duration > 174 && pulse_duration < PulseLimitInUs) // something like 3 cm - X (X selected by sonartype)
              PulseInUs = pulse_duration;
          else
              PulseInUs = 0;                                  // Some wacky pulse measured set Result to zero
        }
    }
    EXTI_ClearITPendingBit(exti_line);
}

void EXTI1_IRQHandler(void)
{
    ECHO_EXTI_IRQHandler();
}

void EXTI9_5_IRQHandler(void)
{
    ECHO_EXTI_IRQHandler();
}

bool hcsr04_init(sonar_config_t config)
{
    bool returnvalue = false;
    GPIO_InitTypeDef GPIO_InitStructure;
    EXTI_InitTypeDef EXTIInit;
	  uint8_t bufdaddy[2];                                        // Dummy for i2c testread
    // enable AFIO for EXTI support - already done is drv_system.c
    // RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | RCC_APB2Periph, ENABLE); 
    // cfg.snr_type = X;  0 = PWM56, 1 = RC78, 2 = I2C (DaddyWalross), 3 = MBPWM56, 4 = MBRC78
    switch(config)
    {
    case sonar_pwm56:
        trigger_pin = GPIO_Pin_8;                               // PWM5 (PB8) - 5v tolerant
        echo_pin = GPIO_Pin_9;                                  // PWM6 (PB9) - 5v tolerant
        exti_line = EXTI_Line9;
        exti_pin_source = GPIO_PinSource9;
        exti_irqn = EXTI9_5_IRQn;
        returnvalue = true;
        break;
    case sonar_rc78:
        trigger_pin = GPIO_Pin_0;                               // RX7 (PB0) - only 3.3v ( add a 1K Ohms resistor )
        echo_pin = GPIO_Pin_1;                                  // RX8 (PB1) - only 3.3v ( add a 1K Ohms resistor )
        exti_line = EXTI_Line1;
        exti_pin_source = GPIO_PinSource1;
        exti_irqn = EXTI1_IRQn;
        returnvalue = true;
        break;
    case sonar_i2cDW:                                           // Deal with I2C daddy walross sonar
        delay(1000);                                            // sleep for 1000ms to startup sonar
        returnvalue = i2cRead(SONARDW_ADDRESS, SONARDW_DISTANCE_OUT, 2, bufdaddy);
        break;
    }
    if (config == sonar_pwm56 || config == sonar_rc78)
    {
        if (cfg.snr_type == 0 || cfg.snr_type == 1)             // Check for HC-SR04    
            PulseLimitInUs = 24000;                             // HC-SR04 Limit 413 cm
        if (cfg.snr_type == 3 || cfg.snr_type == 4)             // Check for Maxbotics (no "else" stuff here to make it expandable)
            PulseLimitInUs = 62000;                             // Datasheet Limit 62ms
        // tp - trigger pin 
        GPIO_InitStructure.GPIO_Pin = trigger_pin;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
        GPIO_Init(GPIOB, &GPIO_InitStructure);
        // ep - echo pin
        GPIO_InitStructure.GPIO_Pin = echo_pin;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
        GPIO_Init(GPIOB, &GPIO_InitStructure);
        // setup external interrupt on echo pin
        GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, exti_pin_source);
        EXTI_ClearITPendingBit(exti_line);
        EXTIInit.EXTI_Line = exti_line;
        EXTIInit.EXTI_Mode = EXTI_Mode_Interrupt;
        EXTIInit.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
        EXTIInit.EXTI_LineCmd = ENABLE;    
        EXTI_Init(&EXTIInit);    
        NVIC_EnableIRQ(exti_irqn);
        last_measurement = 0;                                   // Force 1st measurement in XXX_get_distance
    }
    return returnvalue;                                         // Return the status of initialization
}

bool hcsr04_get_distancePWM(volatile int16_t* distance)         // HC-SR04
{
    uint32_t current_time = millis();
    if(current_time < (last_measurement + SR04Cycle)) return false; // repeat interval should be greater 60ms. Avoid interference between measurements.
    last_measurement = current_time;
    *distance = PulseInUs / UncompDivisor;                      // Calculate
    GPIO_SetBits(GPIOB, trigger_pin);                           // Trigger new "Ping"
    delayMicroseconds(11);                                      // The width of trig signal must be greater than 10us
    GPIO_ResetBits(GPIOB, trigger_pin);
    return true;
}

bool hcsr04_get_distancePWMMB(volatile int16_t* distance)       // Maxbotics PWM support. Tested on MB1200 XL-MaxSonar-EZ0.
{                                                               // Just Connect Echopin!!
    uint32_t current_time = millis();
    if(current_time < (last_measurement + MaxboticsCycle)) return false;// MaxBtx needs min 99 ms here
    last_measurement = current_time;
    *distance = PulseInUs / UncompDivisor;                      // Calculate, no need to trigger MB keeps talking forever
    return true;
}

bool hcsr04_get_i2c_distanceDW(volatile int16_t* distance)
{
    uint8_t buf[2];
    int16_t temp;
    bool    retvalue = false;
    uint32_t current_time = millis();
    if(current_time < (last_measurement + DaddyWCycle)) return false; // Hopefully DaddyW Sonar works with that timing
    last_measurement = current_time;
    retvalue = i2cRead(SONARDW_ADDRESS, SONARDW_DISTANCE_OUT, 2, buf);
    if(retvalue)
    {
        temp = (int16_t)((buf[1] << 8) | buf[0]);
        *distance = temp / UncompDivisor;
	  }
    else *distance = -1;                                        // Error, Sonar not responding
    return retvalue;
}
#endif