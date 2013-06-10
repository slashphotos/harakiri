#include "board.h"
#include "mw.h"

static uint8_t  ledIndex;                                   // Is the led GPIO channel enabled
static uint16_t ledtoggle_pin;
static uint8_t  ledDelay;                                   // make circle slower

static void ledEnable()
{
    digitalHi( GPIOA, ledtoggle_pin );
}

static void ledDisable()
{
    digitalLo( GPIOA, ledtoggle_pin );
}

void ledToggleInit()
{
    GPIO_InitTypeDef GPIO_InitStructure;                    // Set the led pin to be an output
    switch(cfg.LED_Pinout)
    {
    case 0:
        ledtoggle_pin = GPIO_Pin_6;                         // RC5 (PB8) - 3.3v
        break;
    case 1:
        ledtoggle_pin = GPIO_Pin_7;                         // RC6 (PB8) - 3.3v
        break;
    }
    GPIO_InitStructure.GPIO_Pin   = ledtoggle_pin;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;       // Set the pin in push/pull mode
    GPIO_Init( GPIOA, &GPIO_InitStructure);
    ledDisable();
}

void ledToggleUpdate( bool activated )                      // Ignore when we're not using leds
{
    uint8_t bit;
    if ( !activated )
    {
        ledDisable();
        return;
    }
    ledDelay++;
    if (ledDelay == LED_Value_Delay)
    {
        ledDelay=0;
        bit = (ledIndex++ >> 1) & 31;
        if ( LED_Value & ( 1 << bit ) )
        {
            ledEnable();  // LED0_ON;
        }
        else
        {
            ledDisable(); // LED0_OFF;
        }
    }
}
