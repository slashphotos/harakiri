#include "board.h"
#include "mw.h"
#include <string.h>

#ifndef FLASH_PAGE_COUNT
#define FLASH_PAGE_COUNT 128
#endif

#define FLASH_PAGE_SIZE                 ((uint16_t)0x400)
#define FLASH_WRITE_ADDR                (0x08000000 + (uint32_t)FLASH_PAGE_SIZE * (FLASH_PAGE_COUNT - 1))       // use the last KB for storage

config_t cfg;
const char rcChannelLetters[] = "AERT1234";

static uint8_t EEPROM_CONF_VERSION = 34;
static uint32_t enabledSensors = 0;
static void resetConf(void);

void parseRcChannels(const char *input)
{
    const char *c, *s;

    for (c = input; *c; c++) {
        s = strchr(rcChannelLetters, *c);
        if (s)
            cfg.rcmap[s - rcChannelLetters] = c - input;
    }
}

static uint8_t validEEPROM(void)
{
    const config_t *temp = (const config_t *)FLASH_WRITE_ADDR;
    const uint8_t *p;
    uint8_t chk = 0;

    // check version number
    if (EEPROM_CONF_VERSION != temp->version)
        return 0;

    // check size and magic numbers
    if (temp->size != sizeof(config_t) || temp->magic_be != 0xBE || temp->magic_ef != 0xEF)
        return 0;

    // verify integrity of temporary copy
    for (p = (const uint8_t *)temp; p < ((const uint8_t *)temp + sizeof(config_t)); p++)
        chk ^= *p;

    // checksum failed
    if (chk != 0)
        return 0;

    // looks good, let's roll!
    return 1;
}

void readEEPROM(void)
{
    uint8_t i;

    // Read flash
    memcpy(&cfg, (char *)FLASH_WRITE_ADDR, sizeof(config_t));

    for (i = 0; i < 6; i++)
        lookupPitchRollRC[i] = (2500 + cfg.rcExpo8 * (i * i - 25)) * i * (int32_t) cfg.rcRate8 / 2500;

    for (i = 0; i < 11; i++) {
        int16_t tmp = 10 * i - cfg.thrMid8;
        uint8_t y = 1;
        if (tmp > 0)
            y = 100 - cfg.thrMid8;
        if (tmp < 0)
            y = cfg.thrMid8;
        lookupThrottleRC[i] = 10 * cfg.thrMid8 + tmp * (100 - cfg.thrExpo8 + (int32_t) cfg.thrExpo8 * (tmp * tmp) / (y * y)) / 10;      // [0;1000]
        lookupThrottleRC[i] = cfg.minthrottle + (int32_t) (cfg.maxthrottle - cfg.minthrottle) * lookupThrottleRC[i] / 1000;     // [0;1000] -> [MINTHROTTLE;MAXTHROTTLE]
    }

    cfg.tri_yaw_middle = constrain(cfg.tri_yaw_middle, cfg.tri_yaw_min, cfg.tri_yaw_max);       //REAR
}

void writeParams(uint8_t b)
{
    FLASH_Status status;
    uint32_t i;
    uint8_t chk = 0;
    const uint8_t *p;

    cfg.version = EEPROM_CONF_VERSION;
    cfg.size = sizeof(config_t);
    cfg.magic_be = 0xBE;
    cfg.magic_ef = 0xEF;
    cfg.chk = 0;
    // recalculate checksum before writing
    for (p = (const uint8_t *)&cfg; p < ((const uint8_t *)&cfg + sizeof(config_t)); p++)
        chk ^= *p;
    cfg.chk = chk;

    // write it
    FLASH_Unlock();
    FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);

    if (FLASH_ErasePage(FLASH_WRITE_ADDR) == FLASH_COMPLETE) {
        for (i = 0; i < sizeof(config_t); i += 4) {
            status = FLASH_ProgramWord(FLASH_WRITE_ADDR + i, *(uint32_t *) ((char *) &cfg + i));
            if (status != FLASH_COMPLETE)
                break;          // TODO: fail
        }
    }
    FLASH_Lock();

    readEEPROM();
    if (b) blinkLED(15, 20, 1);
    GPS_set_pids();                       // Set GPS PIDS in any case
    GPS_reset_nav();
}

void checkFirstTime(bool reset)
{
    // check the EEPROM integrity before resetting values
    if (!validEEPROM() || reset)
        resetConf();
}

// Default settings
static void resetConf(void)
{
    int i;
    const int8_t default_align[3][3] = { /* GYRO */ { 0, 0, 0 }, /* ACC */ { 0, 0, 0 }, /* MAG */ { -2, -3, 1 } };

    memset(&cfg, 0, sizeof(config_t));

    cfg.version = EEPROM_CONF_VERSION;
    cfg.mixerConfiguration = MULTITYPE_QUADX;
    featureClearAll();
//    featureSet(FEATURE_VBAT);
//    featureSet(FEATURE_PPM);                    // Crashpilot
//    featureSet(FEATURE_FAILSAFE);               // Crashpilot
//    featureSet(FEATURE_LCD);                    // Crashpilot 
//    featureSet(FEATURE_GPS);                    // Crashpilot 
//    featureSet(FEATURE_PASS);                   // Just pass Throttlechannel Crashpilot

    cfg.P8[ROLL]                  = 40;
    cfg.I8[ROLL]                  = 20;
    cfg.D8[ROLL]                  = 30;

    cfg.P8[PITCH]                 = 40;
    cfg.I8[PITCH]                 = 20;
    cfg.D8[PITCH]                 = 30;

    cfg.P8[YAW]                   = 70;
    cfg.I8[YAW]                   = 45;

    cfg.P8[PIDALT]                = 100;
    cfg.I8[PIDALT]                = 30;
    cfg.D8[PIDALT]                = 80;


    cfg.P8[PIDPOS]                = 0;          // FIND YOUR VALUE
    cfg.I8[PIDPOS]                = 0;          // NOT USED
    cfg.D8[PIDPOS]                = 0;          // NOT USED

    cfg.P8[PIDPOSR]               = 0;          // FIND YOUR VALUE                    // Controls the speed part with my PH logic
    cfg.I8[PIDPOSR]               = 0;          // DANGER "I" may lead to circeling   // Controls the speed part with my PH logic
    cfg.D8[PIDPOSR]               = 0;          // FIND YOUR VALUE                    // Controls the speed part with my PH logic

    cfg.P8[PIDNAVR]               = 20;         // Crashpilot: Double the value
    cfg.I8[PIDNAVR]               = 20;         // NAV_I * 100;                       // Scaling/Purpose unchanged
    cfg.D8[PIDNAVR]               = 80;         // NAV_D * 1000;                      // Scaling/Purpose unchanged

//    cfg.P8[PIDPOS]                = 11;         // APM PH Stock values
//    cfg.I8[PIDPOS]                = 0;
//    cfg.D8[PIDPOS]                = 0;

//    cfg.P8[PIDPOSR]               = 20;         // POSHOLD_RATE_P * 10;
//    cfg.I8[PIDPOSR]               = 8;          // POSHOLD_RATE_I * 100;
//    cfg.D8[PIDPOSR]               = 45;         // POSHOLD_RATE_D * 1000;

//    cfg.P8[PIDNAVR]               = 14;         // NAV_P * 10;
//    cfg.I8[PIDNAVR]               = 20;         // NAV_I * 100;
//    cfg.D8[PIDNAVR]               = 80;         // NAV_D * 1000;

    cfg.P8[PIDLEVEL]              = 55;
    cfg.I8[PIDLEVEL]              = 10;
    cfg.D8[PIDLEVEL]              = 20;

    cfg.P8[PIDMAG]                = 80;         // cfg.P8[PIDVEL] = 0;// cfg.I8[PIDVEL] = 0;// cfg.D8[PIDVEL] = 0;

    cfg.rcRate8                   = 100;
    cfg.rcExpo8                   = 80;         // cfg.rollPitchRate = 0;// cfg.yawRate = 0;// cfg.dynThrPID = 0;
    cfg.thrMid8                   = 50;
    // cfg.thrExpo8 = 0;//for (i = 0; i < CHECKBOXITEMS; i++)//cfg.activate[i] = 0;
    // cfg.angleTrim[0] = 0;// cfg.angleTrim[1] = 0;// cfg.accZero[0] = 0;// cfg.accZero[1] = 0;
    // cfg.accZero[2] = 0;// cfg.mag_declination = 0;    // For example, -6deg 37min, = -637 Japan, format is [sign]dddmm (degreesminutes) default is zero.
    memcpy(&cfg.align, default_align, sizeof(cfg.align));

//    cfg.mag_declination           = 0;          // For example, -6deg 37min, = -637 Japan, format is [sign]dddmm (degreesminutes) default is zero.
    cfg.mag_declination           = 107;        // Crashpilot //cfg.acc_hardware = ACC_DEFAULT;// default/autodetect
    cfg.mag_oldcalib              = 0;          // 1 = old hard iron calibration // 0 = extended calibration (better)
    cfg.acc_hardware              = 2;          // Crashpilot MPU6050
		cfg.acc_lpf_factor            = 100;	      // changed 27.11.2012
    cfg.acc_ins_lpf               = 10;         // General LPF for INS

    cfg.looptime                  = 3000;	      // changed 27.11.2012 //    cfg.acc_lpf_factor = 4;
		cfg.oldcontroller             = 0;          // 1 selects the more or less the original BF main PID Controller
    cfg.gyro_cmpf_factor          = 400;        // default MWC
    cfg.gyro_lpf                  = 42;         // Possible values 256 188 98 42 20 10 (HZ)
    cfg.accz_vel_cf               = 0.985f;     // Crashpilot: Value for complementary filter accz and barovelocity
    cfg.accz_alt_cf               = 0.940f;     // Crashpilot: Value for complementary filter accz and altitude
    cfg.baro_lag                  = 0.3f;       // Lag of Baro
    cfg.barodownscale             = 0.8f;       // Scale downmovement down (because copter drops faster than rising)
    // Autoland
		cfg.autolandrate              = 75;         // Temporary value "64" increase to increase Landingspeed

    cfg.nazedebug                 = 0;          // Crashpilot: 1 = Debug Barovalues //cfg.baro_noise_lpf = 0.6f;// Crashpilot: Not used anymore//cfg.baro_cf = 0.985f;// Crashpilot: Not used anymore
    cfg.moron_threshold           = 32;
    cfg.gyro_smoothing_factor     = 0x00141403; // default factors of 20, 20, 3 for R/P/Y
    cfg.vbatscale                 = 110;
    cfg.vbatmaxcellvoltage        = 43;
    cfg.vbatmincellvoltage        = 33;
		cfg.power_adc_channel         = 0;

    // Radio
    parseRcChannels("AETR1234");
    cfg.deadband                  = 10;         // Crashpilot: A little deadband will not harm our crappy RC
    cfg.yawdeadband               = 15;         // Crashpilot: A little deadband will not harm our crappy RC
    cfg.alt_hold_throttle_neutral = 50;         // Crashpilot: A little deadband will not harm our crappy RC
		cfg.phdeadband                = 15;         // This Deadband adds to cfg.deadband
		
    // cfg.spektrum_hires = 0;
    cfg.midrc                     = 1500;
    cfg.mincheck                  = 1100;
    cfg.maxcheck                  = 1900;
    cfg.retarded_arm              = 0;          // disable arm/disarm on roll left/right
		cfg.auxChannels               = 4;          // cGiesen: Default = 4, then like the standard!
		cfg.killswitchtime            = 0;          // Time in ms when your arm switch becomes a Killswitch, 0 disables the Killswitch

    // Motor/ESC/Servo
    cfg.minthrottle               = 1150;       // ORIG
//	  cfg.minthrottle               = 1220;
//	  cfg.minthrottle               = 1080;
    cfg.maxthrottle               = 1950;
		cfg.passmotor                 = 0;          // Crashpilot: Only used with feature pass. If 0 = all Motors, otherwise specific Motor
    cfg.mincommand                = 1000;
    cfg.motor_pwm_rate            = 400;
    cfg.servo_pwm_rate            = 50;

    // servos
    cfg.yaw_direction             = 1;
    cfg.tri_yaw_middle            = 1500;
    cfg.tri_yaw_min               = 1020;
    cfg.tri_yaw_max               = 2000;

    // flying wing
    cfg.wing_left_min             = 1020;
    cfg.wing_left_mid             = 1500;
    cfg.wing_left_max             = 2000;
    cfg.wing_right_min            = 1020;
    cfg.wing_right_mid            = 1500;
    cfg.wing_right_max            = 2000;
    cfg.pitch_direction_l         = 1;
    cfg.pitch_direction_r         = -1;
    cfg.roll_direction_l          = 1;
    cfg.roll_direction_r          = 1;

    // gimbal
    cfg.gimbal_pitch_gain         = 10;
    cfg.gimbal_roll_gain          = 10;
    cfg.gimbal_flags              = GIMBAL_NORMAL;
    cfg.gimbal_pitch_min          = 1020;
    cfg.gimbal_pitch_max          = 2000;
    cfg.gimbal_pitch_mid          = 1500;
    cfg.gimbal_roll_min           = 1020;
    cfg.gimbal_roll_max           = 2000;
    cfg.gimbal_roll_mid           = 1500;

    // gps/nav
    cfg.gps_type                  = 1;          // GPS_NMEA = 0, GPS_UBLOX = 1, GPS_MTK16 = 2, GPS_MTK19 = 3, GPS_UBLOX_DUMB = 4
    cfg.gps_baudrate              = 38400;      // Changed 5/3/13 was 115200;
    cfg.gps_ins_vel               = 0.72f;      // Crashpilot GPS INS The LOWER the value the closer to gps speed // Dont go to high here
    cfg.gps_lag                   = 1.0f;       // This is the assumed time of GPS Lag, Ublox is supposed to be 0.8 sec behind, 1.5s is better for ublox on the "bench test"
    cfg.gps_phase                 = 0;          // Make a phaseshift +-90 Deg max of GPS output
    cfg.gps_ph_minsat             = 6;          // Minimal Satcount for PH, PH on RTL is still done with 5Sats or more
		cfg.gps_ph_apm                = 0;          // If 1 the original APM PH Controller is used and original parameter scaling
    cfg.gps_ph_settletime         = 400;        // Time in ms before new absolute Position is taken into account after settlespeed reached
    cfg.gps_ph_settlespeed        = 50;         // PH settlespeed in cm/s
    cfg.gps_ph_targetsqrt         = 2;          // This is the speed target of PH. That means if the copter moves faster than that, the maximal tiltangle reduced dramatically. Just think of the value as a working point for the sqrt brake
		cfg.gps_minanglepercent       = 10;         // Percent 1 - 100% of gps_maxangle for minimal tilt, as lower limit for "gps_ph_targetsqrt"
    cfg.gps_maxangle              = 25;         // maximal over all GPS bank angle
    cfg.gps_phmove_speed          = 0.0f;       // DONT USE THIS FOR FLIGHT! PH move speed // 0 disables PH move - recommended!!
    cfg.gps_wp_radius             = 200;
    cfg.gps_rtl_minhight          = 20;         // (0-200) Minimal RTL hight in m, 0 disables feature
//	  cfg.gps_rtl_minhight          = 0;          // (0-200) Minimal RTL hight in m, 0 disables feature
    cfg.gps_rtl_mindist           = 0;          // 0 Disables. Minimal distance for RTL in m, otherwise it will just autoland, prevent Failsafe jump in your face, when arming copter and turning off TX
    cfg.gps_rtl_flyaway           = 0;          // 0 Disables. If during RTL the distance increases beyond this value (in meters), something is wrong, autoland
    cfg.gps_yaw                   = 30;         // Thats the MAG P during GPS functions, substitute for "cfg.P8[PIDMAG]"
    cfg.nav_rtl_lastturn          = 1;          // 1 = when copter gets to home position it rotates it's head to takeoff direction independend of nav_controls_heading
    cfg.nav_slew_rate             = 50;         // 0 Enables Spikefilter 
    cfg.nav_tail_first            = 0;          // 1 = Copter comes back with ass first (only works with nav_controls_heading = 1) 
//    cfg.nav_tail_first            = 1;          // 1 = Copter comes back with ass first (only works with nav_controls_heading = 1) 
    cfg.nav_controls_heading      = 1;          // 1 = Nav controls YAW during WP ONLY
    cfg.nav_speed_min             = 150;
    cfg.nav_speed_max             = 350;

    // Failsafe Variables
    cfg.failsafe_delay            = 10;         // in 0.1s (10 = 1sec)
    cfg.failsafe_off_delay        = 200;        // in 0.1s (200 = 20sec)
    cfg.failsafe_throttle         = 1200;       // decent default which should always be below hover throttle for people.
    cfg.failsafe_deadpilot        = 0;		      // DONT USE, EXPERIMENTAL Time in sec when FS is engaged after idle on THR/YAW/ROLL/PITCH, 0 disables max 250
    cfg.failsafe_justph           = 0;          // Does just PH&Autoland an not RTL, use this in difficult areas with many obstacles to avoid RTL crash into something

    // serial (USART1) baudrate
    cfg.serial_baudrate           = 115200;

	  // LED Stuff
	  cfg.led_invert                = 0;          // Crashpilot: Inversion of LED 0&1 Partly implemented because Bootup is not affected
	  cfg.LED_Type                  = 1;		      // 1=MWCRGB / 2=MONO_LED / 3=LEDRing
	  cfg.LED_Pinout                = 1;		      // rc6
	  cfg.LED_ControlChannel        = 8;		      // AUX4 (Channel 8)
	  cfg.LED_Armed                 = 0;		      // 0 = Show LED only if armed, 1 = always show LED
	  cfg.LED_Pattern1			        = 1300; 		  // 32bit bit pattern to have flickering led patterns / the pattern for MWCRGB 1000-2000
	  cfg.LED_Pattern2			        = 1800; 		  // 32bit bit pattern to have flickering led patterns / the pattern for MWCRGB 1000-2000
	  cfg.LED_Pattern3			        = 1900; 		  // 32bit bit pattern to have flickering led patterns / the pattern for MWCRGB 1000-2000
	  cfg.LED_Toggle_Delay          = 0x08; 	    // slow down LED_Pattern

	  // Sonar Stuff
	  cfg.SONAR_Pinout              = 1;          // cGiesen: rc78

    // custom mixer. clear by defaults.
    for (i = 0; i < MAX_MOTORS; i++) cfg.customMixer[i].throttle = 0.0f;
    writeParams(0);
}

bool sensors(uint32_t mask)
{
    return enabledSensors & mask;
}

void sensorsSet(uint32_t mask)
{
    enabledSensors |= mask;
}

void sensorsClear(uint32_t mask)
{
    enabledSensors &= ~(mask);
}

uint32_t sensorsMask(void)
{
    return enabledSensors;
}

bool feature(uint32_t mask)
{
    return cfg.enabledFeatures & mask;
}

void featureSet(uint32_t mask)
{
    cfg.enabledFeatures |= mask;
}

void featureClear(uint32_t mask)
{
    cfg.enabledFeatures &= ~(mask);
}

void featureClearAll()
{
    cfg.enabledFeatures = 0;
}

uint32_t featureMask(void)
{
    return cfg.enabledFeatures;
}
