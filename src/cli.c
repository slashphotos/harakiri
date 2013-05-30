#include "board.h"
#include "mw.h"

// we unset this on 'exit'
extern uint8_t cliMode;
static void cliAux(char *cmdline);
static void cliCMix(char *cmdline);
static void cliDefaults(char *cmdline);
static void cliDump(char *cmdLine);
static void cliExit(char *cmdline);
static void cliFeature(char *cmdline);
static void cliHelp(char *cmdline);
static void cliMap(char *cmdline);
static void cliMixer(char *cmdline);
static void cliSave(char *cmdline);
static void cliSet(char *cmdline);
static void cliStatus(char *cmdline);
static void cliVersion(char *cmdline);
static void cliScanI2Cbus(char *cmdline);
static void LCDinit(void);
static void LCDoff(void);
static void LCDclear(void);
static void LCDline1(void);
static void LCDline2(void);

// from sensors.c
extern uint8_t batteryCellCount;
extern uint8_t accHardware;

// from config.c RC Channel mapping
extern const char rcChannelLetters[];

// buffer
static char cliBuffer[48];
static uint32_t bufferIndex = 0;

static float _atof(const char *p);
static char *ftoa(float x, char *floatString);

// sync this with MultiType enum from mw.h
const char * const mixerNames[] =
{
    "TRI", "QUADP", "QUADX", "BI",
    "GIMBAL", "Y6", "HEX6",
    "FLYING_WING", "Y4", "HEX6X", "OCTOX8", "OCTOFLATP", "OCTOFLATX",
    "AIRPLANE", "HELI_120_CCPM", "HELI_90_DEG", "VTAIL4", "CUSTOM", NULL
};

// sync this with AvailableFeatures enum from board.h
const char * const featureNames[] =
{
    "PPM", "VBAT", "INFLIGHT_ACC_CAL", "SPEKTRUM", "MOTOR_STOP",
    "SERVO_TILT", "GYRO_SMOOTHING", "LED", "GPS",
    "FAILSAFE", "SONAR", "TELEMETRY", "PASS", "POWERMETER", "LCD",
    NULL
};

// sync this with AvailableSensors enum from board.h
const char * const sensorNames[] =
{
    "ACC", "BARO", "MAG", "SONAR", "GPS", NULL
};

//
const char * const accNames[] =
{
    "", "ADXL345", "MPU6050", "MMA845x", NULL
};

typedef struct
{
    char *name;
    char *param;
    void (*func)(char *cmdline);
} clicmd_t;

// should be sorted a..z for bsearch()
const clicmd_t cmdTable[] =
{
    { "aux", "feature_name auxflag or blank for list", cliAux },
    { "cmix", "design custom mixer", cliCMix },
    { "defaults", "reset to defaults and reboot", cliDefaults },
    { "dump", "print configurable settings in a pastable form", cliDump },
    { "exit", "exit and reboot", cliExit },
    { "feature", "list or -val or val", cliFeature },
    { "help", "", cliHelp },
    { "map", "mapping of rc channel order", cliMap },
    { "mixer", "mixer name or list", cliMixer },
    { "save", "save and reboot", cliSave },
//    { "scani2cbus", "scan for I2C devices", cliScanI2Cbus }, EDIT: Moved to clistatus
    { "set", "name=value or blank or * for list", cliSet },
    { "status", "show system status & I2C devices", cliStatus },
    { "version", "", cliVersion },
};
#define CMD_COUNT (sizeof(cmdTable) / sizeof(cmdTable[0]))

typedef enum
{
    VAR_UINT8,
    VAR_INT8,
    VAR_UINT16,
    VAR_INT16,
    VAR_UINT32,
    VAR_FLOAT
} vartype_e;

typedef struct
{
    const char *name;
    const uint8_t type; // vartype_e
    void *ptr;
    const int32_t min;
    const int32_t max;
    const uint8_t lcd; // 1 = Displayed in LCD // 0 = Not displayed
} clivalue_t;

const clivalue_t valueTable[] =
{
    { "deadband",                  VAR_UINT8,  &cfg.deadband,                    0,         32, 1 },
    { "yawdeadband",               VAR_UINT8,  &cfg.yawdeadband,                 0,        100, 1 },
    { "alt_hold_throttle_neutral", VAR_UINT8,  &cfg.alt_hold_throttle_neutral,   1,        20,  1 },
    { "midrc",                     VAR_UINT16, &cfg.midrc,                    1200,       1700, 1 },
    { "rc_rate",                   VAR_UINT8,  &cfg.rcRate8,                     0,        250, 1 },
    { "rc_expo",                   VAR_UINT8,  &cfg.rcExpo8,                     0,        100, 1 },
    { "thr_mid",                   VAR_UINT8,  &cfg.thrMid8,                     0,        100, 1 },
    { "thr_expo",                  VAR_UINT8,  &cfg.thrExpo8,                    0,        250, 1 },
    { "roll_pitch_rate",           VAR_UINT8,  &cfg.rollPitchRate,               0,        100, 1 },
    { "yawrate",                   VAR_UINT8,  &cfg.yawRate,                     0,        100, 1 },
    { "minthrottle",               VAR_UINT16, &cfg.minthrottle,                 0,       2000, 0 },
    { "maxthrottle",               VAR_UINT16, &cfg.maxthrottle,                 0,       2000, 0 },
    { "passmotor",                 VAR_UINT8,  &cfg.passmotor,                   0,         10, 0 },
    { "mincommand",                VAR_UINT16, &cfg.mincommand,                  0,       2000, 0 },
    { "mincheck",                  VAR_UINT16, &cfg.mincheck,                    0,       2000, 0 },
    { "maxcheck",                  VAR_UINT16, &cfg.maxcheck,                    0,       2000, 0 },
    { "retarded_arm",              VAR_UINT8,  &cfg.retarded_arm,                0,          1, 0 },
    { "killswitchtime",            VAR_UINT16, &cfg.killswitchtime,              0,      10000, 1 },
    { "failsafe_delay",            VAR_UINT8,  &cfg.failsafe_delay,              0,         40, 1 },
    { "failsafe_off_delay",        VAR_UINT8,  &cfg.failsafe_off_delay,          0,        200, 1 },
    { "failsafe_throttle",         VAR_UINT16, &cfg.failsafe_throttle,        1000,       2000, 1 },
    { "failsafe_deadpilot",        VAR_UINT8,  &cfg.failsafe_deadpilot,          0,        250, 1 },
    { "failsafe_justph",           VAR_UINT8,  &cfg.failsafe_justph,             0,          1, 1 },
    { "failsafe_ignoreSNR",        VAR_UINT8,  &cfg.failsafe_ignoreSNR,          0,          1, 1 },
    { "motor_pwm_rate",            VAR_UINT16, &cfg.motor_pwm_rate,             50,        498, 0 },
    { "servo_pwm_rate",            VAR_UINT16, &cfg.servo_pwm_rate,             50,        498, 0 },
    { "serial_baudrate",           VAR_UINT32, &cfg.serial_baudrate,          1200,     115200, 0 },
    { "spektrum_hires",            VAR_UINT8,  &cfg.spektrum_hires,              0,          1, 0 },
    { "vbatscale",                 VAR_UINT8,  &cfg.vbatscale,                  10,        200, 0 },
    { "vbatmaxcellvoltage",        VAR_UINT8,  &cfg.vbatmaxcellvoltage,         10,         50, 0 },
    { "vbatmincellvoltage",        VAR_UINT8,  &cfg.vbatmincellvoltage,         10,         50, 0 },
    { "power_adc_channel",         VAR_UINT8,  &cfg.power_adc_channel,           0,          9, 0 },
    { "yaw_direction",             VAR_INT8,   &cfg.yaw_direction,              -1,          1, 0 },
    { "tri_yaw_middle",            VAR_UINT16, &cfg.tri_yaw_middle,              0,       2000, 1 },
    { "tri_yaw_min",               VAR_UINT16, &cfg.tri_yaw_min,                 0,       2000, 1 },
    { "tri_yaw_max",               VAR_UINT16, &cfg.tri_yaw_max,                 0,       2000, 1 },
    { "wing_left_min",             VAR_UINT16, &cfg.wing_left_min,               0,       2000, 0 },
    { "wing_left_mid",             VAR_UINT16, &cfg.wing_left_mid,               0,       2000, 0 },
    { "wing_left_max",             VAR_UINT16, &cfg.wing_left_max,               0,       2000, 0 },
    { "wing_right_min",            VAR_UINT16, &cfg.wing_right_min,              0,       2000, 0 },
    { "wing_right_mid",            VAR_UINT16, &cfg.wing_right_mid,              0,       2000, 0 },
    { "wing_right_max",            VAR_UINT16, &cfg.wing_right_max,              0,       2000, 0 },
    { "pitch_direction_l",         VAR_INT8,   &cfg.pitch_direction_l,          -1,          1, 0 },
    { "pitch_direction_r",         VAR_INT8,   &cfg.pitch_direction_r,          -1,          1, 0 },
    { "roll_direction_l",          VAR_INT8,   &cfg.roll_direction_l,           -1,          1, 0 },
    { "roll_direction_r",          VAR_INT8,   &cfg.roll_direction_r,           -1,          1, 0 },
    { "gimbal_flags",              VAR_UINT8,  &cfg.gimbal_flags,                0,        255, 0 },
    { "gimbal_pitch_gain",         VAR_INT8,   &cfg.gimbal_pitch_gain,        -100,        100, 0 },
    { "gimbal_roll_gain",          VAR_INT8,   &cfg.gimbal_roll_gain,         -100,        100, 0 },
    { "gimbal_pitch_min",          VAR_UINT16, &cfg.gimbal_pitch_min,          100,       3000, 0 },
    { "gimbal_pitch_max",          VAR_UINT16, &cfg.gimbal_pitch_max,          100,       3000, 0 },
    { "gimbal_pitch_mid",          VAR_UINT16, &cfg.gimbal_pitch_mid,          100,       3000, 0 },
    { "gimbal_roll_min",           VAR_UINT16, &cfg.gimbal_roll_min,           100,       3000, 0 },
    { "gimbal_roll_max",           VAR_UINT16, &cfg.gimbal_roll_max,           100,       3000, 0 },
    { "gimbal_roll_mid",           VAR_UINT16, &cfg.gimbal_roll_mid,           100,       3000, 0 },
    { "al_barolr",                 VAR_UINT8,  &cfg.al_barolr,                  30,        200, 1 },
    { "al_snrlr",                  VAR_UINT8,  &cfg.al_snrlr,                   30,        200, 1 },
    { "al_lndpercent",             VAR_UINT8,  &cfg.al_lndpercent,               0,         80, 1 },
    { "align_gyro_x",              VAR_INT8,   &cfg.align[ALIGN_GYRO][0],       -3,          3, 0 },
    { "align_gyro_y",              VAR_INT8,   &cfg.align[ALIGN_GYRO][1],       -3,          3, 0 },
    { "align_gyro_z",              VAR_INT8,   &cfg.align[ALIGN_GYRO][2],       -3,          3, 0 },
    { "align_acc_x",               VAR_INT8,   &cfg.align[ALIGN_ACCEL][0],      -3,          3, 0 },
    { "align_acc_y",               VAR_INT8,   &cfg.align[ALIGN_ACCEL][1],      -3,          3, 0 },
    { "align_acc_z",               VAR_INT8,   &cfg.align[ALIGN_ACCEL][2],      -3,          3, 0 },
    { "align_mag_x",               VAR_INT8,   &cfg.align[ALIGN_MAG][0],        -3,          3, 0 },
    { "align_mag_y",               VAR_INT8,   &cfg.align[ALIGN_MAG][1],        -3,          3, 0 },
    { "align_mag_z",               VAR_INT8,   &cfg.align[ALIGN_MAG][2],        -3,          3, 0 },
    { "acc_hardware",              VAR_UINT8,  &cfg.acc_hardware,                0,          3, 0 },
    { "acc_lpf_factor",            VAR_UINT8,  &cfg.acc_lpf_factor,              1,        250, 1 },
    { "acc_ins_lpf",               VAR_UINT8,  &cfg.acc_ins_lpf,                 1,        250, 1 },
    { "acc_trim_pitch",            VAR_INT16,  &cfg.angleTrim[PITCH],         -300,        300, 1 },
    { "acc_trim_roll",             VAR_INT16,  &cfg.angleTrim[ROLL],          -300,        300, 1 },
    { "gyro_lpf",                  VAR_UINT16, &cfg.gyro_lpf,                    0,        256, 0 },
    { "gyro_cmpf_factor",          VAR_UINT16, &cfg.gyro_cmpf_factor,          100,       1000, 1 },
    { "accz_vel_cf",               VAR_FLOAT,  &cfg.accz_vel_cf,                 0,          1, 1 },
    { "accz_alt_cf",               VAR_FLOAT,  &cfg.accz_alt_cf,                 0,          1, 1 },
    { "baro_lag",                  VAR_FLOAT,  &cfg.baro_lag,                    0,         10, 1 },
    { "barodownscale",             VAR_FLOAT,  &cfg.barodownscale,               0,          1, 1 },
    { "baro_debug",                VAR_UINT8,  &cfg.baro_debug,                  0,          1, 0 },
    { "moron_threshold",           VAR_UINT8,  &cfg.moron_threshold,             0,        128, 0 },
    { "mag_declination",           VAR_INT16,  &cfg.mag_declination,        -18000,      18000, 1 },
    { "mag_oldcalib",              VAR_UINT8,  &cfg.mag_oldcalib,                0,          1, 0 },
    { "gps_baudrate",              VAR_UINT32, &cfg.gps_baudrate,             1200,     115200, 0 },
    { "gps_debug",                 VAR_UINT8,  &cfg.gps_debug,                   0,          1, 0 },    
    { "gps_type",                  VAR_UINT8,  &cfg.gps_type,                    0,          9, 0 },
    { "gps_ins_vel",               VAR_FLOAT,  &cfg.gps_ins_vel,                 0,          1, 1 },
    { "gps_lag",                   VAR_FLOAT,  &cfg.gps_lag,                     0,         20, 1 },
    { "gps_phase",                 VAR_FLOAT,  &cfg.gps_phase,                 -30,         30, 1 },
    { "gps_ph_minsat",             VAR_UINT8,  &cfg.gps_ph_minsat,               5,         10, 1 },
    { "gps_ph_settlespeed",        VAR_UINT16, &cfg.gps_ph_settlespeed,          0,       1000, 1 },
    { "gps_ph_targetsqrt",         VAR_UINT16, &cfg.gps_ph_targetsqrt,           1,      60000, 1 },
    { "gps_maxangle",              VAR_UINT8,  &cfg.gps_maxangle,               10,         45, 1 },
    { "gps_minanglepercent",       VAR_UINT8,  &cfg.gps_minanglepercent,         1,        100, 1 },
    { "gps_wp_radius",             VAR_UINT16, &cfg.gps_wp_radius,               0,       2000, 1 },
    { "gps_rtl_minhight",          VAR_UINT16, &cfg.gps_rtl_minhight,            0,        200, 1 },
    { "gps_rtl_mindist",           VAR_UINT8,  &cfg.gps_rtl_mindist,             0,         50, 1 },
    { "gps_rtl_flyaway",           VAR_UINT8,  &cfg.gps_rtl_flyaway,             0,        100, 1 },
    { "gps_yaw",                   VAR_UINT8,  &cfg.gps_yaw,                    20,        150, 1 },
    { "nav_rtl_lastturn",          VAR_UINT8,  &cfg.nav_rtl_lastturn,            0,          1, 1 },
    { "nav_speed_min",             VAR_INT16,  &cfg.nav_speed_min,              10,       2000, 1 },
    { "nav_speed_max",             VAR_INT16,  &cfg.nav_speed_max,              50,       2000, 1 },
    { "nav_slew_rate",             VAR_UINT8,  &cfg.nav_slew_rate,              10,        200, 1 },
    { "nav_controls_heading",      VAR_UINT8,  &cfg.nav_controls_heading,        0,          1, 1 },
    { "nav_tail_first",            VAR_UINT8,  &cfg.nav_tail_first,              0,          1, 1 },
    { "gps_pos_p",                 VAR_UINT8,  &cfg.P8[PIDPOS],                  0,        200, 1 },
    { "gps_pos_i",                 VAR_UINT8,  &cfg.I8[PIDPOS],                  0,        200, 0 },
    { "gps_pos_d",                 VAR_UINT8,  &cfg.D8[PIDPOS],                  0,        200, 0 },
    { "gps_posr_p",                VAR_UINT8,  &cfg.P8[PIDPOSR],                 0,        200, 1 },
    { "gps_posr_i",                VAR_UINT8,  &cfg.I8[PIDPOSR],                 0,        200, 1 },
    { "gps_posr_d",                VAR_UINT8,  &cfg.D8[PIDPOSR],                 0,        200, 1 },
    { "gps_nav_p",                 VAR_UINT8,  &cfg.P8[PIDNAVR],                 0,        200, 1 },
    { "gps_nav_i",                 VAR_UINT8,  &cfg.I8[PIDNAVR],                 0,        200, 1 },
    { "gps_nav_d",                 VAR_UINT8,  &cfg.D8[PIDNAVR],                 0,        200, 1 },
    { "LED_invert",                VAR_UINT8,  &cfg.LED_invert,                  0,          1, 0 },
    { "looptime",                  VAR_UINT16, &cfg.looptime,                    0,       9000, 1 },
    { "oldcontroller",             VAR_UINT8,  &cfg.oldcontroller,               0,          1, 1 },
    { "p_pitch",                   VAR_UINT8,  &cfg.P8[PITCH],                   0,        200, 1 },
    { "i_pitch",                   VAR_UINT8,  &cfg.I8[PITCH],                   0,        200, 1 },
    { "d_pitch",                   VAR_UINT8,  &cfg.D8[PITCH],                   0,        200, 1 },
    { "p_roll",                    VAR_UINT8,  &cfg.P8[ROLL],                    0,        200, 1 },
    { "i_roll",                    VAR_UINT8,  &cfg.I8[ROLL],                    0,        200, 1 },
    { "d_roll",                    VAR_UINT8,  &cfg.D8[ROLL],                    0,        200, 1 },
    { "p_yaw",                     VAR_UINT8,  &cfg.P8[YAW],                     0,        200, 1 },
    { "i_yaw",                     VAR_UINT8,  &cfg.I8[YAW],                     0,        200, 1 },
    { "d_yaw",                     VAR_UINT8,  &cfg.D8[YAW],                     0,        200, 1 },
    { "p_alt",                     VAR_UINT8,  &cfg.P8[PIDALT],                  0,        200, 1 },
    { "i_alt",                     VAR_UINT8,  &cfg.I8[PIDALT],                  0,        200, 1 },
    { "d_alt",                     VAR_UINT8,  &cfg.D8[PIDALT],                  0,        200, 1 },
    { "p_level",                   VAR_UINT8,  &cfg.P8[PIDLEVEL],                0,        200, 1 },
    { "i_level",                   VAR_UINT8,  &cfg.I8[PIDLEVEL],                0,        200, 1 },
    { "d_level",                   VAR_UINT8,  &cfg.D8[PIDLEVEL],                0,        200, 1 },
    { "auxChannels",               VAR_UINT8,  &cfg.auxChannels,                 4,         14, 0 },
    { "snr_type",                  VAR_UINT8,  &cfg.snr_type,                    0,          4, 0 },
    { "snr_min",                   VAR_UINT8,  &cfg.snr_min,                     0,        200, 1 },
    { "snr_max",                   VAR_UINT16, &cfg.snr_max,                    50,        800, 1 },
    { "snr_debug",                 VAR_UINT8,  &cfg.snr_debug,                   0,          1, 0 },
    { "snr_tilt",                  VAR_UINT8,  &cfg.snr_tilt,                   10,         50, 1 },
    { "snr_cf",                    VAR_FLOAT,  &cfg.snr_cf,                      0,          1, 1 },
    { "snr_diff",                  VAR_UINT8,  &cfg.snr_diff,                    0,        200, 1 },
    { "snr_land",                  VAR_UINT8,  &cfg.snr_land,                    0,          1, 1 },    
    { "LED_Type",                  VAR_UINT8,  &cfg.LED_Type,                    0,          3, 0 },
    { "LED_pinout",                VAR_UINT8,  &cfg.LED_Pinout,                  0,          1, 0 },
    { "LED_ControlChannel",        VAR_UINT8,  &cfg.LED_ControlChannel,          1,         12, 0 }, // Aux Channel to controll the LED Pattern
    { "LED_ARMED",                 VAR_UINT8,  &cfg.LED_Armed,                   0,          1, 1 }, // 0 = Show LED only if armed, 1 = always show LED
    { "LED_Toggle_Delay1",         VAR_UINT8, &cfg.LED_Toggle_Delay1,            0,        255, 0 },
    { "LED_Toggle_Delay2",         VAR_UINT8, &cfg.LED_Toggle_Delay2,            0,        255, 0 },
    { "LED_Toggle_Delay3",         VAR_UINT8, &cfg.LED_Toggle_Delay3,            0,        255, 0 },
    { "LED_Pattern1",              VAR_UINT32, &cfg.LED_Pattern1,                0, 0x7FFFFFFF, 0 }, // Pattern for Switch position 1
    { "LED_Pattern2",              VAR_UINT32, &cfg.LED_Pattern2,                0, 0x7FFFFFFF, 0 }, // Pattern for Switch position 2
    { "LED_Pattern3",              VAR_UINT32, &cfg.LED_Pattern3,                0, 0x7FFFFFFF, 0 }, // Pattern for Switch position 3};
};

#define VALUE_COUNT (sizeof(valueTable) / sizeof(valueTable[0]))

static void cliSetVar(const clivalue_t *var, const int32_t value);
static void changeval(const clivalue_t *var, const int8_t adder);
static void cliPrintVar(const clivalue_t *var, uint32_t full);

#ifndef HAVE_ITOA_FUNCTION

/*
** The following two functions together make up an itoa()
** implementation. Function i2a() is a 'private' function
** called by the public itoa() function.
**
** itoa() takes three arguments:
**        1) the integer to be converted,
**        2) a pointer to a character conversion buffer,
**        3) the radix for the conversion
**           which can range between 2 and 36 inclusive
**           range errors on the radix default it to base10
** Code from http://groups.google.com/group/comp.lang.c/msg/66552ef8b04fe1ab?pli=1
*/

static char *i2a(unsigned i, char *a, unsigned r)
{
    if (i / r > 0)
        a = i2a(i / r, a, r);
    *a = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ"[i % r];
    return a + 1;
}

char *itoa(int i, char *a, int r)
{
    if ((r < 2) || (r > 36))
        r = 10;
    if (i < 0)
    {
        *a = '-';
        *i2a(-(unsigned)i, a + 1, r) = 0;
    }
    else
        *i2a(i, a, r) = 0;
    return a;
}

#endif

////////////////////////////////////////////////////////////////////////////////
// String to Float Conversion
///////////////////////////////////////////////////////////////////////////////
// Simple and fast atof (ascii to float) function.
//
// - Executes about 5x faster than standard MSCRT library atof().
// - An attractive alternative if the number of calls is in the millions.
// - Assumes input is a proper integer, fraction, or scientific format.
// - Matches library atof() to 15 digits (except at extreme exponents).
// - Follows atof() precedent of essentially no error checking.
//
// 09-May-2009 Tom Van Baak (tvb) www.LeapSecond.com
//
#define white_space(c) ((c) == ' ' || (c) == '\t')
#define valid_digit(c) ((c) >= '0' && (c) <= '9')
static float _atof(const char *p)
{
    int frac = 0;
    double sign, value, scale;

    // Skip leading white space, if any.
    while (white_space(*p) )
    {
        p += 1;
    }

    // Get sign, if any.
    sign = 1.0;
    if (*p == '-')
    {
        sign = -1.0;
        p += 1;

    }
    else if (*p == '+')
    {
        p += 1;
    }

    // Get digits before decimal point or exponent, if any.
    value = 0.0;
    while (valid_digit(*p))
    {
        value = value * 10.0 + (*p - '0');
        p += 1;
    }

    // Get digits after decimal point, if any.
    if (*p == '.')
    {
        double pow10 = 10.0;
        p += 1;

        while (valid_digit(*p))
        {
            value += (*p - '0') / pow10;
            pow10 *= 10.0;
            p += 1;
        }
    }

    // Handle exponent, if any.
    scale = 1.0;
    if ((*p == 'e') || (*p == 'E'))
    {
        unsigned int expon;
        p += 1;

        // Get sign of exponent, if any.
        frac = 0;
        if (*p == '-')
        {
            frac = 1;
            p += 1;

        }
        else if (*p == '+')
        {
            p += 1;
        }

        // Get digits of exponent, if any.
        expon = 0;
        while (valid_digit(*p))
        {
            expon = expon * 10 + (*p - '0');
            p += 1;
        }
        if (expon > 308) expon = 308;

        // Calculate scaling factor.
        while (expon >= 50)
        {
            scale *= 1E50;
            expon -= 50;
        }
        while (expon >=  8)
        {
            scale *= 1E8;
            expon -=  8;
        }
        while (expon >   0)
        {
            scale *= 10.0;
            expon -=  1;
        }
    }

    // Return signed and scaled floating point result.
    return sign * (frac ? (value / scale) : (value * scale));
}

///////////////////////////////////////////////////////////////////////////////
// FTOA
///////////////////////////////////////////////////////////////////////////////
static char *ftoa(float x, char *floatString)
{
    int32_t value;
    char intString1[12];
    char intString2[12] = { 0, };
    char *decimalPoint = ".";
    uint8_t dpLocation;

    if (x > 0)                  // Rounding for x.xxx display format
        x += 0.0005f;
    else
        x -= 0.0005f;

    value = (int32_t) (x * 1000.0f);    // Convert float * 1000 to an integer

    itoa(abs(value), intString1, 10);   // Create string from abs of integer value

    if (value >= 0)
        intString2[0] = ' ';    // Positive number, add a pad space
    else
        intString2[0] = '-';    // Negative number, add a negative sign

    if (strlen(intString1) == 1)
    {
        intString2[1] = '0';
        intString2[2] = '0';
        intString2[3] = '0';
        strcat(intString2, intString1);
    }
    else if (strlen(intString1) == 2)
    {
        intString2[1] = '0';
        intString2[2] = '0';
        strcat(intString2, intString1);
    }
    else if (strlen(intString1) == 3)
    {
        intString2[1] = '0';
        strcat(intString2, intString1);
    }
    else
    {
        strcat(intString2, intString1);
    }

    dpLocation = strlen(intString2) - 3;

    strncpy(floatString, intString2, dpLocation);
    floatString[dpLocation] = '\0';
    strcat(floatString, decimalPoint);
    strcat(floatString, intString2 + dpLocation);

    return floatString;
}

static void cliPrompt(void)
{
    uartPrint("\r\n# ");
}

static int cliCompare(const void *a, const void *b)
{
    const clicmd_t *ca = a, *cb = b;
    return strncasecmp(ca->name, cb->name, strlen(cb->name));
}

static void cliAux(char *cmdline)
{
    int i, val = 0;
    uint8_t len;
    char *ptr;

    len = strlen(cmdline);
    if (len == 0)
    {
        // print out aux channel settings
        for (i = 0; i < CHECKBOXITEMS; i++)
            printf("aux %u %u\r\n", i, cfg.activate[i]);
    }
    else
    {
        ptr = cmdline;
        i = atoi(ptr);
        if (i < CHECKBOXITEMS)
        {
            ptr = strchr(cmdline, ' ');
            val = atoi(ptr);
            cfg.activate[i] = val;
        }
        else
        {
            printf("Invalid Feature index: must be < %u\r\n", CHECKBOXITEMS);
        }
    }
}

static void cliCMix(char *cmdline)
{
    int i, check = 0;
    int num_motors = 0;
    uint8_t len;
    char buf[16];
    float mixsum[3];
    char *ptr;

    len = strlen(cmdline);

    if (len == 0)
    {
        uartPrint("Custom mixer: \r\nMotor\tThr\tRoll\tPitch\tYaw\r\n");
        for (i = 0; i < MAX_MOTORS; i++)
        {
            if (cfg.customMixer[i].throttle == 0.0f)
                break;
//  Ugly error here found by meister:          mixsum[i] = 0.0f;
            num_motors++;
            printf("#%d:\t", i + 1);
            printf("%s\t", ftoa(cfg.customMixer[i].throttle, buf));
            printf("%s\t", ftoa(cfg.customMixer[i].roll, buf));
            printf("%s\t", ftoa(cfg.customMixer[i].pitch, buf));
            printf("%s\r\n", ftoa(cfg.customMixer[i].yaw, buf));
        }
        for (i = 0; i < 3; i++)                                           // Fix by meister
            mixsum[i] = 0.0f;
        for (i = 0; i < num_motors; i++)
        {
            mixsum[0] += cfg.customMixer[i].roll;
            mixsum[1] += cfg.customMixer[i].pitch;
            mixsum[2] += cfg.customMixer[i].yaw;
        }
        uartPrint("Sanity check:\t");
        for (i = 0; i < 3; i++)
            uartPrint(fabs(mixsum[i]) > 0.01f ? "NG\t" : "OK\t");
        uartPrint("\r\n");
        return;
    }
    else if (strncasecmp(cmdline, "load", 4) == 0)
    {
        ptr = strchr(cmdline, ' ');
        if (ptr)
        {
            len = strlen(++ptr);
            for (i = 0; ; i++)
            {
                if (mixerNames[i] == NULL)
                {
                    uartPrint("Invalid mixer type...\r\n");
                    break;
                }
                if (strncasecmp(ptr, mixerNames[i], len) == 0)
                {
                    mixerLoadMix(i);
                    printf("Loaded %s mix...\r\n", mixerNames[i]);
                    cliCMix("");
                    break;
                }
            }
        }
    }
    else
    {
        ptr = cmdline;
        i = atoi(ptr); // get motor number
        if (--i < MAX_MOTORS)
        {
            ptr = strchr(ptr, ' ');
            if (ptr)
            {
                cfg.customMixer[i].throttle = _atof(++ptr);
                check++;
            }
            ptr = strchr(ptr, ' ');
            if (ptr)
            {
                cfg.customMixer[i].roll = _atof(++ptr);
                check++;
            }
            ptr = strchr(ptr, ' ');
            if (ptr)
            {
                cfg.customMixer[i].pitch = _atof(++ptr);
                check++;
            }
            ptr = strchr(ptr, ' ');
            if (ptr)
            {
                cfg.customMixer[i].yaw = _atof(++ptr);
                check++;
            }
            if (check != 4)
            {
                uartPrint("Wrong number of arguments, needs idx thr roll pitch yaw\r\n");
            }
            else
            {
                cliCMix("");
            }
        }
        else
        {
            printf("Motor number must be between 1 and %d\r\n", MAX_MOTORS);
        }
    }
}

static void cliDefaults(char *cmdline)
{
    uartPrint("Resetting to defaults...\r\n");
    checkFirstTime(true);
    uartPrint("Rebooting...");
    delay(10);
    systemReset(false);
}

static void cliDump(char *cmdline)
{
    int i;                                         //, val = 0;
    char buf[16];
    float thr, roll, pitch, yaw;
    uint32_t mask;
    const clivalue_t *setval;

    printf(";Current Config: Copy everything below here...\r\n");
    printf(";Firmware: %s\r\n", FIRMWARE);

    // print out aux switches
    cliAux("");

    // print out current motor mix
    printf("mixer %s\r\n", mixerNames[cfg.mixerConfiguration - 1]);

    // print custom mix if exists
    if (cfg.customMixer[0].throttle != 0.0f)
    {
        for (i = 0; i < MAX_MOTORS; i++)
        {
            if (cfg.customMixer[i].throttle == 0.0f) break;
            thr = cfg.customMixer[i].throttle;
            roll = cfg.customMixer[i].roll;
            pitch = cfg.customMixer[i].pitch;
            yaw = cfg.customMixer[i].yaw;
            printf("cmix %d", i + 1);
            if (thr < 0) printf(" ");
            printf("%s", ftoa(thr, buf));
            if (roll < 0) printf(" ");
            printf("%s", ftoa(roll, buf));
            if (pitch < 0) printf(" ");
            printf("%s", ftoa(pitch, buf));
            if (yaw < 0) printf(" ");
            printf("%s\r\n", ftoa(yaw, buf));
        }
        printf("cmix %d 0 0 0 0\r\n", i + 1);
    }
    mask = featureMask();                                // print enabled features
    for (i = 0; ; i++)                                   // disable all feature first
    {
        if (featureNames[i] == NULL) break;
        printf("feature -%s\r\n", featureNames[i]);
    }
    for (i = 0; ; i++)                                   // reenable what we want.
    {
        if (featureNames[i] == NULL) break;
        if (mask & (1 << i)) printf("feature %s\r\n", featureNames[i]);
    }
    for (i = 0; i < 8; i++) buf[cfg.rcmap[i]] = rcChannelLetters[i]; // print RC MAPPING
    buf[i] = '\0';
    printf("map %s\r\n", buf);
    for (i = 0; i < VALUE_COUNT; i++)                    // print settings
    {
        setval = &valueTable[i];
        printf("set %s = ", valueTable[i].name);
        cliPrintVar(setval, 0);
        uartPrint("\r\n");
    }
}

static void cliExit(char *cmdline)
{
    uartPrint("\r\nLeaving CLI mode without saving\r\n");
    memset(cliBuffer, 0, sizeof(cliBuffer));
    bufferIndex = 0;
    cliMode = 0;
    uartPrint("\r\nRebooting...");
    delay(10);
    systemReset(false);                                 // Just Reset without saving makes more sense
//    cliSave(cmdline);                                   // save and reboot... I think this makes the most sense
}

static void cliFeature(char *cmdline)
{
    uint32_t i;
    uint32_t len;
    uint32_t mask;

    len = strlen(cmdline);
    mask = featureMask();

    if (len == 0)
    {
        uartPrint("Enabled features: ");
        for (i = 0; ; i++)
        {
            if (featureNames[i] == NULL) break;
            if (mask & (1 << i)) printf("%s ", featureNames[i]);
        }
        uartPrint("\r\n");
    }
    else if (strncasecmp(cmdline, "list", len) == 0)
    {
        uartPrint("Available features: \r\n");  // uartPrint("Available features: ");
        for (i = 0; ; i++)
        {
            if (featureNames[i] == NULL) break;
            printf("%s \r\n", featureNames[i]); // printf("%s ", featureNames[i]);
        }
        uartPrint("\r\n");
        return;
    }
    else
    {
        bool remove = false;
        if (cmdline[0] == '-')
        {
            remove = true;            // remove feature
            cmdline++;                // skip over -
            len--;
        }

        for (i = 0; ; i++)
        {
            if (featureNames[i] == NULL)
            {
                uartPrint("Invalid feature name...\r\n");
                break;
            }
            if (strncasecmp(cmdline, featureNames[i], len) == 0)
            {
                if (remove)
                {
                    featureClear(1 << i);
                    uartPrint("Disabled ");
                }
                else
                {
                    featureSet(1 << i);
                    uartPrint("Enabled ");
                }
                printf("%s\r\n", featureNames[i]);
                break;
            }
        }
    }
}

static void cliHelp(char *cmdline)
{
    uint32_t i = 0;
    uartPrint("Available commands:\r\n");
    for (i = 0; i < CMD_COUNT; i++) printf("%s\t%s\r\n", cmdTable[i].name, cmdTable[i].param);
}

static void cliMap(char *cmdline)
{
    uint32_t len;
    uint32_t i;
    char out[9];
    len = strlen(cmdline);
    if (len == 8)
    {
        // uppercase it
        for (i = 0; i < 8; i++) cmdline[i] = toupper(cmdline[i]);
        for (i = 0; i < 8; i++)
        {
            if (strchr(rcChannelLetters, cmdline[i]) && !strchr(cmdline + i + 1, cmdline[i]))
                continue;
            uartPrint("Must be any order of AETR1234\r\n");
            return;
        }
        parseRcChannels(cmdline);
    }
    uartPrint("Current assignment: ");
    for (i = 0; i < 8; i++) out[cfg.rcmap[i]] = rcChannelLetters[i];
    out[i] = '\0';
    printf("%s\r\n", out);
}

static void cliMixer(char *cmdline)
{
    uint8_t i;
    uint8_t len;

    len = strlen(cmdline);

    if (len == 0)
    {
        printf("Current mixer: %s\r\n", mixerNames[cfg.mixerConfiguration - 1]);
        return;
    }
    else if (strncasecmp(cmdline, "list", len) == 0)
    {
        uartPrint("Available mixers: ");
        for (i = 0; ; i++)
        {
            if (mixerNames[i] == NULL) break;
            printf("%s ", mixerNames[i]);
        }
        uartPrint("\r\n");
        return;
    }

    for (i = 0; ; i++)
    {
        if (mixerNames[i] == NULL)
        {
            uartPrint("Invalid mixer type...\r\n");
            break;
        }
        if (strncasecmp(cmdline, mixerNames[i], len) == 0)
        {
            cfg.mixerConfiguration = i + 1;
            printf("Mixer set to %s\r\n", mixerNames[i]);
            break;
        }
    }
}

static void cliSave(char *cmdline)
{
    uartPrint("Saving...");
    writeParams(0);
    uartPrint("\r\nRebooting...");
    delay(10);
    cliMode = 0;
    systemReset(false);
}

static void cliPrintVar(const clivalue_t *var, uint32_t full)
{
    int32_t value = 0;
    char buf[8];

    switch (var->type)
    {
    case VAR_UINT8:
        value = *(uint8_t *)var->ptr;
        break;

    case VAR_INT8:
        value = *(int8_t *)var->ptr;
        break;

    case VAR_UINT16:
        value = *(uint16_t *)var->ptr;
        break;

    case VAR_INT16:
        value = *(int16_t *)var->ptr;
        break;

    case VAR_UINT32:
        value = *(uint32_t *)var->ptr;
        break;

    case VAR_FLOAT:
        printf("%s", ftoa(*(float *)var->ptr, buf));
        if (full)
        {
            printf(" %s", ftoa((float)var->min, buf));
            printf(" %s", ftoa((float)var->max, buf));
        }
        return; // return from case for float only
    }
    printf("%d", value);
    if (full)
        printf(" %d %d", var->min, var->max);
}

static void cliSetVar(const clivalue_t *var, const int32_t value)
{
    switch (var->type)
    {
    case VAR_UINT8:
    case VAR_INT8:
        *(char *)var->ptr = (char)value;
        break;

    case VAR_UINT16:
    case VAR_INT16:
        *(short *)var->ptr = (short)value;
        break;

    case VAR_UINT32:
        *(int *)var->ptr = (int)value;
        break;

    case VAR_FLOAT:
        *(float *)var->ptr = *(float *)&value;
        break;
    }
}

static void cliSet(char *cmdline)
{
    uint32_t i;
    uint32_t len;
    const clivalue_t *val;
    char *eqptr = NULL;
    int32_t value = 0;
    float valuef = 0;

    len = strlen(cmdline);

    if (len == 0 || (len == 1 && cmdline[0] == '*'))
    {
        uartPrint("Current settings: \r\n");
        for (i = 0; i < VALUE_COUNT; i++)
        {
            val = &valueTable[i];
            printf("%s = ", valueTable[i].name);
            cliPrintVar(val, len); // when len is 1 (when * is passed as argument), it will print min/max values as well, for gui
            uartPrint("\r\n");
        }
    }
    else if ((eqptr = strstr(cmdline, "=")))
    {
        // has equal, set var
        eqptr++;
        len--;
        value = atoi(eqptr);
        valuef = _atof(eqptr);
        for (i = 0; i < VALUE_COUNT; i++)
        {
            val = &valueTable[i];
            if (strncasecmp(cmdline, valueTable[i].name, strlen(valueTable[i].name)) == 0)
            {
                if (valuef >= valueTable[i].min && valuef <= valueTable[i].max)   // here we compare the float value since... it should work, RIGHT?
                {
                    cliSetVar(val, valueTable[i].type == VAR_FLOAT ? *(uint32_t *)&valuef : value); // this is a silly dirty hack. please fix me later.
                    printf("%s set to ", valueTable[i].name);
                    cliPrintVar(val, 0);
                }
                else
                {
                    uartPrint("ERR: Value assignment out of range\r\n");
                }
                return;
            }
        }
        uartPrint("ERR: Unknown variable name\r\n");
    }
}

static void cliStatus(char *cmdline)
{
    uint8_t i;
    uint32_t mask;
    char dummy;
    uartPrint("\r\n");
    printf("System Uptime: %d seconds, Voltage: %d * 0.1V (%dS battery)\r\n",
           millis() / 1000, vbat, batteryCellCount);
    uartPrint("\r\n");  
    mask = sensorsMask();

    printf("CPU %dMHz, detected sensors: ", (SystemCoreClock / 1000000));
    for (i = 0; ; i++)
    {
        if (sensorNames[i] == NULL)
            break;
        if (mask & (1 << i))
            printf("%s ", sensorNames[i]);
    }
    if (sensors(SENSOR_ACC))
        printf("ACCHW: %s", accNames[accHardware]);
    uartPrint("\r\n");
    uartPrint("\r\n");
    printf("Cycle Time: %d, I2C Errors: %d\r\n", cycleTime, i2cGetErrorCounter());
    uartPrint("\r\n");
    printf("Config Size: %d Bytes\r\n",cfg.size);
    uartPrint("\r\n");
    cliScanI2Cbus(&dummy);
}

static void cliVersion(char *cmdline)
{
    uartPrint(FIRMWARE);
}

void serialOSD(void)
{
#define LCDdelay 12
    static uint32_t rctimer;
    static uint8_t exit,input,lastinput,brake,brakeval,speeduptimer;
    static uint16_t DatasetNr;
    const clivalue_t *setval;
    uint32_t timetmp;

    if (cliMode != 0) return;                                           // Don't do this if we are in cli mode
    LCDinit();
    printf(FIRMWAREFORLCD);                                             // Defined in mw.h
    LCDline2();
    printf("LCD Interface");
    delay(2000);
    LCDclear();
    exit      = 0;
    DatasetNr = 0;
    brake     = 0;
    brakeval  = 0;
    speeduptimer = 0;
    LCDline1();
    printf("%s", valueTable[DatasetNr].name);                           // Display first item anyway (even if lcd ==0)
    LCDline2();
    setval = &valueTable[DatasetNr];
    cliPrintVar(setval, 0);
    while (exit == 0)
    {
        timetmp = micros();
        if (spektrumFrameComplete()) computeRC();                       // Generates no rcData yet, but rcDataSAVE
        if ((int32_t)(timetmp - rctimer) >= 0)                          // Start of 50Hz Loop
        {
            rctimer = timetmp + 20000;
            LED1_TOGGLE;
            LED0_TOGGLE;
            if (!feature(FEATURE_SPEKTRUM)) computeRC();
            GetActualRCdataOutRCDataSave();                             // Now we have new rcData to deal and MESS with
            if (rcData[THROTTLE] < cfg.mincheck && rcData[YAW] > cfg.maxcheck && rcData[PITCH] > cfg.maxcheck) exit = 1; // Quit don't save
            if (rcData[THROTTLE] < cfg.mincheck && rcData[YAW] < cfg.mincheck && rcData[PITCH] > cfg.maxcheck) exit = 2; // Quit and save
            input = 0;
            if (exit == 0 && input == 0 && rcData[PITCH] < cfg.mincheck) input = 1;
            if (exit == 0 && input == 0 && rcData[PITCH] > cfg.maxcheck) input = 2;
            if (exit == 0 && input == 0 && rcData[ROLL]  < cfg.mincheck) input = 3;
            if (exit == 0 && input == 0 && rcData[ROLL]  > cfg.maxcheck) input = 4;

            if (lastinput == input)                                     // Adjust Inputspeed
            {
                speeduptimer++;
                if (speeduptimer >= 100)
                {
                    speeduptimer = 99;
                    brakeval = 8;
                }
                else brakeval = 17;
            }
            else
            {
                brakeval = 0;
                speeduptimer = 0;
            }
            lastinput = input;
            brake++;
            if (brake >= brakeval) brake = 0;
            else input = 0;

            switch (input)
            {
            case 0:
                break;
            case 1:                                                 // Down
                do                                                  // Search for next Dataset
                {
                    DatasetNr++;
                    if (DatasetNr == VALUE_COUNT) DatasetNr = 0;
                }
                while (valueTable[DatasetNr].lcd == 0);
                LCDclear();
                printf("%s", valueTable[DatasetNr].name);
                LCDline2();
                setval = &valueTable[DatasetNr];
                cliPrintVar(setval, 0);
                break;
            case 2:                                                 // UP
                do                                                  // Search for next Dataset
                {
                    if (DatasetNr == 0) DatasetNr = VALUE_COUNT;
                    DatasetNr--;
                }
                while (valueTable[DatasetNr].lcd == 0);
                LCDclear();
                printf("%s", valueTable[DatasetNr].name);
                LCDline2();
                setval = &valueTable[DatasetNr];
                cliPrintVar(setval, 0);
                break;
            case 3:                                                 // LEFT
                if (brakeval != 8) changeval(setval,-1);            // Substract within the limit
                else changeval(setval,-5);
                LCDline2();
                cliPrintVar(setval, 0);
                break;
            case 4:                                                 // RIGHT
                if (brakeval != 8) changeval(setval,1);             // Add within the limit
                else changeval(setval,5);
                LCDline2();
                cliPrintVar(setval, 0);
                break;
            }
        }                                                               // End of 50Hz Loop
    }
    delay(500);
    LCDclear();
    printf(" Exit & Reboot ");
    LCDline2();
    switch (exit)
    {
    case 1:
        printf(" NOT Saving");
        delay(1000);
        LCDoff();
        systemReset(false);
        break;
    case 2:
        printf(".!.!.Saving.!.!.");
        delay(1000);
        writeParams(0);
        LCDoff();
        systemReset(false);
        break;
    }
}

static void changeval(const clivalue_t *var, const int8_t adder)
{
    int32_t value;
    float   valuef;
    int32_t maximum;
    int32_t minimum;

    maximum = var->max;
    minimum = var->min;
    switch (var->type)
    {
    case VAR_UINT8:
    case VAR_INT8:
        value = *(char *)var->ptr;
        value = constrain(value + adder,minimum,maximum);
        *(char *)var->ptr = (char)value;
        break;

    case VAR_UINT16:
    case VAR_INT16:
        value = *(short *)var->ptr;
        value = constrain(value + adder,minimum,maximum);
        *(short *)var->ptr = (short)value;
        break;

    case VAR_UINT32:
        value = *(int *)var->ptr;
        value = constrain(value + adder,minimum,maximum);
        *(int *)var->ptr = (int)value;
        break;

    case VAR_FLOAT:
        *(float *)&valuef = *(float *)var->ptr;
        valuef = constrain(valuef + (float)adder/1000.0f,minimum,maximum);
        *(float *)var->ptr = *(float *)&valuef;
        break;
    }
}

void LCDinit(void)
{
    serialInit(9600);                                                   // INIT LCD HERE
    LCDoff();
    uartWrite(0xFE);
    delay(LCDdelay);
    uartWrite(0x0C);                                                    // Display ON
    delay(LCDdelay);
    uartWrite(0x7C);
    delay(LCDdelay);
    uartWrite(0x9D);                                                    // 100% Brightness
    LCDclear();
}

void LCDoff(void)
{
    delay(LCDdelay);
    uartWrite(0xFE);
    delay(LCDdelay);
    uartWrite(0x08);                                                    // LCD Display OFF
    delay(LCDdelay);
}

void LCDclear(void)                                                     // clear screen, cursor line 1, pos 0
{
    delay(LCDdelay);
    uartWrite(0xFE);
    delay(LCDdelay);
    uartWrite(0x01);                                                    // Clear
    LCDline1();
}

void LCDline1(void)                                                     // Sets LCD Cursor to line 1 pos 0
{
    delay(LCDdelay);
    uartWrite(0xFE);
    delay(LCDdelay);
    uartWrite(0x80);                                                    // Line #1 pos 0
    delay(LCDdelay);
}

void LCDline2(void)                                                     // Sets LCD Cursor to line 2 pos 0
{
    delay(LCDdelay);
    uartWrite(0xFE);
    delay(LCDdelay);
    uartWrite(0xC0);  	                                                // Line #2
    delay(LCDdelay);
    printf("               ");                                          // Clear Line #2
    delay(LCDdelay);
    uartWrite(0xFE);
    delay(LCDdelay);
    uartWrite(0xC0);  	                                                // Line #2
    delay(LCDdelay);
}

void cliProcess(void)
{
    if (!cliMode)
    {
        cliMode = 1;
        uartPrint("\r\nEntering CLI Mode, type 'exit' or 'save' to return, or 'help' \r\n");
        cliPrompt();
    }

    while (uartAvailable())
    {
        uint8_t c = uartRead();
        if (c == '\t' || c == '?')
        {
            // do tab completion
            const clicmd_t *cmd, *pstart = NULL, *pend = NULL;
            int i = bufferIndex;
            for (cmd = cmdTable; cmd < cmdTable + CMD_COUNT; cmd++)
            {
                if (bufferIndex && (strncasecmp(cliBuffer, cmd->name, bufferIndex) != 0))
                    continue;
                if (!pstart)
                    pstart = cmd;
                pend = cmd;
            }
            if (pstart)      /* Buffer matches one or more commands */
            {
                for (; ; bufferIndex++)
                {
                    if (pstart->name[bufferIndex] != pend->name[bufferIndex])
                        break;
                    if (!pstart->name[bufferIndex])
                    {
                        /* Unambiguous -- append a space */
                        cliBuffer[bufferIndex++] = ' ';
                        break;
                    }
                    cliBuffer[bufferIndex] = pstart->name[bufferIndex];
                }
            }
            if (!bufferIndex || pstart != pend)
            {
                /* Print list of ambiguous matches */
                uartPrint("\r\033[K");
                for (cmd = pstart; cmd <= pend; cmd++)
                {
                    uartPrint(cmd->name);
                    uartWrite('\t');
                }
                cliPrompt();
                i = 0;    /* Redraw prompt */
            }
            for (; i < bufferIndex; i++)
                uartWrite(cliBuffer[i]);
        }
        else if (!bufferIndex && c == 4)
        {
            cliExit(cliBuffer);
            return;
        }
        else if (c == 12)
        {
            // clear screen
            uartPrint("\033[2J\033[1;1H");
            cliPrompt();
        }
        else if (bufferIndex && (c == '\n' || c == '\r'))
        {
            // enter pressed
            clicmd_t *cmd = NULL;
            clicmd_t target;
            uartPrint("\r\n");
            cliBuffer[bufferIndex] = 0; // null terminate

            target.name = cliBuffer;
            target.param = NULL;

            cmd = bsearch(&target, cmdTable, CMD_COUNT, sizeof cmdTable[0], cliCompare);
            if (cmd)
                cmd->func(cliBuffer + strlen(cmd->name) + 1);
            else
                uartPrint("ERR: That command was Harakiri, try 'help'");

            memset(cliBuffer, 0, sizeof(cliBuffer));
            bufferIndex = 0;

            // 'exit' will reset this flag, so we don't need to print prompt again
            if (!cliMode)
                return;
            cliPrompt();
        }
        else if (c == 127)
        {
            // backspace
            if (bufferIndex)
            {
                cliBuffer[--bufferIndex] = 0;
                uartPrint("\010 \010");
            }
        }
        else if (bufferIndex < sizeof(cliBuffer) && c >= 32 && c <= 126)
        {
            if (!bufferIndex && c == 32)
                continue;
            cliBuffer[bufferIndex++] = c;
            uartWrite(c);
        }
    }
}

// ************************************************************************************************************
// TestScan on the I2C bus
// ************************************************************************************************************
#define MMA8452_ADDRESS       0x1C
#define HMC5883L_ADDRESS      0x1E  // 0xA
#define DaddyW_SONAR          0x20  // Daddy Walross Sonar
#define OLED_address          0x3C  // OLED at address 0x3C in 7bit
#define ADXL345_ADDRESS       0x53
#define BMA180_adress         0x64  // don't respond ??
#define MPU6050_ADDRESS       0x68  // 0x75     or 0x68  0x15
#define L3G4200D_ADDRESS      0x68  // 0x0f
#define BMP085_I2C_ADDR       0x77  // 0xD0
#define MS5611_ADDR           0x77  // 0xA0

/*
new May 15 2013 Johannes && Some stuff from me as well :)
*/
static void cliScanI2Cbus(char *cmdline)
{
    bool    ack;
    bool    msbaro   = false;
    bool    L3G4200D = false;
    uint8_t address;
    uint8_t nDevices;
    uint8_t sig = 0;
  	uint8_t bufdaddy[2];                                // Dummy for DaddyW testread
    char    buf[20];

    printf("Scanning I2C-Bus for devices now.\r\n");
    printf("=================================\r\n");

    nDevices = 0;
    for(address = 1; address < 127; address++ )
    {
        ack = i2cRead(address, address, 1, &sig);       // Do a blind read. Perhaps it's sufficient? Otherwise the hard way...
      
        if (!ack && address == MMA8452_ADDRESS)         // Do it the hard way, if no ACK on mma Adress
        {
            sig = 0;
            i2cRead(MMA8452_ADDRESS, 0x0D, 1, &sig);
            if (sig == 0x2A || sig == 0x1A)
                ack = true;
            else
                ack = false;
        }

        if (!ack && address == DaddyW_SONAR)            // Do it the hard way, if no ACK on DaddyW Adress
        {
            ack = i2cRead(DaddyW_SONAR, 0x32, 2, bufdaddy);
        }
        
        if (!ack && address == MS5611_ADDR)             // MS Baro needs special treatment BMP would have said "ack" already
        {
            ack = i2cRead(MS5611_ADDR, 0xA0, 1, &sig);  // Sig is irrelevant?
            msbaro = ack;
        }

        if (!ack && address == MPU6050_ADDRESS)         // Special case mpu and L3G4200D have same Adr.
        {
            sig = 0;
            i2cRead(0x68, 0x0F, 1, &sig);
            if (sig == 0xD3)
            {
                ack = true;              
                L3G4200D = true;
            }
        }

        if (ack)
        {
            printf("I2C device found at 0x");
            if (address<16) printf("0");
            printf("%x",address);
            switch (address)
            {
            case MMA8452_ADDRESS:                       // Detection altered
                strcpy(buf,"MMA8452");
                break;
            case HMC5883L_ADDRESS:
                strcpy(buf,"HMC5883L");
                break;
            case DaddyW_SONAR:                          // Daddy Walross Sonar added
                strcpy(buf,"DaddyW Sonar");							
                break;						
            case OLED_address:
                strcpy(buf,"OLED");                     // i2c_OLED_init();
                break;
            case ADXL345_ADDRESS:                       // ADXL added
                strcpy(buf,"ADXL345");
                break;						
            case BMA180_adress:                         // Sensor currently not supported by a driver
                strcpy(buf,"BMA180");
                break;
            case MPU6050_ADDRESS:
                if (L3G4200D) strcpy(buf,"L3G4200D");
                else strcpy(buf,"MPU3050/MPU6050");
                break;
            case BMP085_I2C_ADDR:
                if(msbaro) strcpy(buf,"MS5611");
                else strcpy(buf,"BMP085");
                break;
            default:                                    // Unknown case added
                strcpy(buf,"UNKNOWN TO ME");
                break;
            }
            printf(" Probably it's %s \r\n",buf);
            nDevices++;
        }
        delay(50);
    }
    uartPrint("\r\n");
    if (nDevices == 0) printf("No I2C devices found\r\n");
    else printf("Done, %d Devices found\r\n",nDevices);
}
