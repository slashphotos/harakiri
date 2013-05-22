#pragma once

/* for VBAT monitoring frequency */
#define VBATFREQ 6        // to read battery voltage - nth number of loop iterations

#define  VERSION  211
#define  FIRMWARE  "Naze32 cGiesen/Crashpilot Harakiri10Beta D " __DATE__ " / " __TIME__

#define LAT  0
#define LON  1
#define GPS_Y 0
#define GPS_X 1

// Serial GPS only variables
// navigation mode
typedef enum NavigationMode
{
    NAV_MODE_NONE = 0,
    NAV_MODE_POSHOLD,
	  NAV_MODE_RTL,
    NAV_MODE_WP,
	  NAV_MODE_CIRCLE
} NavigationMode;

typedef enum WPstatus
{
    WP_STATUS_NONE = 0,
	  WP_STATUS_NAVIGATING,
    WP_STATUS_DONE
} WPstatus;


// Syncronized with GUI. Only exception is mixer > 11, which is always returned as 11 during serialization.
typedef enum MultiType
{
    MULTITYPE_TRI = 1,
    MULTITYPE_QUADP = 2,
    MULTITYPE_QUADX = 3,
    MULTITYPE_BI = 4,
    MULTITYPE_GIMBAL = 5,
    MULTITYPE_Y6 = 6,
    MULTITYPE_HEX6 = 7,
    MULTITYPE_FLYING_WING = 8,
    MULTITYPE_Y4 = 9,
    MULTITYPE_HEX6X = 10,
    MULTITYPE_OCTOX8 = 11,          // Java GUI is same for the next 3 configs
    MULTITYPE_OCTOFLATP = 12,       // MultiWinGui shows this differently
    MULTITYPE_OCTOFLATX = 13,       // MultiWinGui shows this differently
    MULTITYPE_AIRPLANE = 14,        // airplane / singlecopter / dualcopter (not yet properly supported)
    MULTITYPE_HELI_120_CCPM = 15,
    MULTITYPE_HELI_90_DEG = 16,
    MULTITYPE_VTAIL4 = 17,
    MULTITYPE_CUSTOM = 18,          // no current GUI displays this
    MULTITYPE_LAST = 19
} MultiType;

typedef enum GimbalFlags {
    GIMBAL_NORMAL = 1 << 0,
    GIMBAL_TILTONLY = 1 << 1,
    GIMBAL_DISABLEAUX34 = 1 << 2,
    GIMBAL_FORWARDAUX = 1 << 3,
    GIMBAL_MIXTILT = 1 << 4,
} GimbalFlags;

/*********** RC alias *****************/
enum {
    ROLL = 0,
    PITCH,
    YAW,
    THROTTLE,
    AUX1,
    AUX2,
    AUX3,
    AUX4
};

enum {
    PIDROLL,
    PIDPITCH,
    PIDYAW,
    PIDALT,
    PIDPOS,
    PIDPOSR,
    PIDNAVR,
    PIDLEVEL,
    PIDMAG,
    PIDVEL,
    PIDITEMS
};

enum {
    BOXANGLE = 0,
    BOXHORIZON,
    BOXBARO,
    BOXMAG,
    BOXCAMSTAB,
    BOXCAMTRIG,
    BOXARM,
    BOXGPSHOME,
    BOXGPSHOLD,
    BOXPASSTHRU,
    BOXHEADFREE,
    BOXBEEPERON,
    BOXLEDMAX,
    BOXLLIGHTS,
    BOXHEADADJ,
    CHECKBOXITEMS
};

#define min(a, b) ((a) < (b) ? (a) : (b))
#define max(a, b) ((a) > (b) ? (a) : (b))
#define abs(x) ((x) > 0 ? (x) : -(x))
#define constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))

typedef struct motorMixer_t {
    float throttle;
    float roll;
    float pitch;
    float yaw;
} motorMixer_t;

typedef struct mixer_t {
    uint8_t numberMotor;
    uint8_t useServo;
    const motorMixer_t *motor;
} mixer_t;

enum {
    ALIGN_GYRO = 0,
    ALIGN_ACCEL = 1,
    ALIGN_MAG = 2
};

typedef struct config_t {
    uint8_t  version;
    uint16_t size;
    uint8_t  magic_be;                      // magic number, should be 0xBE
    uint8_t  mixerConfiguration;
    uint32_t enabledFeatures;
    uint16_t looptime;                      // imu loop time in us
		uint8_t  oldcontroller;                 // This selects the original BF main PID Controller, but done with floatpoints but without DT
    uint8_t  P8[PIDITEMS];
    uint8_t  I8[PIDITEMS];
    uint8_t  D8[PIDITEMS];
    uint8_t  rcRate8;
    uint8_t  rcExpo8;
    uint8_t  thrMid8;
    uint8_t  thrExpo8;
    uint8_t  rollPitchRate;
    uint8_t  yawRate;
    uint8_t  dynThrPID;
    int16_t  accZero[3];
    int16_t  magZero[3];
    int16_t  mag_declination;               // Get your magnetic decliniation from here : http://magnetic-declination.com/
    uint8_t  mag_oldcalib;                  // use the old hard iron calibration?
    int16_t  angleTrim[2];                  // accelerometer trim
    // sensor-related stuff
    int8_t   align[3][3];                   // acc, gyro, mag alignment (ex: with sensor output of X, Y, Z, align of 1 -3 2 would return X, -Z, Y)
    uint8_t  acc_hardware;                  // Which acc hardware to use on boards with more than one device
    uint8_t  acc_lpf_factor;                // Set the Low Pass Filter factor for ACC. Increasing this value would reduce ACC noise (visible in GUI), but would increase ACC lag time. Zero = no filter
    uint16_t gyro_lpf;                      // mpuX050 LPF setting (TODO make it work on L3GD as well)
    uint16_t gyro_cmpf_factor;              // Set the Gyro Weight for Gyro/Acc complementary filter. Increasing this value would reduce and delay Acc influence on the output of the filter.
    uint32_t gyro_smoothing_factor;         // How much to smoothen with per axis (32bit value with Roll, Pitch, Yaw in bits 24, 16, 8 respectively
		float    accz_vel_cf;                   // Crashpilot: Value for complementary filter accz and barovelocity
		float    accz_alt_cf;                   // Crashpilot: Value for complementary filter accz and altitude
    float    baro_lag;                      // Lag of Baro
    float    barodownscale;                 // Scale downmovement down		
    uint8_t  nazedebug;                     // Crashpilot: 1 = Debug Barovalues
    uint8_t  moron_threshold;               // people keep forgetting that moving model while init results in wrong gyro offsets. and then they never reset gyro. so this is now on by default.

    uint32_t activate[CHECKBOXITEMS];       // activate switches
    uint8_t  vbatscale;                     // adjust this to match battery voltage to reported value
    uint8_t  vbatmaxcellvoltage;            // maximum voltage per cell, used for auto-detecting battery voltage in 0.1V units, default is 43 (4.3V)
    uint8_t  vbatmincellvoltage;            // minimum voltage per cell, this triggers battery out alarms, in 0.1V units, default is 33 (3.3V)
    uint8_t  power_adc_channel;             // which channel is used for current sensor. Right now, only 2 places are supported: RC_CH2 (unused when in CPPM mode, = 1), RC_CH8 (last channel in PWM mode, = 9)

    // Radio/ESC-related configuration
    uint8_t  rcmap[MAX_RC_CHANNELS];        // uint8_t rcmap[8]; // mapping of radio channels to internal RPYTA+ order
    uint8_t  auxChannels;                   // cGiesen: the number of supported aux channels. default = 4
    uint8_t  deadband;                      // introduce a deadband around the stick center for pitch and roll axis. Must be greater than zero.
    uint8_t  yawdeadband;                   // introduce a deadband around the stick center for yaw axis. Must be greater than zero.
    uint8_t  alt_hold_throttle_neutral;     // defines the neutral zone of throttle stick during altitude hold, default setting is +/-20
    uint8_t  phdeadband;                    // This Deadband adds to cfg.deadband
    uint8_t  spektrum_hires;                // spektrum high-resolution y/n (1024/2048bit)
    uint16_t midrc;                         // Some radios have not a neutral point centered on 1500. can be changed here
    uint16_t mincheck;                      // minimum rc end
    uint16_t maxcheck;                      // maximum rc end
    uint8_t  retarded_arm;                  // allow disarsm/arm on throttle down + roll left/right
		uint16_t killswitchtime;                // Time in ms when your arm switch becomes a Killswitch, 0 disables

    // Failsafe related configuration
    uint8_t  failsafe_delay;                // Guard time for failsafe activation after signal lost. 1 step = 0.1sec - 1sec in example (10)
    uint8_t  failsafe_off_delay;            // Time for Landing before motors stop in 0.1sec. 1 step = 0.1sec - 20sec in example (200)
    uint16_t failsafe_throttle;             // Throttle level used for landing - specify value between 1000..2000 (pwm pulse width for slightly below hover). center throttle = 1500.
    uint8_t  failsafe_deadpilot;		        // Time in sec when FS is engaged after idle on THR/YAW/ROLL/PITCH
    uint8_t  failsafe_justph;               // Does just PH&Autoland an not RTL,

    // motor/esc/servo related stuff
    uint16_t minthrottle;                   // Set the minimum throttle command sent to the ESC (Electronic Speed Controller). This is the minimum value that allow motors to run at a idle speed.
    uint16_t maxthrottle;                   // This is the maximum value for the ESCs at full power this value can be increased up to 2000
    uint8_t  passmotor;                     // Crashpilot: Only used with feature pass. If 0 = all Motors, otherwise specific Motor
    uint16_t mincommand;                    // This is the value for the ESCs when they are not armed. In some cases, this value must be lowered down to 900 for some specific ESCs
    uint16_t motor_pwm_rate;                // The update rate of motor outputs (50-498Hz)
    uint16_t servo_pwm_rate;                // The update rate of servo outputs (50-498Hz)
    int16_t  servotrim[8];                  // Adjust Servo MID Offset & Swash angles
    int8_t   servoreverse[8];               // Invert servos by setting -1

    // mixer-related configuration
    int8_t yaw_direction;
    uint16_t tri_yaw_middle;                // tail servo center pos. - use this for initial trim
    uint16_t tri_yaw_min;                   // tail servo min
    uint16_t tri_yaw_max;                   // tail servo max

    // flying wing related configuration
    uint16_t wing_left_min;                 // min/mid/max servo travel
    uint16_t wing_left_mid;
    uint16_t wing_left_max;
    uint16_t wing_right_min;
    uint16_t wing_right_mid;
    uint16_t wing_right_max;
    int8_t   pitch_direction_l;             // left servo - pitch orientation
    int8_t   pitch_direction_r;             // right servo - pitch orientation (opposite sign to pitch_direction_l if servos are mounted mirrored)
    int8_t   roll_direction_l;              // left servo - roll orientation
    int8_t   roll_direction_r;              // right servo - roll orientation  (same sign as ROLL_DIRECTION_L, if servos are mounted in mirrored orientation)

    // gimbal-related configuration
    int8_t   gimbal_pitch_gain;             // gimbal pitch servo gain (tied to angle) can be negative to invert movement
    int8_t   gimbal_roll_gain;              // gimbal roll servo gain (tied to angle) can be negative to invert movement
    uint8_t  gimbal_flags;                  // in servotilt mode, various things that affect stuff
    uint16_t gimbal_pitch_min;              // gimbal pitch servo min travel
    uint16_t gimbal_pitch_max;              // gimbal pitch servo max travel
    uint16_t gimbal_pitch_mid;              // gimbal pitch servo neutral value
    uint16_t gimbal_roll_min;               // gimbal roll servo min travel
    uint16_t gimbal_roll_max;               // gimbal roll servo max travel
    uint16_t gimbal_roll_mid;               // gimbal roll servo neutral value

    // Autoland
		uint8_t  autolandrate;                  // Temporary value "64" increase to increase Landingspeed

    // gps-related stuff
    uint8_t  gps_type;                      // Type of GPS hardware. 0: NMEA 1: UBX 2+ ??
		float    gps_ins_vel;                   // Crashpilot: Value for complementary filter INS and GPS Velocity
    float    gps_proj_smooth;               // Smoothes the speed for projecting coordinates (defined by gps_lag). The Speed is based on the "cfg.gps_ins_vel" result
    float    gps_lag;                       // This is the assumed time of GPS Lag, Ublox is supposed to be 0.8 sec behind
    float    gps_phase;                     // Make a phaseshift +-90 Deg max of GPS output
    uint8_t  acc_ins_lpf;                   // ACC lowpass for Acc GPS INS
		uint8_t  gps_ph_minsat;                 // Minimal Satcount for PH, PH on RTL is still done with 5Sats or more
		uint8_t  gps_ph_apm;                    // If 1 the original APM PH Controller is used
    uint16_t gps_ph_settletime;             // Time in ms before new absolute Position is taken into account after settlespeed reached
    uint8_t  gps_ph_settlespeed;            // PH settlespeed in cm/s
    uint16_t gps_ph_targetsqrt;             // This is the speed target of PH. That means if the copter moves faster than that, the maximal tiltangle reduced dramatically
    float    gps_phmove_speed;              // DONT USE THIS FOR FLIGHT! PH move speed // 0 disables PH move - recommended!!
    uint8_t  gps_maxangle;                  // maximal over all GPS bank angle
    uint8_t  gps_minanglepercent;           // Percent 1-100% of gps_maxangle for minimal tilt, as lower limit for "gps_ph_targetsqrt"
    uint32_t gps_baudrate;                  // GPS baudrate
    uint16_t gps_wp_radius;                 // if we are within this distance to a waypoint then we consider it reached (distance is in cm)
    uint8_t  gps_rtl_mindist;               // Minimal distance for RTL, otherwise it will just autoland, prevent Failsafe jump in your face, when arming copter and turning off TX
    uint8_t  gps_rtl_flyaway;               // If during RTL the distance increases beyond this valus in meters, something is wrong, autoland
    uint8_t  gps_yaw;                       // Thats the MAG P during GPS functions, substitute for "cfg.P8[PIDMAG]"
    uint8_t  nav_slew_rate;                 // Adds a rate control to nav output, will smoothen out nav angle spikes
    uint8_t  nav_tail_first;                // 1 = Copter comes back with ass first (only works with nav_controls_heading = 1) 
    uint8_t  nav_controls_heading;          // copter faces toward the navigation point, maghold must be enabled for it
		uint8_t  nav_rtl_lastturn;              // Something like NAV_SET_TAKEOFF_HEADING on mwii
    int16_t  nav_speed_min;                 // cm/sec
    int16_t  nav_speed_max;                 // cm/sec
    uint16_t gps_rtl_minhight;              // Minimal RTL hight in m, 0 disable  // Crashpilot

    uint32_t serial_baudrate;               // serial(uart1) baudrate

	  // LED Stuff
	  uint8_t  led_invert;                    // Crashpilot invert LED 0&1

	  //cGiesen start
	  uint8_t  LED_Type;                      // 1=MWCRGB / 2=MONO_LED / 3=LEDRing
	  uint8_t  LED_Pinout;                    // choose LED pinout (MONO_LED: 0=LED rc5, 1=LED rc6 / MWCRGB: coming soon)
	  uint8_t  LED_ControlChannel;            // RC Channel to control the LED Pattern
	  uint8_t  LED_Armed;          		        // 0 = Show LED only if armed, 1 = always show LED
	  uint16_t LED_Toggle_Delay;              //16bit bit pattern to slow down led patterns
    uint32_t LED_Pattern1;            	  	// 32bit bit pattern to have flickering led patterns / the pattern for MWCRGB 1000-2000
    uint32_t LED_Pattern2;            		  // 32bit bit pattern to have flickering led patterns / the pattern for MWCRGB 1000-2000
    uint32_t LED_Pattern3;            		  // 32bit bit pattern to have flickering led patterns / the pattern for MWCRGB 1000-2000

    // Sonar Stuff
    uint8_t  SONAR_Pinout;                  // choose sonar pinout 0=sonar_pwm56,1=sonar_rc78
    //cGiesen end

    motorMixer_t customMixer[MAX_MOTORS];   // custom mixtable
    uint8_t  magic_ef;                      // magic number, should be 0xEF
    uint8_t  chk;                           // XOR checksum
} config_t;

typedef struct flags_t {
    uint8_t OK_TO_ARM;
    uint8_t ARMED;
    uint8_t ACC_CALIBRATED;
    uint8_t ANGLE_MODE;
    uint8_t HORIZON_MODE;
    uint8_t MAG_MODE;
    uint8_t BARO_MODE;
    uint8_t GPS_HOME_MODE;
    uint8_t GPS_HOLD_MODE;
    uint8_t HEADFREE_MODE;
    uint8_t PASSTHRU_MODE;
    uint8_t GPS_FIX;
    uint8_t GPS_FIX_HOME;
    uint8_t SMALL_ANGLES_25;
    uint8_t CALIBRATE_MAG;
} flags_t;

extern int16_t  gyroZero[3];
extern int16_t  gyroData[3];
extern int16_t  angle[2];
extern int16_t  axisPID[3];
extern int16_t  rcCommand[4];
extern uint8_t  rcOptions[CHECKBOXITEMS];
extern uint16_t failsafeCnt;
extern float    TiltValue;

extern int16_t  debug[4];
extern int16_t  gyroADC[3], accADC[3], accSmooth[3], magADC[3];
extern uint16_t acc_1G;
extern uint32_t currentTime;
extern uint32_t previousTime;
extern uint16_t cycleTime;
extern uint16_t calibratingA;
extern uint16_t calibratingG;

extern int16_t  annex650_overrun_count;
extern int32_t  BaroAlt;
extern int16_t  sonarAlt;
extern int32_t  EstAlt;
extern int32_t  AltHold;
extern int16_t  vario;

extern uint16_t BaroDeltaTime;             //Crashpilot
extern int16_t  BaroP;                     //Crashpilot
extern int16_t  BaroI;                     //Crashpilot
extern int16_t  BaroD;                     //Crashpilot
extern uint8_t  newbaroalt;                //Crashpilot
extern uint32_t BaroFstTime;               //Crashpilot

extern int16_t  zVelocity;
extern int16_t  motor[MAX_MOTORS];
extern int16_t  servo[8];
extern int16_t  rcData[MAX_RC_CHANNELS];    // extern int16_t rcData[8];
extern int16_t  rcDataSAVE[MAX_RC_CHANNELS];

extern uint8_t  vbat;
extern int16_t  telemTemperature1;          // gyro sensor temperature
extern int16_t  lookupPitchRollRC[6];       // lookup table for expo & RC rate PITCH+ROLL
extern int16_t  lookupThrottleRC[11];       // lookup table for expo & mid THROTTLE
extern uint8_t  toggleBeep;

// MAG
extern float    headFreeModeHold;
extern float    heading;
extern float    magHold;
extern float    magneticDeclination;

// GPS stuff
extern int32_t  GPS_coord[2];
extern int32_t  GPS_home[2];
extern int32_t  GPS_WP[2];                                   // Currently used WP
extern uint8_t  GPS_numSat;
extern uint16_t GPS_distanceToHome;                          // distance to home
extern int16_t  GPS_directionToHome;                         // direction to home
extern uint16_t GPS_altitude,GPS_speed;                      // altitude in 0.1m and speed in 0.1m/s
extern uint8_t  GPS_update;                                  // it's a binary toogle to distinct a GPS position update
extern float    GPS_angle[2];                                // it's the angles that must be applied for GPS correction
extern uint16_t GPS_ground_course;                           // degrees*10
extern uint8_t  GPS_Present;                                 // Checksum from Gps serial
extern uint8_t  GPS_Enable;
extern float    nav[2];
extern int8_t   nav_mode;                                    // Navigation mode
extern int8_t   wp_mode;
extern float    nav_rated[2];                                // Adding a rate controller to the navigation to make it smoother
extern int32_t  WP_Target_Alt;
extern int16_t  WP_Desired_Climbrate;
extern bool     WP_Fastcorner;                               // Dont decrease Speed at Target
extern float    sin_yaw_y;
extern float    cos_yaw_x;

// General
extern config_t cfg;
extern flags_t  f;
extern sensor_t acc;
extern sensor_t gyro;
extern baro_t   baro;

// MWCRGB
extern uint32_t LED_Value;

// main
void loop(void);
void pass(void);
void LD0_OFF(void);                 // Crashpilot LED Inverter stuff
void LD1_OFF(void);
void LD0_ON(void);
void LD1_ON(void);

// IMU
void imuInit(void);
void annexCode(void);
void computeIMU(void);
void blinkLED(uint8_t num, uint8_t wait, uint8_t repeat);
void getEstimatedAltitude(void);

// Sensors
void sensorsAutodetect(void);
void batteryInit(void);
uint16_t batteryAdcToVoltage(uint16_t src);
void ACC_getADC(void);
void Baro_update(void);
void Gyro_getADC(void);
void Mag_init(void);
int  Mag_getADC(void);
void Sonar_init(void);
void Sonar_update(void);

// Output
void mixerInit(void);
void mixerLoadMix(int index);
void writeServos(void);
void writeMotors(void);
void writeAllMotors(int16_t mc);
void mixTable(void);

// Serial
void serialInit(uint32_t baudrate);
void serialCom(void);
void serialOSD(void);

// Config
void parseRcChannels(const char *input);
void readEEPROM(void);
void writeParams(uint8_t b);
void checkFirstTime(bool reset);
bool sensors(uint32_t mask);
void sensorsSet(uint32_t mask);
void sensorsClear(uint32_t mask);
uint32_t sensorsMask(void);
bool feature(uint32_t mask);
void featureSet(uint32_t mask);
void featureClear(uint32_t mask);
void featureClearAll(void);
uint32_t featureMask(void);

// General RC stuff
void computeRC(void);
void GetActualRCdataOutRCDataSave(void);

// RC spektrum
void spektrumInit(void);
bool spektrumFrameComplete(void);

// buzzer
void buzzer(uint8_t warn_vbat);

// cli
void cliProcess(void);

// gps
void gpsInit(uint32_t baudrate);
void GPS_set_pids(void);
void GPS_reset_home_position(void);
void GPS_reset_nav(void);
void GPS_set_next_wp(int32_t* lat, int32_t* lon);
void GPS_alltime(void);
bool DoingGPS(void);
float wrap_18000(float);
int16_t RCDeadband(int16_t rcvalue, uint8_t rcdead);

// telemetry
void initTelemetry(bool State);
void sendTelemetry(void);

//Init the led gpio port when enabled
void ledToggleInit();

//Update the leds, enabled signals that the leds are enabled
void ledToggleUpdate(bool activated);