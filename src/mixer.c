//*********************************************************************
//* mixer.c  March 2013    Crashpilot 1000
//*          Based on the Timecop /  OpenAero32 Mwii Port
//*          V1.1 @ Johannes, Airplane, Flying_Wing added
//*********************************************************************

//***********************************************************
//* Includes
//***********************************************************
#include "board.h"
#include "mw.h"
#include "drv_pwm.h"

#define PID_1_MIX(X) rcCommand[THROTTLE] + axisPID[ROLL]*X

//************************************************************
// Variables
//************************************************************
static uint8_t  numberMotor = 0;
uint8_t         useServo = 0;
int16_t         motor[MAX_MOTORS];
int16_t         servo[MAX_SERVOS];

static motorMixer_t currentMixer[MAX_MOTORS];

static const motorMixer_t mixerTri[] =
{
    { 1.0f,  0.0f,  1.333333f,  0.0f },     // REAR
    { 1.0f, -1.0f, -0.666667f,  0.0f },     // RIGHT
    { 1.0f,  1.0f, -0.666667f,  0.0f },     // LEFT
};

static const motorMixer_t mixerQuadP[] =
{
    { 1.0f,  0.0f,  1.0f, -1.0f },          // REAR
    { 1.0f, -1.0f,  0.0f,  1.0f },          // RIGHT
    { 1.0f,  1.0f,  0.0f,  1.0f },          // LEFT
    { 1.0f,  0.0f, -1.0f, -1.0f },          // FRONT
};

static const motorMixer_t mixerQuadX[] =
{
    { 1.0f, -1.0f,  1.0f, -1.0f },          // REAR_R
    { 1.0f, -1.0f, -1.0f,  1.0f },          // FRONT_R
    { 1.0f,  1.0f,  1.0f,  1.0f },          // REAR_L
    { 1.0f,  1.0f, -1.0f, -1.0f },          // FRONT_L
};

/* static const motorMixer_t mixerBi[] = {
    { 1.0f,  1.0f,  0.0f,  0.0f },          // LEFT
    { 1.0f, -1.0f,  0.0f,  0.0f },          // RIGHT
}; doesn't work Johannes */

static const motorMixer_t mixerY6[] =
{
    { 1.0f,  0.0f,  1.333333f,  1.0f },     // REAR
    { 1.0f, -1.0f, -0.666667f, -1.0f },     // RIGHT
    { 1.0f,  1.0f, -0.666667f, -1.0f },     // LEFT
    { 1.0f,  0.0f,  1.333333f, -1.0f },     // UNDER_REAR
    { 1.0f, -1.0f, -0.666667f,  1.0f },     // UNDER_RIGHT
    { 1.0f,  1.0f, -0.666667f,  1.0f },     // UNDER_LEFT
};

static const motorMixer_t mixerHex6P[] =
{
    { 1.0f, -1.0f,  0.866025f,  1.0f },     // REAR_R
    { 1.0f, -1.0f, -0.866025f, -1.0f },     // FRONT_R
    { 1.0f,  1.0f,  0.866025f,  1.0f },     // REAR_L
    { 1.0f,  1.0f, -0.866025f, -1.0f },     // FRONT_L
    { 1.0f,  0.0f, -0.866025f,  1.0f },     // FRONT
    { 1.0f,  0.0f,  0.866025f, -1.0f },     // REAR
};

static const motorMixer_t mixerY4[] =
{
    { 1.0f,  0.0f,  1.0f, -1.0f },          // REAR_TOP CW
    { 1.0f, -1.0f, -1.0f,  0.0f },          // FRONT_R CCW
    { 1.0f,  0.0f,  1.0f,  1.0f },          // REAR_BOTTOM CCW
    { 1.0f,  1.0f, -1.0f,  0.0f },          // FRONT_L CW
};

static const motorMixer_t mixerHex6X[] =
{
    { 1.0f, -0.866025f,  1.0f,  1.0f },     // REAR_R
    { 1.0f, -0.866025f, -1.0f,  1.0f },     // FRONT_R
    { 1.0f,  0.866025f,  1.0f, -1.0f },     // REAR_L
    { 1.0f,  0.866025f, -1.0f, -1.0f },     // FRONT_L
    { 1.0f, -0.866025f,  0.0f, -1.0f },     // RIGHT
    { 1.0f,  0.866025f,  0.0f,  1.0f },     // LEFT
};

static const motorMixer_t mixerOctoX8[] =
{
    { 1.0f, -1.0f,  1.0f, -1.0f },          // REAR_R
    { 1.0f, -1.0f, -1.0f,  1.0f },          // FRONT_R
    { 1.0f,  1.0f,  1.0f,  1.0f },          // REAR_L
    { 1.0f,  1.0f, -1.0f, -1.0f },          // FRONT_L
    { 1.0f, -1.0f,  1.0f,  1.0f },          // UNDER_REAR_R
    { 1.0f, -1.0f, -1.0f, -1.0f },          // UNDER_FRONT_R
    { 1.0f,  1.0f,  1.0f, -1.0f },          // UNDER_REAR_L
    { 1.0f,  1.0f, -1.0f,  1.0f },          // UNDER_FRONT_L
};

static const motorMixer_t mixerOctoFlatP[] =
{
    { 1.0f,  0.707107f, -0.707107f,  1.0f },    // FRONT_L
    { 1.0f, -0.707107f, -0.707107f,  1.0f },    // FRONT_R
    { 1.0f, -0.707107f,  0.707107f,  1.0f },    // REAR_R
    { 1.0f,  0.707107f,  0.707107f,  1.0f },    // REAR_L
    { 1.0f,  0.0f, -1.0f, -1.0f },              // FRONT
    { 1.0f, -1.0f,  0.0f, -1.0f },              // RIGHT
    { 1.0f,  0.0f,  1.0f, -1.0f },              // REAR
    { 1.0f,  1.0f,  0.0f, -1.0f },              // LEFT
};

static const motorMixer_t mixerOctoFlatX[] =
{
    { 1.0f,  1.0f, -0.5f,  1.0f },          // MIDFRONT_L
    { 1.0f, -0.5f, -1.0f,  1.0f },          // FRONT_R
    { 1.0f, -1.0f,  0.5f,  1.0f },          // MIDREAR_R
    { 1.0f,  0.5f,  1.0f,  1.0f },          // REAR_L
    { 1.0f,  0.5f, -1.0f, -1.0f },          // FRONT_L
    { 1.0f, -1.0f, -0.5f, -1.0f },          // MIDFRONT_R
    { 1.0f, -0.5f,  1.0f, -1.0f },          // REAR_R
    { 1.0f,  1.0f,  0.5f, -1.0f },          // MIDREAR_L
};

static const motorMixer_t mixerVtail4[] =
{
    { 1.0f,  0.0f,  1.0f,  1.0f },          // REAR_R
    { 1.0f, -1.0f, -1.0f,  0.0f },          // FRONT_R
    { 1.0f,  0.0f,  1.0f, -1.0f },          // REAR_L
    { 1.0f,  1.0f, -1.0f, -0.0f },          // FRONT_L
};

// new Type, a copy from Baseflight
static const motorMixer_t mixerHexV6[] = {
    { 1.0f,  1.25f,  1.0f,  1.25f },    // front l
    { 1.0f, -1.0f,   0.0f,  1.0 },      // mid r
    { 1.0f,  0.75f,  1.0f,  0.75f },    // REAR_L
    { 1.0f,  1.0f,   0.0f, -1.0f },     // mid_L
    { 1.0f,  1.25f, -1.0f, -1.25f },    // front r
    { 1.0f,  0.75f,  1.0f, -0.75f },    // rear l
};

// Keep this synced with MultiType struct in mw.h!
const mixer_t mixers[] = {
//    Mo Se Mixtable
    { 0, 0, NULL },                // 0  entry 0
    { 3, 1, mixerTri },            // 1  MULTITYPE_TRI
    { 4, 0, mixerQuadP },          // 2  MULTITYPE_QUADP
    { 4, 0, mixerQuadX },          // 3  MULTITYPE_QUADX
    { 2, 1, NULL },                // 4  * MULTITYPE_BI
//    { 2, 1, mixerBi },             // 4  MULTITYPE_BI  @Johannes
    { 0, 1, NULL },                // 5  * MULTITYPE_GIMBAL
    { 6, 0, mixerY6 },             // 6  MULTITYPE_Y6
    { 6, 0, mixerHex6P },          // 7  MULTITYPE_HEX6
    { 2, 1, NULL },                // 8  * MULTITYPE_FLYING_WING
    { 4, 0, mixerY4 },             // 9  MULTITYPE_Y4
    { 6, 0, mixerHex6X },          // 10 MULTITYPE_HEX6X
    { 8, 0, mixerOctoX8 },         // 11 MULTITYPE_OCTOX8
    { 8, 0, mixerOctoFlatP },      // 12 MULTITYPE_OCTOFLATP
    { 8, 0, mixerOctoFlatX },      // 13 MULTITYPE_OCTOFLATX
    { 2, 1, NULL },                // 14 * MULTITYPE_AIRPLANE
    { 0, 1, NULL },                // 15 * MULTITYPE_HELI_120_CCPM
    { 0, 1, NULL },                // 16 * MULTITYPE_HELI_90_DEG
    { 4, 0, mixerVtail4 },         // 17 MULTITYPE_VTAIL4
    { 6, 0, NULL },                // 18 * MULTITYPE_HEX6H,       not implemented
    { 0, 0, NULL },                // 19 * MULTITYPE_CUSTOM
    { 2, 1, NULL },                // 20 * MULTITYPE_DUALCOPTER, not implemented 
    { 2, 1, NULL },                // 21 * MULTITYPE_SINGLECOPTER, not implemented 
    { 6, 1, mixerHexV6 },          // 22 MULTITYPE_HEXV6          @Johannes, its a copy from Baseflight
    { 2, 1, NULL },                // 23 * MULTITYPE_FW_DRAG      @Johannes
    { 2, 1, NULL },                // 24 * MULTITYPE_TILTROTOR    @Johannes

};


// **********************
// writeMotors
// **********************
void writeMotors(void)
{
    uint8_t i;
    for (i = 0; i < numberMotor; i++)
        pwmWriteMotor(i, motor[i]);
}


// **********************
// writeAllMotors
// **********************
void writeAllMotors(int16_t mc)
{
    uint8_t i;
    // Sends commands to all motors
    for (i = 0; i < numberMotor; i++)
        motor[i] = mc;
    writeMotors();
}


// **********************
// mixerInit
// **********************
uint8_t  mixerInit(void)
{
    int i;

    // enable servos for mixes that require them. note, this shifts motor counts.
    useServo = mixers[cfg.mixerConfiguration].useServo;
    // if we want camstab/trig, that also enables servos, even if mixer doesn't
    if (feature(FEATURE_SERVO_TILT))
        useServo = 1;

    if (cfg.mixerConfiguration == MULTITYPE_CUSTOM)
    {
        // load custom mixer into currentMixer
        for (i = 0; i < MAX_MOTORS; i++)
        {
            // check if done
            if (cfg.customMixer[i].throttle == 0.0f)
                break;
            currentMixer[i] = cfg.customMixer[i];
            numberMotor++;
        }
    }
    else
    {
        numberMotor = mixers[cfg.mixerConfiguration].numberMotor;
        // copy motor-based mixers
        if (mixers[cfg.mixerConfiguration].motor)
        {
            for (i = 0; i < numberMotor; i++)
                currentMixer[i] = mixers[cfg.mixerConfiguration].motor[i];
        }
    }
    if (useServo)
    {
        for (i = 0; i<MAX_SERVOS; i++)
            servo[i] = cfg.servoConf[i].middle;
    }
    return numberMotor;
}


// **********************
// mixerLoadMix
// **********************
void mixerLoadMix(int index)
{
    int i;

    // we're 1-based
    index++;
    // clear existing
    for (i = 0; i < MAX_MOTORS; i++)
        cfg.customMixer[i].throttle = 0.0f;

    // do we have anything here to begin with?
    if (mixers[index].motor != NULL)
    {
        for (i = 0; i < mixers[index].numberMotor; i++)
            cfg.customMixer[i] = mixers[index].motor[i];
    }
}


// **********************
// writeServos
// **********************
void writeServos(void)
{
    if (!useServo)
        return;

    switch (cfg.mixerConfiguration)
    {
        case MULTITYPE_BI:
            pwmWriteServo(0, servo[4]);
            pwmWriteServo(1, servo[5]);
            break;

        case MULTITYPE_TRI:
            pwmWriteServo(0, servo[5]);
            break;

        case MULTITYPE_AIRPLANE:
            pwmWriteServo(0, servo[3]); // Left aileron
            pwmWriteServo(1, servo[4]); // Right aileron
            pwmWriteServo(2, servo[5]); // Rudder
            pwmWriteServo(3, servo[6]); // Elevator
            if (cfg.flapmode > 0)
                pwmWriteServo(4, servo[2]); // Flap
            break;

#ifdef DEFINED_FW_DRAG
        case MULTITYPE_FW_DRAG:
            pwmWriteServo(0, servo[0]); // Left elevon
            pwmWriteServo(1, servo[1]); // Right elevon
            pwmWriteServo(2, servo[2]); // Left drag rudder
            pwmWriteServo(3, servo[3]); // Right drag rudder
        break;
#endif
        case MULTITYPE_FLYING_WING:
            pwmWriteServo(0, servo[3]); // Left elevon
            pwmWriteServo(1, servo[4]); // Right elevon
            pwmWriteServo(2, servo[5]); // Rudder
            break;

#ifdef DEFINED_TILTROTOR
        case MULTITYPE_TILTROTOR:
            pwmWriteServo(0, servo[3]); // Left elevon
            pwmWriteServo(1, servo[4]); // Right elevon
            pwmWriteServo(2, servo[5]); // Rudder
            pwmWriteServo(3, servo[6]); // Elevator
            if (cfg.flapmode > 0)
              pwmWriteServo(4, servo[2]); // Flap
            pwmWriteServo(5, servo[1]); // Engine Pod's
            break;
#endif
        case MULTITYPE_GIMBAL:
            pwmWriteServo(0, servo[0]);
            pwmWriteServo(1, servo[1]);
            break;

        default:
            // Two servos for SERVO_TILT, if enabled
            if (feature(FEATURE_SERVO_TILT))
            {
                pwmWriteServo(0, servo[0]);
                pwmWriteServo(1, servo[1]);
            }
            break;
    }
}



// **********************
// airplaneMixer
// **********************
static void airplaneMixer(void)
{
    static int16_t flaperons = 0;
    static int16_t flap = 0;
    static int16_t slowFlaps = 0;
    static uint8_t flapskip;
    uint8_t first_servo = 0;
    uint8_t last_servo = 0;
    uint8_t speed;
    uint8_t i;
    #define SERVODIR(n,b) ((cfg.servoConf[n].rate & b) ? -1 : 1)		

    // 1. Throttle is handled separately here for all planes
    motor[0] = rcData[THROTTLE];  // Send directly from RC for now
    motor[1] = rcData[THROTTLE];  // Copy to motor[0]

    // 2. Recover flap and aileron info
    switch(cfg.flapmode)
    {
        // No flaperons, no flap's
        case NO_FLAP: // 0
            flap = 0;
            break;

        // No flaperons - standard flap in one servo channel
        case BASIC_FLAP: // 1
            flap = rcData[cfg.flapchan] - cfg.mincommand;  // normaly AUX2, see config.c
            break;

        // Flaperons - two ailerons with flaps pre-mixed in the TX
        case PREMIXED_FLAP: // 2
            flap = rcData[cfg.flapchan] - cfg.mincommand;     // normaly AUX2, see config.c
            break;

#if 0
         // isn't working currently Johannes
        // Flaperons - two independant aileron channels + one flap input on cfg.flapchan
        case ADV_FLAP: // 3
            left_roll = rcCommand[ROLL];
            right_roll = rcCommand[cfg.aileron2];
            // Ignore if no flap channel
            if (cfg.flapchan != NOCHAN)
                flap = rcCommand[cfg.flapchan];  // normaly AUX2, see config.c
                else flap = 0;
            break;
#endif
        default:
            break;
    }

    if (cfg.flapmode > 0){
        // Do flap speed control
        if (cfg.flapspeed)
    {
            if (abs(slowFlaps - flap) >= cfg.flapstep)  // Difference larger than one step, so ok
                speed = cfg.flapstep;                   // Need to manipulate speed as target approaches
            else    speed = 1;                          // Otherwise this will oscillate

            if ((slowFlaps < flap) && (flapskip == cfg.flapspeed))
                slowFlaps += speed;
    else
                if ((slowFlaps > flap) && (flapskip == cfg.flapspeed))
                    slowFlaps -= speed;
        }
        // No speed control requested so copy flaps
        else    slowFlaps = flap;

        flapskip++;
        if (flapskip > cfg.flapspeed) flapskip = 0;

        if (slowFlaps < 0) slowFlaps = 0;
    }
    else slowFlaps = flap;

    if (slowFlaps > cfg.flapmaxmove) slowFlaps = cfg.flapmaxmove;

    if (cfg.flapmode < 2)
         flaperons = 0;
    else flaperons = slowFlaps;
    
    // 3. Servomixing
    switch (cfg.mixerConfiguration) {

        // tested
        case MULTITYPE_FLYING_WING:
            first_servo = 0;
            last_servo = 5;
            if (f.PASSTHRU_MODE) {
                // don't use sensors for correction, simple 2 channel mixing
                servo[3] = (SERVODIR(0,1) * (rcCommand[PITCH] - rcCommand[ROLL])); // Left elevon
                servo[4] = (SERVODIR(1,1) * (rcCommand[PITCH] + rcCommand[ROLL])); // Right elevon
                servo[5] = (SERVODIR(5,1) * rcCommand[YAW]);                       // Rudder
            } else {
                // use sensors to correct (gyro only or gyro+acc according to aux1/aux2 configuration
                servo[3] = (SERVODIR(0,1) * (axisPID[PITCH]  - axisPID[ROLL]));    // Left elevon
                servo[4] = (SERVODIR(1,1) * (axisPID[PITCH]  + axisPID[ROLL]));    // Right elevon
                servo[5] = (SERVODIR(5,1) * axisPID[YAW]);                         // Rudder
        }

        // development in progress, not tested !!
        case MULTITYPE_AIRPLANE:
            first_servo = 2;
            last_servo = 6;
            // Basic functions
            servo[2] = (SERVODIR(2,1) * (slowFlaps));                   // Speed-controlled flap
            servo[3] = (SERVODIR(3,1) * (rcCommand[ROLL] - flaperons)); // Left flaperon or Aileron
            servo[4] = (SERVODIR(4,1) * (rcCommand[ROLL] + flaperons)); // Right flaperon or Aileron
            servo[5] = (SERVODIR(5,1) * (rcCommand[YAW]));              // Rudder
            servo[6] = (SERVODIR(6,1) * (rcCommand[PITCH]));            // Elevator
            servo[7] = motor[0];                                        // only for GUI-Display
            if (!f.PASSTHRU_MODE) {
                // use sensors to correct (gyro only or gyro+acc according to aux1/aux2 configuration						
                servo[3] += (SERVODIR(3,1)  * axisPID[ROLL]);           // Stabilised left flaperon or Aileron
                servo[4] += (SERVODIR(4,1)  * axisPID[ROLL]);           // Stabilised right flaperon
                servo[5] += (SERVODIR(5,1)  * axisPID[YAW]);            // Stabilised Rudder
                servo[6] += (SERVODIR(6,1)  * axisPID[PITCH]);          // Stabilised Elevator
        }
            break;


#ifdef DEFINED_TILTROTOR
        // development in progress, not tested
        case MULTITYPE_TILTROTOR: // new type Johannes:
            first_servo = 1;
            last_servo = 6;
            //servo[2]  = (SERVODIR(2,1) * rcData[AUX3]);               // Flaps  TODO today, flaps have the same pos. as the Engine Pod's
            servo[2] = (SERVODIR(2,1) * (slowFlaps));                   // Speed-controlled flap

            if ((rcData[AUX2]) < cfg.tilt_vtol_detect)
            {
                /**** Heli-Mode ****/
                motor[0] = PID_1_MIX(+1); // LEFT, with Roll, without YAW
                motor[1] = PID_1_MIX(-1); // RIGHT
                //  Engine Pod's
                servo[0] = /*(SERVODIR(0,1) * */ (cfg.yawPIDpol * axisPID[YAW]) + axisPID[PITCH] /*)*/; //  left
                servo[1] = /*(SERVODIR(1,1) * */ (cfg.yawPIDpol * axisPID[YAW]) - axisPID[PITCH] /*)*/; //  right

                /*servo[3]  = axisPID[ROLL]; //  Left Aileron
                servo[4]  = axisPID[ROLL]; //  Right Aileron
                servo[5]  = axisPID[YAW];  //  Rudder
                servo[6]  = axisPID[PITCH];//  Elevator  */
            }// else {
                /**** Heli-Mode ****/
                /**** Airplane-Mode ****/
                // Basic functions
                servo[3] = /*(SERVODIR(3,1) * */(rcCommand[ROLL] - flaperons)/*)*/; // Left flaperon or Aileron
                servo[4] = /*(SERVODIR(4,1) * */(rcCommand[ROLL] + flaperons)/*)*/; // Right flaperon
                servo[5] = /*(SERVODIR(5,1) * */(rcCommand[YAW])/*)*/;              // Rudder
                servo[6] = /*(SERVODIR(6,1) * */(rcCommand[PITCH])/*)*/;            // Elevator
                servo[7] = motor[0];                                        // only for GUI-Display
                if (!f.PASSTHRU_MODE)
                {
                    servo[3] += /*(SERVODIR(3,1)  * */ axisPID[ROLL]/*)*/;           // Stabilised left flaperon or Aileron
                    servo[4] += /*(SERVODIR(4,1)  * */ axisPID[ROLL]/*)*/;           // Stabilised right flaperon
                    servo[5] += /*(SERVODIR(5,1)  * */ axisPID[YAW]/*)*/;            // Stabilised Rudder
                    servo[6] += /*(SERVODIR(6,1)  * */ axisPID[PITCH]/*)*/;          // Stabilised Elevator
                }
            //}
            break;
    #if 1
            // Debug output, only for test
            debug[0] = servo[3];
            debug[1] = servo[5];
            debug[2] = flaperons;
            debug[3] = slowFlaps;
    #endif
#endif

#ifdef DEFINED_FW_DRAG
        // isn't working/tested currently
        case MULTITYPE_FW_DRAG:
            first_servo = 0;
            last_servo = 3;
            // Basic functions
            servo[0] = (rcCommand[PITCH] + rcCommand[ROLL]) >> 1;   // Left elevon
            servo[1] = (rcCommand[PITCH] - rcCommand[ROLL]) >> 1;   // Right elevon
            if (rcCommand[YAW] >= 0) servo[2] = rcCommand[YAW];     // Left drag rudder
            else                     servo[2] = 0;
            if (rcCommand[YAW] <= 0) servo[3] = rcCommand[YAW];     // Right drag rudder
            else                     servo[3] = 0;
            // Ignore if in pass-through mode
            if (!f.PASSTHRU_MODE)
            {
                // Stabilised left elevon
                servo[0] = servo[0] + (cfg.pitchPIDpol * axisPID[PITCH]) - (cfg.rollPIDpol * axisPID[ROLL]);
                // Stabilised right elevon
                servo[1] = servo[1] + (cfg.pitchPIDpol * axisPID[PITCH]) + (cfg.rollPIDpol * axisPID[ROLL]);
                // Stabilised drag rudders (only for one side of movement)
                if (axisPID[YAW] < 0) servo[2] -= axisPID[YAW];
                if (axisPID[YAW] > 0) servo[3] -= axisPID[YAW];
            }
            break;
#endif

        default:
            break;
    }

    // Reverse, offset, then check all servo outputs against mechanical endpoints
    for (i = first_servo; i < (last_servo+1); i++) {
        servo[i]  = ((int32_t)cfg.servoConf[i].rate * servo[i])/100L;  // servo rates in
        servo[i] += cfg.servoConf[i].middle;
        if (servo[i] < cfg.servoConf[i].min) servo[i] = cfg.servoConf[i].min;
        if (servo[i] > cfg.servoConf[i].max) servo[i] = cfg.servoConf[i].max;
    }
}


// **********************
// mixTable
// **********************
void mixTable(void)
{
    int16_t maxMotor;
    uint32_t i;

    if (numberMotor > 3)
    {
        // prevent "yaw jump" during yaw correction
        axisPID[YAW] = constrain(axisPID[YAW], -100 - abs(rcCommand[YAW]), +100 + abs(rcCommand[YAW]));
    }

    // motors for non-servo mixes
    if ( (numberMotor > 1) & (!cfg.airplane)  & ( !(cfg.mixerConfiguration == MULTITYPE_BI)) ) 
    {
        for (i = 0; i < numberMotor; i++)
            motor[i] = rcCommand[THROTTLE] * currentMixer[i].throttle + axisPID[PITCH] * currentMixer[i].pitch + axisPID[ROLL] * currentMixer[i].roll + cfg.yaw_direction * axisPID[YAW] * currentMixer[i].yaw;
    }

    // airplane / servo mixes
    switch (cfg.mixerConfiguration)
    {
        case MULTITYPE_BI:
            motor[0] = PID_1_MIX(+1); // LEFT, without YAW mix
            motor[1] = PID_1_MIX(-1); // RIGHT without YAW mix
            servo[4] = constrain(cfg.servoConf[4].middle + (cfg.yaw_direction * axisPID[YAW]) + axisPID[PITCH], cfg.servoConf[4].min , cfg.servoConf[4].max );   //LEFT  Johannes:
            servo[5] = constrain(cfg.servoConf[5].middle - (cfg.yaw_direction * axisPID[YAW]) - axisPID[PITCH], cfg.servoConf[5].min , cfg.servoConf[5].max );   //RIGHT Johannes:
        break;

        case MULTITYPE_TRI:
            servo[5] = constrain(cfg.servoConf[5].middle + cfg.yaw_direction * axisPID[YAW], cfg.servoConf[5].min, cfg.servoConf[5].max); //REAR Johannes
        break;

        case MULTITYPE_GIMBAL:
            servo[0] = constrain(cfg.servoConf[0].middle + cfg.gimbal_pitch_gain * angle[PITCH] / 16 + rcCommand[PITCH], cfg.servoConf[0].min, cfg.servoConf[0].max);
            servo[1] = constrain(cfg.servoConf[1].middle + cfg.gimbal_roll_gain  * angle[ROLL]  / 16 + rcCommand[ROLL] , cfg.servoConf[1].min, cfg.servoConf[1].max);
        break;

        case MULTITYPE_AIRPLANE:
#ifdef DEFINED_TILTROTOR
        case MULTITYPE_TILTROTOR:
#endif
#ifdef DEFINED_FW_DRAG
        case MULTITYPE_FW_DRAG:
#endif
        case MULTITYPE_FLYING_WING:
            airplaneMixer();
        break;

        default:
        break;
    }

    // do camstab
    if (feature(FEATURE_SERVO_TILT))
    {
        uint16_t aux[2] = { 0, 0 };

        if ((cfg.gimbal_flags & GIMBAL_NORMAL) || (cfg.gimbal_flags & GIMBAL_TILTONLY))
            aux[0] = rcData[AUX3] - cfg.midrc;
        if (!(cfg.gimbal_flags & GIMBAL_DISABLEAUX34))
            aux[1] = rcData[AUX4] - cfg.midrc;

        servo[0] = cfg.servoConf[0].middle + aux[0];
        servo[1] = cfg.servoConf[1].middle + aux[1];

        if (rcOptions[BOXCAMSTAB])
        {
            if (cfg.gimbal_flags & GIMBAL_MIXTILT)
            {
                servo[0] -= (-cfg.gimbal_pitch_gain) * angle[PITCH] / 16 - cfg.gimbal_roll_gain * angle[ROLL] / 16;
                servo[1] += (-cfg.gimbal_pitch_gain) * angle[PITCH] / 16 + cfg.gimbal_roll_gain * angle[ROLL] / 16;
            }
            else
            {
                servo[0] += cfg.gimbal_pitch_gain * angle[PITCH] / 16;
                servo[1] += cfg.gimbal_roll_gain * angle[ROLL]  / 16;
            }
        }
        servo[0] = constrain(servo[0], cfg.servoConf[0].min, cfg.servoConf[0].max);
        servo[1] = constrain(servo[1], cfg.servoConf[1].min, cfg.servoConf[1].max);
    }

    if (cfg.gimbal_flags & GIMBAL_FORWARDAUX)
    {
        int offset = 0;
        if (feature(FEATURE_SERVO_TILT))
            offset = 2;
        for (i = 0; i < 4; i++)     // isn't clean !!
            pwmWriteServo(i + offset, rcData[AUX1 + i]);
    }

    if (feature(FEATURE_LED) && (cfg.LED_Type == 1))
    {
        if (feature(FEATURE_SERVO_TILT))
            pwmWriteServo(2, LED_Value);
        else
            pwmWriteServo(0, LED_Value);
    }

    maxMotor = motor[0];
    for (i = 1; i < numberMotor; i++) if (motor[i] > maxMotor) maxMotor = motor[i];
    for (i = 0; i < numberMotor; i++)
    {
        if (maxMotor > cfg.maxthrottle) motor[i] -= maxMotor - cfg.maxthrottle;    // this is a way to still have good gyro corrections if at least one motor reaches its max.
        motor[i] = constrain(motor[i], cfg.minthrottle, cfg.maxthrottle);
        if ((rcData[THROTTLE]) < cfg.mincheck)
        {
            if (!feature(FEATURE_MOTOR_STOP)) motor[i] = cfg.minthrottle;
            else motor[i] = cfg.mincommand;
        }
        if (!f.ARMED) motor[i] = cfg.mincommand;
    }
}
