#include "board.h"
#include "mw.h"

int16_t         gyroADC[3], accADC[3], accSmooth[3];
float           magADCfloat[3];
int16_t         acc_25deg = 0;
int32_t         BaroAlt;
int16_t         sonarAlt;         // to think about the unit
int32_t         EstAlt;           // in cm
int32_t         AltHold;
int16_t         vario;            // variometer in cm/s
int16_t         BaroP;
int16_t         BaroI;
int16_t         BaroD;
uint8_t         newbaroalt;
bool            GroundAltInitialized;
static int32_t  GroundAlt;
static uint16_t ACCDeltaTime;
uint16_t        BaroDeltaTime;
float           actual_speed[2];
float           ACCDeltaTimeINS = 0;
static float    VelUP = 0;

// **************
// gyro+acc IMU
// **************
int16_t gyroData[3] = { 0, 0, 0 };
int16_t gyroZero[3] = { 0, 0, 0 };
int16_t angle[2] = { 0, 0 };      // absolute angle inclination in multiple of 0.1 degree    180 deg = 1800

static void getEstimatedAttitude(void);

void imuInit(void)
{
    acc_25deg = acc_1G * 0.423f;

#ifdef MAG                        // if mag sensor is enabled, use it
    if (sensors(SENSOR_MAG))
        Mag_init();
#endif
}

void computeIMU(void)
{
    uint32_t axis;
    static int16_t gyroADCprevious[3] = { 0, 0, 0 };
    int16_t gyroADCp[3];
    int16_t gyroADCinter[3];
    static uint32_t timeInterleave = 0;
    static int16_t gyroYawSmooth = 0;

#define GYRO_INTERLEAVE

    if (sensors(SENSOR_ACC))
    {
        ACC_getADC();
        getEstimatedAttitude();
    }

    Gyro_getADC();

    for (axis = 0; axis < 3; axis++)
    {
#ifdef GYRO_INTERLEAVE
        gyroADCp[axis] = gyroADC[axis];
#else
        gyroData[axis] = gyroADC[axis];
#endif
        if (!sensors(SENSOR_ACC))
            accADC[axis] = 0;
    }
    timeInterleave = micros();
    annexCode();
#ifdef GYRO_INTERLEAVE
    if ((micros() - timeInterleave) > 650)
    {
        annex650_overrun_count++;
    }
    else
    {
        while ((micros() - timeInterleave) < 650);  // empirical, interleaving delay between 2 consecutive reads
    }

    Gyro_getADC();
    for (axis = 0; axis < 3; axis++)
    {
        gyroADCinter[axis] = gyroADC[axis] + gyroADCp[axis];
        // empirical, we take a weighted value of the current and the previous values
        gyroData[axis] = (gyroADCinter[axis] + gyroADCprevious[axis]) / 3;
        gyroADCprevious[axis] = gyroADCinter[axis] / 2;
        if (!sensors(SENSOR_ACC))
            accADC[axis] = 0;
    }
#endif

    if (feature(FEATURE_GYRO_SMOOTHING))
    {
        static uint8_t Smoothing[3] = { 0, 0, 0 };
        static int16_t gyroSmooth[3] = { 0, 0, 0 };
        if (Smoothing[0] == 0)
        {
            // initialize
            Smoothing[ROLL] = (cfg.gyro_smoothing_factor >> 16) & 0xff;
            Smoothing[PITCH] = (cfg.gyro_smoothing_factor >> 8) & 0xff;
            Smoothing[YAW] = (cfg.gyro_smoothing_factor) & 0xff;
        }
        for (axis = 0; axis < 3; axis++)
        {
            gyroData[axis] = (int16_t)(((int32_t)((int32_t)gyroSmooth[axis] * (Smoothing[axis] - 1)) + gyroData[axis] + 1 ) / Smoothing[axis]);
            gyroSmooth[axis] = gyroData[axis];
        }
    }
    else if (cfg.mixerConfiguration == MULTITYPE_TRI)
    {
        gyroData[YAW] = (gyroYawSmooth * 2 + gyroData[YAW]) / 3;
        gyroYawSmooth = gyroData[YAW];
    }
}

// **************************************************
// Simplified IMU based on "Complementary Filter"
// Inspired by http://starlino.com/imu_guide.html
//
// adapted by ziss_dm : http://www.multiwii.com/forum/viewtopic.php?f=8&t=198
//
// The following ideas was used in this project:
// 1) Rotation matrix: http://en.wikipedia.org/wiki/Rotation_matrix
// 2) Small-angle approximation: http://en.wikipedia.org/wiki/Small-angle_approximation
// 3) C. Hastings approximation for atan2()
// 4) Optimization tricks: http://www.hackersdelight.org/
//
// Currently Magnetometer uses separate CF which is used only
// for heading approximation.
//
// Modified: 19/04/2011  by ziss_dm
// Version: V1.1
//
// code size deduction and tmp vector intermediate step for vector rotation computation: October 2011 by Alex
// **************************************************

//******  advanced users settings *******************

/* Set the Low Pass Filter factor for Magnetometer */
/* Increasing this value would reduce Magnetometer noise (not visible in GUI), but would increase Magnetometer lag time*/
/* Comment this if  you do not want filter at all.*/
/* Default WMC value: n/a*/

/* Set the Gyro Weight for Gyro/Magnetometer complementary filter */
/* Increasing this value would reduce and delay Magnetometer influence on the output of the filter*/
/* Default WMC value: n/a*/
#define GYR_CMPFM_FACTOR 200.0f

//****** end of advanced users settings *************

#define INV_GYR_CMPF_FACTOR   (1.0f / ((float)cfg.gyro_cmpf_factor + 1.0f))
#define INV_GYR_CMPFM_FACTOR  (1.0f / (GYR_CMPFM_FACTOR + 1.0f))

#define GYRO_SCALE ((1998 * M_PI)/((32767.0f / 4.0f ) * 180.0f * 1000000.0f))     // 32767 / 16.4lsb/dps for MPU3000

/*
#define GYRO_SCALE (4  / 14.375 * PI / 180.0 / 1000000.0) // ITG3200 14.375 LSB/(deg/s) and we ignore the last 2 bits
#define GYRO_SCALE (4  / 16.4   * PI / 180.0 / 1000000.0) // MPU6050 and MPU3050 16.4 LSB/(deg/s) and we ignore the last 2 bits
#define GYRO_SCALE ((4.0f * PI * 70.0f)/(1000.0f * 180.0f * 1000000.0f))  // L3g4200 70 LSB/(deg/s)
#define GYRO_SCALE (1.0f/200e6f) // WMP - LOL
*/

// #define GYRO_SCALE ((2380 * M_PI)/((32767.0f / 4.0f ) * 180.0f * 1000000.0f))     //should be 2279.44 but 2380 gives better result (ITG-3200)
// +-2000/sec deg scale
//#define GYRO_SCALE ((200.0f * PI)/((32768.0f / 5.0f / 4.0f ) * 180.0f * 1000000.0f) * 1.5f)
// +- 200/sec deg scale
// 1.5 is emperical, not sure what it means
// should be in rad/sec

typedef struct fp_vector
{
    float X;
    float Y;
    float Z;
} t_fp_vector_def;

typedef union
{
    float A[3];
    t_fp_vector_def V;
} t_fp_vector;

t_fp_vector EstG;

// Rotate Estimated vector(s) with small angle approximation, according to the gyro data
void rotateV(struct fp_vector *v, float *delta)
{
    struct fp_vector v_tmp = *v;
    float mat[3][3];                                      // This does a  "proper" matrix rotation using gyro deltas without small-angle approximation
    float cosx, sinx, cosy, siny, cosz, sinz;
    float coszcosx, coszcosy, sinzcosx, coszsinx, sinzsinx;

    cosx = cosf(-delta[PITCH]);
    sinx = sinf(-delta[PITCH]);
    cosy = cosf(delta[ROLL]);
    siny = sinf(delta[ROLL]);
    cosz = cosf(delta[YAW]);
    sinz = sinf(delta[YAW]);

    coszcosx = cosz * cosx;
    coszcosy = cosz * cosy;
    sinzcosx = sinz * cosx;
    coszsinx = sinx * cosz;
    sinzsinx = sinx * sinz;

    mat[0][0] = coszcosy;
    mat[0][1] = sinz * cosy;
    mat[0][2] = -siny;
    mat[1][0] = (coszsinx * siny) - sinzcosx;
    mat[1][1] = (sinzsinx * siny) + (coszcosx);
    mat[1][2] = cosy * sinx;
    mat[2][0] = (coszcosx * siny) + (sinzsinx);
    mat[2][1] = (sinzcosx * siny) - (coszsinx);
    mat[2][2] = cosy * cosx;

    v->X = v_tmp.X * mat[0][0] + v_tmp.Y * mat[1][0] + v_tmp.Z * mat[2][0];
    v->Y = v_tmp.X * mat[0][1] + v_tmp.Y * mat[1][1] + v_tmp.Z * mat[2][1];
    v->Z = v_tmp.X * mat[0][2] + v_tmp.Y * mat[1][2] + v_tmp.Z * mat[2][2];
}

static int16_t _atan2f(float y, float x)
{
    return (int16_t)(atan2f(y, x) * (1800.0f / M_PI));
}

static void getEstimatedAttitude(void)
{
    static t_fp_vector EstM;
    static float    accLPF[3],accLPFINS[3];
    static uint32_t previousT;
    float           scale, deltaGyroAngle[3];
    float           rollRAD, pitchRAD, cr, sr, cp, sp, Xh, Yh;
    float           cy, sy, spcy, spsy, acc_south, acc_west, acc_up;
    float           tmp0, tmp1, tmp2, tmp3;
    uint8_t         axis;
    int32_t         accMag = 0;
    uint32_t        currentT = micros();

    ACCDeltaTime    = currentT - previousT;
    scale           = (float)ACCDeltaTime * GYRO_SCALE;
    ACCDeltaTimeINS = (float)ACCDeltaTime * 0.000001f;
    previousT       = currentT;
    if (cfg.acc_ins_lpf == 0)    cfg.acc_ins_lpf    = 1;                 // Just to be safe
    if (cfg.acc_lpf_factor == 0) cfg.acc_lpf_factor = 1;                 // Just to be safe
	  tmp0 = (1.0f / (float)cfg.acc_ins_lpf);                              // tmp0 = 1.0f/1 = 1
    tmp1 = 1.0f - tmp0;                                                  // tmp1 = 0
    tmp2 = (1.0f / (float)cfg.acc_lpf_factor);
    tmp3 = 1.0f - tmp2;
    for (axis = 0; axis < 3; axis++)
    {
        deltaGyroAngle[axis] = (float)gyroADC[axis] * scale;
        accLPFINS[axis] = accLPFINS[axis] * tmp1 + accADC[axis] * tmp0;
        accLPF[axis]    = accLPF[axis]    * tmp3 + accADC[axis] * tmp2;
        accSmooth[axis] = accLPF[axis];                                  // Store float in 16Bit Shit accSmooth. Dont know why that is neccessary
        accMag += (int32_t)accSmooth[axis] * accSmooth[axis];
    }
    accMag = accMag * 100 / ((int32_t)acc_1G * acc_1G);
    rotateV(&EstG.V, deltaGyroAngle);
    if (sensors(SENSOR_MAG)) rotateV(&EstM.V, deltaGyroAngle);
    if (abs(accSmooth[ROLL]) < acc_25deg && abs(accSmooth[PITCH]) < acc_25deg && accSmooth[YAW] > 0) f.SMALL_ANGLES_25 = 1;
    else f.SMALL_ANGLES_25 = 0;

    // Apply complimentary filter (Gyro drift correction)
    // If accel magnitude >1.4G or <0.6G and ACC vector outside of the limit range => we neutralize the effect of accelerometers in the angle estimation.
    // To do that, we just skip filter, as EstV already rotated by Gyro
    if ((36 < accMag && accMag < 196) || f.SMALL_ANGLES_25)
    {
        for (axis = 0; axis < 3; axis++)
            EstG.A[axis] = (EstG.A[axis] * (float)cfg.gyro_cmpf_factor + accSmooth[axis]) * INV_GYR_CMPF_FACTOR;
    }
    if (sensors(SENSOR_MAG)) for (axis = 0; axis < 3; axis++) EstM.A[axis] = (EstM.A[axis] * GYR_CMPFM_FACTOR + magADCfloat[axis]) * INV_GYR_CMPFM_FACTOR;
    angle[ROLL] = _atan2f(EstG.V.X, EstG.V.Z);
    angle[PITCH] = -asinf(EstG.V.Y / -sqrtf(EstG.V.X * EstG.V.X + EstG.V.Y * EstG.V.Y + EstG.V.Z * EstG.V.Z)) * (1800.0f / M_PI);             // This hack removes gimbal lock (sorta) on pitch, // * 1.0227 is an arbitrary value to get 90 Deg

////////////////////////////////////////////////////////////////////////////////////
// This is the core of the INS stuff
    rollRAD   = (float)angle[ROLL]   * RADX10;                         // Some common values
    pitchRAD  = -(float)angle[PITCH] * RADX10;
    cr        = cosf(rollRAD);
    sr        = sinf(rollRAD);
    cp        = cosf(pitchRAD);
    sp        = sinf(pitchRAD);
    TiltValue = cp * cr;                                               // 1.0 is horizontal 0 is vertical, minus is upsidedown
#ifdef MAG
    if (sensors(SENSOR_MAG) && cfg.mag_calibrated == 1)
    {
        tmp0 = EstM.A[0];                                              // CORRECT MAG TILT COMPENSATION
        tmp1 = EstM.A[1];
        tmp2 = EstM.A[2];
        Xh   = tmp1 * cp + tmp0 * sr * sp + tmp2 * cr * sp;            // Xh = EstM.A[1] * cp + EstM.A[0] * sr * sp + EstM.A[2] * cr * sp;
        Yh   = tmp0 * cr - tmp2 * sr;                                  // Yh = EstM.A[0] * cr - EstM.A[2] * sr;
        tmp0 = atan2f(-Yh,Xh) * 180.0f / M_PI;                         // Get rad to Degree
        heading = tmp0 + magneticDeclination;                          // Add Declination
        if (heading > 180.0f) heading = heading - 360.0f;              // Wrap to -180 0 +180 Degree
        else if (heading < -180.0f) heading = heading + 360.0f;
    }
    else heading = 0;                                                  // if no mag or not calibrated do bodyframe below
    tmp0      = heading * RADX;                                        // Do GPS INS rotate ACC X/Y to earthframe no centrifugal comp. yet
    cy        = cosf(tmp0);
    sy        = sinf(tmp0);
    cos_yaw_x = cy;                                                    // Store for general use
    sin_yaw_y = sy;                                                    // Store for general use
    spcy      = sp * cy;
    spsy      = sp * sy;
    tmp3      = acc_1G;
    tmp0      = (float)accLPFINS[0] / tmp3;                            // Reference to Gravity before rotation for better accuracy
    tmp1      = (float)accLPFINS[1] / tmp3;
    tmp2      = (float)accLPFINS[2] / tmp3;
    acc_up    = ((-sp) * tmp1 + (sr * cp) * tmp0 + TiltValue * tmp2)-1;// -1G That works perfect for althold, but normalized Vector is better for GPS, dunno why
    tmp0      = accLPFINS[0];
    tmp1      = accLPFINS[1];
    tmp2      = accLPFINS[2];
    tmp3      = sqrtf(tmp0 * tmp0 + tmp1 * tmp1 + tmp2 * tmp2);        // Normalize ACCvector so the gps ins works
    if (tmp3 == 0.0f) tmp3 = 1;                                        // In that case all tmp must be zero so div by 1 is ok
    tmp0      = tmp0 / tmp3;
    tmp1      = tmp1 / tmp3;
    tmp2      = tmp2 / tmp3;
    acc_south = (cp * cy) * tmp1 + (sr * spcy - cr * sy) * tmp0 + (sr * sy + cr * spcy)  * tmp2;
    acc_west  = (cp * sy) * tmp1 + (cr * cy + sr * spsy) * tmp0 + (-sr * cy + cr * spsy) * tmp2;
    tmp3      = 980.665f * ACCDeltaTimeINS;                            // vel factor for normalized output tmp3      = (9.80665f * (float)ACCDeltaTime) / 10000.0f;
    tmp2      = constrain(TiltValue, 0.5f, 1.0f) * tmp3;               // Empirical reduction of hightdrop in forward flight
    actual_speed[LAT] = actual_speed[LAT] - acc_south * tmp3;          // Positive when moving North cm/sec when no MAG this is speed to the front
    actual_speed[LON] = actual_speed[LON] - acc_west  * tmp3;          // Positive when moving East cm/sec when no MAG this is speed to the right
    VelUP     = VelUP + acc_up * tmp2;                                 // Positive when moving Up
#endif
}

#ifdef BARO
///////////////////////////////////////////////
//Crashpilot1000 Mod getEstimatedAltitude ACC//
///////////////////////////////////////////////

#define VarioTabsize 8
#define BaroTabsize 5
void getEstimatedAltitude(bool purge)
{
    static uint8_t  Vidx,Bidx;
    static int32_t  LastEstAltBaro;
    static uint32_t IniTimer = 0;
    static float    VarioTab[VarioTabsize];
    static int32_t  BaroTab[BaroTabsize];
    static float    accalt;
    uint8_t         i, ThrAngle;
    int32_t         tmp32;
    int32_t         EstAltBaro;
    float           BaroClimbRate;
    float           fltmp;

    if (!GroundAltInitialized && newbaroalt == 1)                // Do init here
    {
      if (IniTimer == 0) IniTimer = currentTime + 2000000;       // 2 Secs of warmup
       else
       {
           if (currentTime >=IniTimer)
           {
               GroundAlt = BaroAlt;                              // Single measurement is sufficient after warmup
               GroundAltInitialized = true;
           }
       }
    }

    if (purge)                                                   // Do fast Brainwash, keep Vario
    {
        tmp32 = BaroAlt - GroundAlt;      
        for (i = 0; i < BaroTabsize; i++) BaroTab[i] = tmp32;    // Set BaroTab to current Alt, don't alter Variotab
        LastEstAltBaro = tmp32;
        accalt = (float)tmp32;
    }
    
    ThrAngle = TiltValue * 100.0f;
    if (ThrAngle > 100) ThrAngle = 100;
    accalt = accalt + VelUP * ACCDeltaTimeINS;
    
    if (newbaroalt!=0)                                           // MS Baro Timecheck 27ms // BMP085 Timecheck 26ms debug[0] = BaroDeltaTime/1000;
    {
        BaroTab[Bidx] = BaroAlt - GroundAlt;                     //BaroAlt - GroundAlt Get EstAltBaro
        Bidx++;
        if (Bidx == BaroTabsize) Bidx = 0;
        tmp32 = 0;
        for (i = 0; i < BaroTabsize; i++) tmp32 = tmp32 + BaroTab[i];
        EstAltBaro = tmp32 / BaroTabsize;
        fltmp = 1000000/(float)BaroDeltaTime;                    // BaroDeltaTime in us
        VarioTab[Vidx] = (float)constrain(EstAltBaro - LastEstAltBaro,-127,127) * fltmp;
        Vidx++;                                                  // Baro Climbrate
        if (Vidx == VarioTabsize) Vidx = 0;
        LastEstAltBaro = EstAltBaro;
        fltmp = 0;
        for (i = 0; i < VarioTabsize; i++) fltmp = fltmp + VarioTab[i];
        BaroClimbRate = fltmp /(float)VarioTabsize;              // BaroClimbRate in cm/sec // + is up // 27ms * 37 = 999ms
        VelUP = VelUP * cfg.accz_vel_cf + BaroClimbRate * (1.0f - cfg.accz_vel_cf);
        accalt = accalt * cfg.accz_alt_cf + (float)EstAltBaro * (1.0f - cfg.accz_alt_cf);
        if (cfg.nazedebug == 1)
        {
            debug[0] = EstAltBaro*10;
            debug[1] = accalt*10;
            debug[2] = BaroClimbRate;
            debug[3] = VelUP;
        }
    }
    EstAlt = accalt;
    vario  = VelUP;
    BaroP = 0;
    BaroI = 0;
    BaroD = 0;
    if (ThrAngle < 40 || TiltValue < 0) return;                   // Don't do BaroPID if copter too tilted// EstAlt & Baroclimbrate are always done :)
    BaroP = ((AltHold-EstAlt)*cfg.P8[PIDALT])/200;
    BaroI = (VelUP*cfg.I8[PIDALT])/50;                            //  BaroI = constrain(ClimbRate*conf.I8[PIDALT],-150,150);
    BaroD = (((int16_t)cfg.D8[PIDALT]) * (100 - ThrAngle))/25;
}
#endif

/*
float applyDeadbandFloat(float value, float deadband)
{
    if (abs(value) < deadband) value = 0;
     else if (value > 0) value = value - deadband;
           else if (value < 0) value = value + deadband;
    return value;
}

/////// GPS INS TESTCODE
//  Testcode
    static uint32_t previous5HzT = 0;
		flthead = 0;                                                // if no mag do bodyframe below
//  Testcode
    int16_t knob = constrain(rcData[AUX3]-1000,0,1000);
		float  knobbi= (float)knob * 0.001f;
		debug[0] = knobbi * 1000;
	  if (currentT > previous5HzT + 200000){
        previous5HzT = currentT;
		    VelNorth     = VelNorth * knobbi;
        VelEast      = VelEast  * knobbi;
		}
		debug[1] = VelNorth;
		debug[2] = VelEast;
/////// GPS INS TESTCODE
*/
