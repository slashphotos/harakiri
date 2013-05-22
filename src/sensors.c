#include "board.h"
#include "mw.h"

uint16_t calibratingA = 0;          // the calibration is done is the main loop. Calibrating decreases at each cycle down to 0, then we enter in a normal mode.
uint16_t calibratingG = 0;
uint16_t acc_1G = 256;              // this is the 1G measured acceleration.

extern uint16_t InflightcalibratingA;
extern int16_t  AccInflightCalibrationArmed;
extern uint16_t AccInflightCalibrationMeasurementDone;
extern uint16_t AccInflightCalibrationSavetoEEProm;
extern uint16_t AccInflightCalibrationActive;
extern uint16_t batteryWarningVoltage;
extern uint8_t  batteryCellCount;

sensor_t acc;                         // acc access functions
sensor_t gyro;                        // gyro access functions
baro_t   baro;                        // barometer access functions
uint8_t  accHardware = ACC_DEFAULT;   // which accel chip is used/detected

#ifdef FY90Q
// FY90Q analog gyro/acc
void sensorsAutodetect(void)
{
    adcSensorInit(&acc, &gyro);
}
#else
// AfroFlight32 i2c sensors
void sensorsAutodetect(void)
{
    int16_t deg, min;
    drv_adxl345_config_t acc_params;
    bool haveMpu6k = false;
    bool havel3g4200d = false;

    // Autodetect gyro hardware. We have MPU3050 or MPU6050.
    if (mpu6050Detect(&acc, &gyro)) {
        // this filled up  acc.* struct with init values
        haveMpu6k = true;
    } else if (l3g4200dDetect(&gyro)) {
        havel3g4200d = true;
    } else if (!mpu3050Detect(&gyro)) {
        // if this fails, we get a beep + blink pattern. we're doomed, no gyro or i2c error.
        failureMode(3);
    }

    // Accelerometer. Fuck it. Let user break shit.
retry:
    switch (cfg.acc_hardware) {
        case 0: // autodetect
        case 1: // ADXL345
            acc_params.useFifo = false;
            acc_params.dataRate = 800; // unused currently
            if (adxl345Detect(&acc_params, &acc))
                accHardware = ACC_ADXL345;
            if (cfg.acc_hardware == ACC_ADXL345)
                break;
            ; // fallthrough
       case 2: // MPU6050
            if (haveMpu6k) {
                mpu6050Detect(&acc, &gyro); // yes, i'm rerunning it again.  re-fill acc struct
                accHardware = ACC_MPU6050;
                if (cfg.acc_hardware == ACC_MPU6050)
                    break;
            }
            ; // fallthrough
        case 3: // MMA8452
            if (mma8452Detect(&acc)) {
                accHardware = ACC_MMA8452;
                if (cfg.acc_hardware == ACC_MMA8452)
                    break;
            }
    }

    // Found anything? Check if user fucked up or ACC is really missing.
    if (accHardware == ACC_DEFAULT) {
        if (cfg.acc_hardware > ACC_DEFAULT) {
            // Nothing was found and we have a forced sensor type. Stupid user probably chose a sensor that isn't present.
            cfg.acc_hardware = ACC_DEFAULT;
            goto retry;
        } else {
            // We're really screwed
            sensorsClear(SENSOR_ACC);
        }
    }

#ifdef BARO                                          // Crashpilot Skip Baro & Mag on feature pass
    // Detect what pressure sensors are available. baro->update() is set to sensor-specific update function
	  if (!feature(FEATURE_PASS)){                     // Skip this if in "pass" mode
        if (!ms5611Detect(&baro)) {                  // ms5611 disables BMP085, and tries to initialize + check PROM crc. if this works, we have a baro
            if (!bmp085Detect(&baro)) {              // if both failed, we don't have anything
                sensorsClear(SENSOR_BARO);
            }
        }
	  } else sensorsClear(SENSOR_BARO);
#endif

    // Now time to init things, acc first
    if (sensors(SENSOR_ACC))
        acc.init();
    // this is safe because either mpu6050 or mpu3050 or lg3d20 sets it, and in case of fail, we never get here.
    gyro.init();

    // todo: this is driver specific :(
    if (havel3g4200d) {
        l3g4200dConfig(cfg.gyro_lpf);
    } else {
        if (!haveMpu6k)
            mpu3050Config(cfg.gyro_lpf);
    }

#ifdef MAG
		if (!feature(FEATURE_PASS)){                     // Skip this if in "pass" mode
        if (!hmc5883lDetect()) sensorsClear(SENSOR_MAG);
		} else sensorsClear(SENSOR_MAG);
#endif

    // calculate magnetic declination
    deg = cfg.mag_declination / 100;
    min = cfg.mag_declination % 100;
    magneticDeclination = ((float)deg + ((float)min / 60.0f)); // heading is in deg units no 0.1 deg shit
}
#endif

uint16_t batteryAdcToVoltage(uint16_t src){
    // calculate battery voltage based on ADC reading
    // result is Vbatt in 0.1V steps. 3.3V = ADC Vref, 4095 = 12bit adc, 110 = 11:1 voltage divider (10k:1k) * 10 for 0.1V
    return (((src) * 3.3f) / 4095) * cfg.vbatscale;
}

void batteryInit(void)
{
    uint32_t i;
    uint32_t voltage = 0;

    // average up some voltage readings
    for (i = 0; i < 32; i++) {
        voltage += adcGetChannel(ADC_BATTERY);
        delay(10);
    }

    voltage = batteryAdcToVoltage((uint16_t)(voltage / 32));

    // autodetect cell count, going from 2S..6S
    for (i = 2; i < 6; i++) {
        if (voltage < i * cfg.vbatmaxcellvoltage)
            break;
    }
    batteryCellCount = i;
    batteryWarningVoltage = i * cfg.vbatmincellvoltage; // 3.3V per cell minimum, configurable in CLI
}

// ALIGN_GYRO = 0,
// ALIGN_ACCEL = 1,
// ALIGN_MAG = 2
static void alignSensors(uint8_t type, int16_t *data)
{
    int i;
    int16_t tmp[3];

    // make a copy :(
    tmp[0] = data[0];
    tmp[1] = data[1];
    tmp[2] = data[2];

    for (i = 0; i < 3; i++) {
        int8_t axis = cfg.align[type][i];
        if (axis > 0)
            data[axis - 1] = tmp[i];
        else
            data[-axis - 1] = -tmp[i];
    }
}

static void ACC_Common(void)
{
    static int32_t a[3];
    int axis;

    if (calibratingA > 0) {
        for (axis = 0; axis < 3; axis++) {
            // Reset a[axis] at start of calibration
            if (calibratingA == 400)
                a[axis] = 0;
            // Sum up 400 readings
            a[axis] += accADC[axis];
            // Clear global variables for next reading
            accADC[axis] = 0;
            cfg.accZero[axis] = 0;
        }
        // Calculate average, shift Z down by acc_1G and store values in EEPROM at end of calibration
        if (calibratingA == 1) {
            cfg.accZero[ROLL] = a[ROLL] / 400;
            cfg.accZero[PITCH] = a[PITCH] / 400;
            cfg.accZero[YAW] = a[YAW] / 400 - acc_1G;       // for nunchuk 200=1G
            cfg.angleTrim[ROLL] = 0;
            cfg.angleTrim[PITCH] = 0;
            writeParams(1);      // write accZero in EEPROM
        }
        calibratingA--;
    }

    if (feature(FEATURE_INFLIGHT_ACC_CAL)) {
        static int32_t b[3];
        static int16_t accZero_saved[3] = { 0, 0, 0 };
        static int16_t angleTrim_saved[2] = { 0, 0 };
        // Saving old zeropoints before measurement
        if (InflightcalibratingA == 50) {
            accZero_saved[ROLL] = cfg.accZero[ROLL];
            accZero_saved[PITCH] = cfg.accZero[PITCH];
            accZero_saved[YAW] = cfg.accZero[YAW];
            angleTrim_saved[ROLL] = cfg.angleTrim[ROLL];
            angleTrim_saved[PITCH] = cfg.angleTrim[PITCH];
        }
        if (InflightcalibratingA > 0) {
            for (axis = 0; axis < 3; axis++) {
                // Reset a[axis] at start of calibration
                if (InflightcalibratingA == 50)
                    b[axis] = 0;
                // Sum up 50 readings
                b[axis] += accADC[axis];
                // Clear global variables for next reading
                accADC[axis] = 0;
                cfg.accZero[axis] = 0;
            }
            // all values are measured
            if (InflightcalibratingA == 1) {
                AccInflightCalibrationActive = 0;
                AccInflightCalibrationMeasurementDone = 1;
                toggleBeep = 2;      // buzzer for indicatiing the end of calibration
                // recover saved values to maintain current flight behavior until new values are transferred
                cfg.accZero[ROLL] = accZero_saved[ROLL];
                cfg.accZero[PITCH] = accZero_saved[PITCH];
                cfg.accZero[YAW] = accZero_saved[YAW];
                cfg.angleTrim[ROLL] = angleTrim_saved[ROLL];
                cfg.angleTrim[PITCH] = angleTrim_saved[PITCH];
            }
            InflightcalibratingA--;
        }
        // Calculate average, shift Z down by acc_1G and store values in EEPROM at end of calibration
        if (AccInflightCalibrationSavetoEEProm == 1) {      // the copter is landed, disarmed and the combo has been done again
            AccInflightCalibrationSavetoEEProm = 0;
            cfg.accZero[ROLL] = b[ROLL] / 50;
            cfg.accZero[PITCH] = b[PITCH] / 50;
            cfg.accZero[YAW] = b[YAW] / 50 - acc_1G;    // for nunchuk 200=1G
            cfg.angleTrim[ROLL] = 0;
            cfg.angleTrim[PITCH] = 0;
            writeParams(1);          // write accZero in EEPROM
        }
    }

    accADC[ROLL] -= cfg.accZero[ROLL];
    accADC[PITCH] -= cfg.accZero[PITCH];
    accADC[YAW] -= cfg.accZero[YAW];
}

void ACC_getADC(void)
{
    acc.read(accADC);
    // if we have CUSTOM alignment configured, user is "assumed" to know what they're doing
    if (cfg.align[ALIGN_ACCEL][0])
        alignSensors(ALIGN_ACCEL, accADC);
    else
        acc.align(accADC);

    ACC_Common();
}

#ifdef BARO
void Baro_update(void)
{
	  static int32_t BaroSpikeTab[5];                                   // CRASHPILOT SPIKEFILTER Vars Start
    int32_t extmp;
    uint8_t rdy;
	  uint8_t sortidx;
	  uint8_t maxsortidx;                                               // CRASHPILOT SPIKEFILTER Vars End
    static uint32_t baroDeadline = 0;
		static uint32_t LastBaroTime;		
  	static uint8_t state = 0;
    int32_t pressure;
	  uint32_t acttime;
	  float PressureTMP;

  	newbaroalt = 0;                                                   // Reset Newbarovalue since it's iterative and not interrupt driven it's OK
    if (micros() < baroDeadline) return;
    switch (state) {
        case 0:
            baro.start_ut();                                          // Temperature Conversion start
            state++;
            baroDeadline =  micros() + baro.ut_delay - 1;
            break;
        case 1:                                                       // Crashpilot: reduced one case
            baro.get_ut();                                            // Readout Temp
            baro.start_up();                                          // Pressure Conversion start
            state++;
            baroDeadline =  micros() + baro.up_delay - 1;
    				break;
        case 2:
            baro.get_up();                                            // Readout Pressure
 				    pressure = baro.calculate();
  			 	  acttime = micros();                                       // Do timestuff
        	  BaroDeltaTime = acttime-LastBaroTime;                     // Real timediff of Barodata
	          LastBaroTime = acttime;
            baroDeadline = acttime + baro.repeat_delay -1;            // Don't take Spikefilter into account for delay
            extmp = pressure;                                         // CRASHPILOT SPIKEFILTER START
            BaroSpikeTab[4] = extmp;                                  // feed both ends of array with new data
            BaroSpikeTab[0] = extmp;                                  // feed both ends of array with new data
            rdy = 0;
  	        maxsortidx = 4;
            while(rdy == 0){
              rdy = 1;
              for (sortidx = 0; sortidx<maxsortidx;sortidx++){
                extmp = BaroSpikeTab[sortidx];
                if (extmp > BaroSpikeTab[sortidx+1]) {
                  BaroSpikeTab[sortidx] = BaroSpikeTab[sortidx+1];
                  BaroSpikeTab[sortidx+1] = extmp;
                  rdy = 0;
                }
              }
             maxsortidx --;
            }
            pressure = BaroSpikeTab[2];                               // CRASHPILOT SPIKEFILTER END
            PressureTMP = (float)pressure / 101325.0f;
            BaroAlt = (1.0f-pow(PressureTMP, 0.190295f))*4433000.0f;  // centimeter
            state = 0;
				    newbaroalt = 1;
    			  break;
    }
}
#endif /* BARO */

typedef struct stdev_t
{
    float m_oldM, m_newM, m_oldS, m_newS;
    int m_n;
} stdev_t;

static void devClear(stdev_t *dev)
{
    dev->m_n = 0;
}

static void devPush(stdev_t *dev, float x)
{
    dev->m_n++;
    if (dev->m_n == 1) {
        dev->m_oldM = dev->m_newM = x;
        dev->m_oldS = 0.0f;
    } else {
        dev->m_newM = dev->m_oldM + (x - dev->m_oldM) / dev->m_n;
        dev->m_newS = dev->m_oldS + (x - dev->m_oldM) * (x - dev->m_newM);
        dev->m_oldM = dev->m_newM;
        dev->m_oldS = dev->m_newS;
    }
}

static float devVariance(stdev_t *dev)
{
    return ((dev->m_n > 1) ? dev->m_newS / (dev->m_n - 1) : 0.0f);
}

static float devStandardDeviation(stdev_t *dev)
{
    return sqrtf(devVariance(dev));
}

static void GYRO_Common(void)
{
    int axis;
    static int16_t previousGyroADC[3] = { 0, 0, 0 };
    static int32_t g[3];
    static stdev_t var[3];

    if (calibratingG > 0) {
        for (axis = 0; axis < 3; axis++) {
            // Reset g[axis] at start of calibration
            if (calibratingG == 1000) {
                g[axis] = 0;
                devClear(&var[axis]);
            }
            // Sum up 1000 readings
            g[axis] += gyroADC[axis];
            devPush(&var[axis], gyroADC[axis]);
            // Clear global variables for next reading
            gyroADC[axis] = 0;
            gyroZero[axis] = 0;
            if (calibratingG == 1) {
                float dev = devStandardDeviation(&var[axis]);
                // check deviation and startover if idiot was moving the model
                if (cfg.moron_threshold && dev > cfg.moron_threshold) {
                    calibratingG = 1000;
                    devClear(&var[0]);
                    devClear(&var[1]);
                    devClear(&var[2]);
                    g[0] = g[1] = g[2] = 0;
                    continue;
                }
                gyroZero[axis] = g[axis] / 1000;
                blinkLED(10, 15, 1);
            }
        }
        calibratingG--;
    }
    for (axis = 0; axis < 3; axis++) {
        gyroADC[axis] -= gyroZero[axis];
        //anti gyro glitch, limit the variation between two consecutive readings
        gyroADC[axis] = constrain(gyroADC[axis], previousGyroADC[axis] - 800, previousGyroADC[axis] + 800);
        previousGyroADC[axis] = gyroADC[axis];
    }
}

void Gyro_getADC(void)
{
    // range: +/- 8192; +/- 2000 deg/sec
    gyro.read(gyroADC);
    // if we have CUSTOM alignment configured, user is "assumed" to know what they're doing
    if (cfg.align[ALIGN_GYRO][0])
        alignSensors(ALIGN_GYRO, gyroADC);
    else
        gyro.align(gyroADC);

    GYRO_Common();
}

#ifdef SONAR

void Sonar_init(void) 
{
    hcsr04_init(sonar_rc78);
    sensorsSet(SENSOR_SONAR);
    sonarAlt = 0;
}

void Sonar_update(void) 
{
    hcsr04_get_distance(&sonarAlt);
}

#endif

#ifdef MAG
static float magCal[3] = { 1.0, 1.0, 1.0 };     // gain for each axis, populated at sensor init
static uint8_t magInit = 0;

static void Mag_getRawADC(void){
    hmc5883lRead(magADC);
    // Default mag orientation is -2, -3, 1 or
    // no way? is THIS finally the proper orientation?? (by GrootWitBaas)
    // magADC[ROLL] = rawADC[2]; // X
    // magADC[PITCH] = -rawADC[0]; // Y
    // magADC[YAW] = -rawADC[1]; // Z
    alignSensors(ALIGN_MAG, magADC);
}

void Mag_init(void){
    // initialize and calibration. turn on led during mag calibration (calibration routine blinks it)
    LD1_ON();
    hmc5883lInit(magCal);
    LD1_OFF();
    magInit = 1;
}

int Mag_getADC(void){
    static uint32_t t, tCal = 0;
    static int16_t magZeroTempMin[3];
    static int16_t magZeroTempMax[3];
    uint32_t axis;

    if ((int32_t)(currentTime - t) < 0) return 0;                 // each read is spaced by 100ms
    t = currentTime + 100000;
    Mag_getRawADC();                                              // Read mag sensor
    magADC[ROLL]  = magADC[ROLL]  * magCal[ROLL];
    magADC[PITCH] = magADC[PITCH] * magCal[PITCH];
    magADC[YAW]   = magADC[YAW]   * magCal[YAW];

    if (f.CALIBRATE_MAG) {
        tCal = t;
        for (axis = 0; axis < 3; axis++) {
            cfg.magZero[axis]    = 0;
            magZeroTempMin[axis] = magADC[axis];
            magZeroTempMax[axis] = magADC[axis];
        }
        f.CALIBRATE_MAG = 0;
    }

    if (magInit) {                                                // we apply offset only once mag calibration is done
        magADC[ROLL]  -= cfg.magZero[ROLL];
        magADC[PITCH] -= cfg.magZero[PITCH];
        magADC[YAW]   -= cfg.magZero[YAW];
    }

    if (tCal != 0) {
        if ((t - tCal) < 60000000) {                             // 60s: you have 60s to turn the multi in all directions
            LED0_TOGGLE;
            for (axis = 0; axis < 3; axis++) {
                if (magADC[axis] < magZeroTempMin[axis]) magZeroTempMin[axis] = magADC[axis];
                if (magADC[axis] > magZeroTempMax[axis]) magZeroTempMax[axis] = magADC[axis];
            }
        } else {
            tCal = 0;
            for (axis = 0; axis < 3; axis++) cfg.magZero[axis] = (magZeroTempMin[axis] + magZeroTempMax[axis]) / 2; // Calculate offsets
            writeParams(1);
        }
    }
    return 1;
}
#endif


/*
// My changes
#ifdef MAG
static float magCal[3] = { 1.0, 1.0, 1.0 };     // gain for each axis, populated at sensor init
static uint8_t magInit = 0;

static void Mag_getRawADC(void)
{
    hmc5883lRead(magADC);
    alignSensors(ALIGN_MAG, magADC);
}

void Mag_init(void)
{
    uint8_t calibration_gain = 0x60;                                  // HMC5883
    uint8_t numAttempts = 0, good_count = 0, axis;
    bool success = false;
    uint16_t expected_x = 766, expected_yz = 713;                     // default values for HMC5883
    float gain_multiple = 660.0f / 1090.0f;                           // adjustment for runtime vs calibration gain
    float cal[3];
	
    hmc5883lInit();                                                   // initial calibration
	  for (axis = 0; axis < 3; axis++) magCal[axis] = 0;
    while (success == false && numAttempts < 20 && good_count < 5) {
        numAttempts++;                                                // record number of attempts at initialisation
        hmc5883lCal(calibration_gain);                                // enter calibration mode
        delay(100);
        Mag_getRawADC();
        delay(10);
        cal[0] = fabsf(expected_x / (float)magADC[ROLL]);
        cal[1] = fabsf(expected_yz / (float)magADC[PITCH]);
        cal[2] = fabsf(expected_yz / (float)magADC[ROLL]);
        if (cal[0] > 0.7f && cal[0] < 1.3f
         && cal[1] > 0.7f && cal[1] < 1.3f
				 && cal[2] > 0.7f && cal[2] < 1.3f){
            good_count++;
            for (axis = 0; axis < 3; axis++) magCal[axis] += cal[axis];
        }
    }
    if (good_count >= 5) {
		    for (axis = 0; axis < 3; axis++) magCal[axis] = (magCal[axis] * gain_multiple) / (float)good_count;
        success = true;
    } else for (axis = 0; axis < 3; axis++) magCal[axis] = 1.0f;          // best guess
    hmc5883lFinishCal();
    magInit = 1;
}

void Mag_getADC(void)
{
    static uint32_t t, tCal = 0;
    static int16_t magZeroTempMin[3], magZeroTempMax[3];
	  float magRAW[3];                                                     // Hacky
    uint8_t axis;
    
    if ((int32_t)(currentTime - t) < 0) return;                          // each read is spaced by 100ms
    t = currentTime + 100000;
    // Read mag sensor
    Mag_getRawADC();
    for (axis = 0; axis < 3; axis++){                                    // Hacky
			magRAW[axis] = (float)magADC[axis]* magCal[axis];
			magADC[axis] = (int16_t)magRAW[axis];
		}
	  if (f.CALIBRATE_MAG){
        tCal = t;
        for (axis = 0; axis < 3; axis++) {
            cfg.magZero[axis] = 0;
            magZeroTempMin[axis] = magADC[axis];
            magZeroTempMax[axis] = magADC[axis];
        }
        f.CALIBRATE_MAG = 0;
    }
    if (magInit) {                                                       // we apply offset only once mag calibration is done
        for (axis = 0; axis < 3; axis++){                                // Hacky
			      magRAW[axis]-= (float)cfg.magZero[axis];
			      magADC[axis] = (int16_t)magRAW[axis];
		    }			                                                           // keep float and do 16bit copy for min/max or output
    }
    if (tCal != 0) {
        if ((t - tCal) < 60000000) {                                     // 60s: you have 60s to turn the multi in all directions
            LED0_TOGGLE;
            for (axis = 0; axis < 3; axis++) {
                if (magADC[axis] < magZeroTempMin[axis]) magZeroTempMin[axis] = magADC[axis];
                if (magADC[axis] > magZeroTempMax[axis]) magZeroTempMax[axis] = magADC[axis];
            }
        } else {
            tCal = 0;
            for (axis = 0; axis < 3; axis++) cfg.magZero[axis] = (magZeroTempMin[axis] + magZeroTempMax[axis]) / 2; // Calculate offsets
            writeParams(1);
        }
    }
}
#endif
*/
