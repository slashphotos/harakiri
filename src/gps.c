#include "board.h"
#include "mw.h"

//#define Earthradius           637100078.5d                         // Average Earthradius in cm (from 637813700cm ellipsoid to a sphere)
//#define MagicEarthNumber       1.1119494f                          // Based on average Earthradius pi/180 * Earthradius / 10^7
//#define MagicEarthNumber       1.1113175f                            // used by apm "INERTIALNAV_LATLON_TO_CM"

#define MagicEarthNumber         1.11163345f                         // OWN Earth number does correct projection!!

// They are defined in mw.h
// #define LAT  0
// #define LON  1
// #define GPS_Y 0
// #define GPS_X 1

#define MTK_BAUD_RATE_57600			"$PMTK251,57600*2C\r\n"
#define MTK_SBAS_INTEGRITYMODE	"$PMTK319,1*24\r\n"
#define MTK_OUTPUT_5HZ					"$PMTK220,200*2C\r\n"
#define MTK_NAVTHRES_OFF      	"$PMTK397,0*23\r\n"
#define MTK_SBAS_ON							"$PMTK313,1*2E\r\n"
#define MTK_WAAS_ON           	"$PMTK301,2*2E\r\n"
#define MTK_SET_BINARY					"$PGCMD,16,0,0,0,0,0*6A\r\n"

const  uint32_t init_speed[5] = { 9600, 19200, 38400, 57600, 115200 };
static const uint8_t ubloxInit[] = {
     0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x05, 0x00, 0xFF, 0x19,                                  // disable all default NMEA messages
     0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x03, 0x00, 0xFD, 0x15,
     0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x01, 0x00, 0xFB, 0x11,
     0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x00, 0x00, 0xFA, 0x0F,
     0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x02, 0x00, 0xFC, 0x13,
     0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x04, 0x00, 0xFE, 0x17,
     0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x02, 0x01, 0x0E, 0x47,                                  // set POSLLH MSG rate
     0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x03, 0x01, 0x0F, 0x49,                                  // set STATUS MSG rate
     0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x06, 0x01, 0x12, 0x4F,                                  // set SOL MSG rate
     0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x12, 0x01, 0x1E, 0x67,                                  // set VELNED MSG rate
     0xB5, 0x62, 0x06, 0x16, 0x08, 0x00, 0x03, 0x07, 0x03, 0x00, 0x51, 0x08, 0x00, 0x00, 0x8A, 0x41,    // set WAAS to EGNOS
     0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0xC8, 0x00, 0x01, 0x00, 0x01, 0x00, 0xDE, 0x6A                 // set rate to 5Hz
};

typedef struct PID_PARAM_ {
    float kP;
    float kI;
    float kD;
    float Imax;
} PID_PARAM;

typedef struct PID_ {
    float integrator;          // integrator value
    float last_input;          // last input for derivative
    float last_derivative;     // last derivative for low-pass filter
    float output;
    float derivative;
} PID;

/*
typedef struct ProjectBrake {
    int32_t  StartLat;         // Input
	  int32_t  StartLon;
	  uint32_t TimeStampStart;
	  float    Deceleration;     // Wanted Deceleration in cm/s*s
	  int32_t  ProjStopLat;      // Output
		int32_t  ProjStopLon;
		uint32_t ProjTimeStampStop;
} ProjectBrake;
*/

static bool      check_missed_wp(void);
static void      GPS_NewData(uint16_t c);
static void      GPS_distance_cm_bearing(int32_t * lat1, int32_t * lon1, int32_t * lat2, int32_t * lon2, uint32_t * dist, int32_t * bearing);
static void      GPS_calc_longitude_scaling();
static void      GPS_calc_velocity(void);
static void      GPS_calc_location_error(int32_t * target_lat, int32_t * target_lng, int32_t * gps_lat, int32_t * gps_lng);
static void      GPS_calc_posholdCrashpilot(bool useabsolutepos);
static void      GPS_calc_posholdAPM(void);
static void      GPS_calc_nav_rate(int16_t max_speed);
static int16_t   GPS_calc_desired_speed(void);
static bool      GPS_newFrame(char c);
static bool      GPS_NMEA_newFrame(char c);
static bool      GPS_MTK_newFrame(uint8_t data); 
static bool      GPS_UBLOX_newFrame(uint8_t data);
static bool      UBLOX_parse_gps(void);
static void      SpikefilterGPSOutput(bool reset);
static void      gpsPrint(const char *str);
float            wrap_18000(float error);
static int32_t   wrap_36000(int32_t angle);
//static int16_t   RCDeadband(int16_t rcvalue, int16_t rcdead);
//static float     SpecialSQRT(float value);
static float     get_P(float error, struct PID_PARAM_* pid);
static float     get_I(float error, float* dt, struct PID_* pid, struct PID_PARAM_* pid_param);
static float     get_D(float input, float* dt, struct PID_* pid, struct PID_PARAM_* pid_param);

// NAV & PH PID Variables
static PID_PARAM posholdPID_PARAM;
static PID_PARAM poshold_ratePID_PARAM;
static PID_PARAM navPID_PARAM;

static PID       posholdPID[2];
static PID       poshold_ratePID[2];
static PID       navPID[2];

// INS & Core Variables
extern float     actual_speed[2], ACCDeltaTimeINS;
static float     dTnav;                   // Delta Time in milliseconds for navigation computations, updated with every good GPS read
static int32_t   Real_GPS_coord[2];
static float     rate_error[2];           // The difference between the desired rate of travel and the actual rate of travel
static float     error[2];                // updated after GPS read - 5-10hz
static float     INSTotalSpeed;      // In cm/s
static uint32_t  TimestampNewGPSdata;     // Crashpilot in micros
static int16_t   maxbank100;              // Maximum GPS Tiltangle

// PH Variables
static bool      PHcompletelySettled;
static float     MinAngleFactor;

// NAVIGATION & Crosstrack Variables
static int32_t   target_bearing;          // target_bearing is where we should be heading
static uint32_t  wp_distance;
static float     waypoint_speed_gov;      // used for slow speed wind up when start navigation;
static int32_t   nav_bearing;             // This is the angle from the copter to the "next_WP" location  with the addition of Crosstrack error in degrees * 100
static int16_t   nav_takeoff_bearing;     // saves the bearing at takeof (1deg = 1) used to rotate to takeoff direction when arrives at home
static int32_t   original_target_bearing; // deg*100, The original angle to the next_WP when the next_WP was set Also used to check when we pass a WP
static int16_t   crosstrack_error;        // The amount of angle correction applied to target_bearing to bring the copter back on its optimum path

// Earth / Location constants
static float     OneCmTo[2];              // Moves one cm in Gps coords
static float     CosLatScaleLon;          // this is used to offset the shrinking longitude as we go towards the poles

void GPS_alltime(void){
    float vel_forward_cms, vel_right_cms;
    static uint32_t phmovetimer = 0, phsettletimer = 0;
	  uint32_t dist;
    int32_t  dir;
    int16_t  speed;
	  uint8_t  PHoverride;
	
    if (f.GPS_FIX && GPS_numSat >= 5){                                             // Do gps stuff with at least 5 Sats

	      if (!f.ARMED) f.GPS_FIX_HOME = 0;
        if (!f.GPS_FIX_HOME && f.ARMED) GPS_reset_home_position();
        dTnav = ACCDeltaTimeINS;                                                   // Time in Secs (0.xxxx sec) from ACC read, that is important
	      dTnav = min(dTnav, 1.0f);
        GPS_calc_velocity();                                                       // Heart of the gps ins (and getEstimatedAttitude()), called every time
        if (!f.GPS_FIX_HOME) {                                                     // Do relative to home stuff for gui etc
            GPS_distanceToHome = 0;
            GPS_directionToHome = 0;
        } else {
            GPS_distance_cm_bearing(&GPS_coord[LAT], &GPS_coord[LON], &GPS_home[LAT], &GPS_home[LON], &dist, &dir);
            GPS_distanceToHome = dist / 100;
            GPS_directionToHome = dir / 100;
	      }

        if (DoingGPS()){
            switch (nav_mode) {
							
			          case NAV_MODE_POSHOLD:
								     PHoverride = 0;
// Will be probably deleted just for testing
								    if (cfg.gps_phmove_speed != 0){                                // Do PH MOVE SHIT
											  PHcompletelySettled = true;                                // This is always true with phmove
								        if (currentTime >= phmovetimer) {                          // 10Hz Loop
								            phmovetimer  = currentTime + 100000;
								            if (rcCommand[PITCH] !=0 || rcCommand[ROLL] !=0) {
											          vel_forward_cms = (float)constrain(rcCommand[PITCH], -450, 450) * cfg.gps_phmove_speed;
                                vel_right_cms   = (float)constrain(rcCommand[ROLL] , -450, 450) * cfg.gps_phmove_speed;
                                GPS_WP[LON] += (vel_right_cms   * cos_yaw_x + vel_forward_cms * sin_yaw_y) * OneCmTo[LON];
                                GPS_WP[LAT] += (vel_forward_cms * cos_yaw_x - vel_right_cms   * sin_yaw_y) * OneCmTo[LAT];
												    }
										    }
										}
// Will be probably deleted just for testing

										if (cfg.gps_phmove_speed == 0){                                // This is not done with "else" because the part above will probably be removed
                        if (RCDeadband(rcCommand[PITCH], cfg.phdeadband) !=0 || RCDeadband(rcCommand[ROLL], cfg.phdeadband) !=0){
												    PHcompletelySettled = false;
                            GPS_WP[LON]         = GPS_coord[LON];                  // To think about the original APM controller
                            GPS_WP[LAT]         = GPS_coord[LAT];					         // To think about the original APM controller
                            PHoverride          = 1;
												}

										    if (!PHcompletelySettled && INSTotalSpeed < cfg.gps_ph_settlespeed && phsettletimer == 0){
											    phsettletimer = currentTime + ((uint32_t)cfg.gps_ph_settletime * 1000);
										    } else phsettletimer = 0;

										    if (!PHcompletelySettled && phsettletimer != 0 && currentTime >= phsettletimer){
												    PHcompletelySettled = true;
                            GPS_WP[LON] = GPS_coord[LON];
                            GPS_WP[LAT] = GPS_coord[LAT];
												}
 									  }

                    GPS_calc_location_error(&GPS_WP[LAT], &GPS_WP[LON], &GPS_coord[LAT], &GPS_coord[LON]);
 				            if (cfg.gps_ph_apm == 0) GPS_calc_posholdCrashpilot(PHcompletelySettled); // Only use absolute Position if copter settled, otherwise do relative PH (just brake)
							       else GPS_calc_posholdAPM();
										if (PHoverride == 1){                                                     // This is done so no influence of PH during move
										    nav[0] = 0;                                                           // but let the PID Controllers run in the Background, just testing
											  nav[1] = 0;											
										}
                break;

						    case NAV_MODE_CIRCLE:
						    // *** DO SOME SERIOUS SHIT HERE
								//		GPS_distance_cm_bearing(&GPS_coord[LAT], &GPS_coord[LON], &GPS_WP[LAT], &GPS_WP[LON], &wp_distance, &target_bearing);
                //    GPS_calc_location_error(&GPS_WP[LAT], &GPS_WP[LON], &GPS_coord[LAT], &GPS_coord[LON]);	
						    break;

						    case NAV_MODE_WP:
                case NAV_MODE_RTL:
								    GPS_distance_cm_bearing(&GPS_coord[LAT], &GPS_coord[LON], &GPS_WP[LAT], &GPS_WP[LON], &wp_distance, &target_bearing);
                    GPS_calc_location_error(&GPS_WP[LAT], &GPS_WP[LON], &GPS_coord[LAT], &GPS_coord[LON]);
								
					          speed = GPS_calc_desired_speed();
                    GPS_calc_nav_rate(speed);                                      // use error as the desired rate towards the target Desired output is in nav_lat and nav_lon where 1deg inclination is 100

								    if (cfg.nav_controls_heading == 1 && wp_distance > 200){       // Tail control only update beyond 2 m
                        if (cfg.nav_tail_first == 1) magHold = wrap_18000(nav_bearing - 18000) / 100;
                         else magHold = nav_bearing / 100;
                    }

								    if ((wp_distance <= cfg.gps_wp_radius) || check_missed_wp()){  // if yes switch to poshold mode
                        if (cfg.nav_rtl_lastturn == 1 && nav_mode == NAV_MODE_RTL) magHold = nav_takeoff_bearing;  // rotates it's head to takeoff direction if wanted
							          nav_mode = NAV_MODE_POSHOLD;
                        wp_mode  = WP_STATUS_DONE;
											  PHcompletelySettled = true;
								    } else wp_mode = WP_STATUS_NAVIGATING;
                break;
            }                                                                      // END Switch nav_mode
						
            if (cfg.nav_slew_rate == 0) SpikefilterGPSOutput(false);               // Do Spikefilter all the time when no slew rate wanted. Reset is "false"
        }                                                                          // END of gps calcs i.e navigating
		}else GPS_reset_nav();                                                         // END GPS_numSat >= 5
}

void GPS_NewData(uint16_t c){             // Called by uart2Init interrupt
    static int32_t  LatSpikeTab[5];
    static int32_t  LonSpikeTab[5];
  	int32_t  extmp;                       // Crashpilot Spikefilter
    uint8_t  rdy;                         // Crashpilot Spikefilter
		uint8_t  sortidx;                     // Crashpilot Spikefilter
		uint8_t  maxsortidx;                  // Crashpilot Spikefilter
	
    if (GPS_newFrame(c)) {
        if (GPS_update == 1) GPS_update = 0; // Some strange telemetry shit
         else GPS_update = 1;
			  extmp = Real_GPS_coord[LAT];      //  Crashpilot Spikefilter: Do it with every GPS Data only use when less 7 sats
        LatSpikeTab[4] = extmp;
        LatSpikeTab[0] = extmp;
        extmp = Real_GPS_coord[LON];
        LonSpikeTab[4] = extmp;
        LonSpikeTab[0] = extmp;
        rdy = 0; maxsortidx=4;
        while(rdy == 0){
            rdy = 1;
            for (sortidx = 0; sortidx<maxsortidx;sortidx++){
                extmp = LatSpikeTab[sortidx];
                if (extmp > LatSpikeTab[sortidx+1]) {LatSpikeTab[sortidx] = LatSpikeTab[sortidx+1]; LatSpikeTab[sortidx+1] = extmp; rdy = 0;}
             }
            maxsortidx --;
        }
        rdy = 0; maxsortidx=4;
        while(rdy == 0){
            rdy = 1;
            for (sortidx = 0; sortidx<maxsortidx;sortidx++){
                extmp = LonSpikeTab[sortidx];
                if (extmp > LonSpikeTab[sortidx+1]) {LonSpikeTab[sortidx] = LonSpikeTab[sortidx+1]; LonSpikeTab[sortidx+1] = extmp; rdy = 0;}
             }
            maxsortidx --;
        }
				if(!f.GPS_FIX){                                                    // Don't fill spikefilter with pure shit
			      for (sortidx = 0; sortidx < 5; sortidx++) {                    // Abuse sortidx and clear filter
		            LatSpikeTab[sortidx] = 0;
                LonSpikeTab[sortidx] = 0;
		        }
				}
        if (GPS_numSat < 7){                                               // Use Spikefiltervalues with less than 7 Sats
            if (LatSpikeTab[2] != 0 && LonSpikeTab[2] != 0){               // Use filtervalues if they are not zero
						    Real_GPS_coord[LAT] = LatSpikeTab[2];
                Real_GPS_coord[LON] = LonSpikeTab[2];
						}
				}
        TimestampNewGPSdata = millis();                                    // Set time of Data arrival in MS
    }
}

////////////////////////////////////////////////////////////////////////////////////
// Calculate our current speed vector from gps&acc position data
// This is another important part of the gps ins
static void GPS_calc_velocity(void){                                                // actual_speed[GPS_Y] y_GPS_speed positve = Up (NORTH) // actual_speed[GPS_X] x_GPS_speed positve = Right (EAST)
    static int32_t  Last_Real_GPS_coord[2];
	  static uint32_t LastTimestampNewGPSdata;
	  static float    SmoothActualSpeed[2];
	  static float    LagCompensation[2];
	  static float    GPSmovementAdder[2];
	  static bool     INSusable;
    float           Real_GPS_speed[2];
    float           gpsHz, tmp0;	
    uint32_t        RealGPSDeltaTime;
	  uint8_t         i;
	
	  if (CosLatScaleLon == 0.0f) GPS_calc_longitude_scaling();                       // Init CosLatScaleLon if not already done to avoid div by zero etc..
    RealGPSDeltaTime = TimestampNewGPSdata - LastTimestampNewGPSdata;               // RealGPSDeltaTime in ms! NOT us!
	  LastTimestampNewGPSdata = TimestampNewGPSdata;
	  if (RealGPSDeltaTime != 0)                                                      // New GPS Data?
		{
        INSusable = false;                                                          // Set INS to ununsable
			  if (RealGPSDeltaTime < 400)                                                 // In Time? 2,5Hz-XXHz
				{
					  INSusable = true;                                                       // INS is alive
					  gpsHz = 1000.0f/(float)RealGPSDeltaTime;                                // Set it, try to filter below
					  if (RealGPSDeltaTime >  80 && RealGPSDeltaTime < 120) gpsHz = 10.0f;    // 10Hz Data 100ms filter out timejitter
					  if (RealGPSDeltaTime > 180 && RealGPSDeltaTime < 220) gpsHz = 5.0f;     //  5Hz Data 200ms
					  if (RealGPSDeltaTime > 230 && RealGPSDeltaTime < 270) gpsHz = 4.0f;     //  4Hz Data 250ms
					  tmp0 = MagicEarthNumber * gpsHz;
            Real_GPS_speed[LON] = (float)(Real_GPS_coord[LON] - Last_Real_GPS_coord[LON]) * tmp0 * CosLatScaleLon ; // cm/s
            Real_GPS_speed[LAT] = (float)(Real_GPS_coord[LAT] - Last_Real_GPS_coord[LAT]) * tmp0;                   // cm/s
            for (i = 0; i < 2; i++){
					      Last_Real_GPS_coord[i] = Real_GPS_coord[i];
		            actual_speed[i]        = actual_speed[i] * cfg.gps_ins_vel + Real_GPS_speed[i] * (1.0f - cfg.gps_ins_vel); // CF: GPS Correction
                if (cfg.gps_proj_smooth == 0.0f) SmoothActualSpeed[i] = actual_speed[i];
								 else SmoothActualSpeed[i] = SmoothActualSpeed[i] * cfg.gps_proj_smooth + actual_speed[i] * (1.0f - cfg.gps_proj_smooth);
							  LagCompensation[i]     = SmoothActualSpeed[i] * cfg.gps_lag * OneCmTo[i];
						    GPSmovementAdder[i]    = 0;
				    }
				}
		}                                                                               // End of X Hz Loop
		if ((millis() - TimestampNewGPSdata) > 500) INSusable = false;                  // INS is NOT OK, too long (500ms) no correction
    if (INSusable){
        for (i = 0; i < 2; i++){
			      GPSmovementAdder[i] = GPSmovementAdder[i] + (actual_speed[i]   * ACCDeltaTimeINS * OneCmTo[i]);
            GPS_coord[i]        = Real_GPS_coord[i]   + LagCompensation[i] + GPSmovementAdder[i];
				}
				INSTotalSpeed = sqrtf(SmoothActualSpeed[LAT] * SmoothActualSpeed[LAT] + SmoothActualSpeed[LON] * SmoothActualSpeed[LON]); // This is better than GPS speed because we only want XY and not XYZ Speed
		} else {
			  GPS_reset_nav();                                                            // Ins is fucked, reset stuff
				INSTotalSpeed = 0;                                                          // Reset the Rest
			  for (i = 0; i < 2; i++){
            GPS_coord[i]           = Real_GPS_coord[i];
			      Last_Real_GPS_coord[i] = Real_GPS_coord[i];
            actual_speed[i]        = 0;
            SmoothActualSpeed[i]   = 0;					
					  LagCompensation[i]     = 0;
					  GPSmovementAdder[i]    = 0;
		    }
		}
}

////////////////////////////////////////////////////////////////////////////////////
// Get distance between two points in cm Get bearing from pos1 to pos2, returns an 1deg = 100 precision
static void GPS_distance_cm_bearing(int32_t * lat1, int32_t * lon1, int32_t * lat2, int32_t * lon2, uint32_t * dist, int32_t * bearing){
	  float dLatsq, dLonsq, dLat, dLon;
	  if (*lat2 != 0 && *lat1 != 0 && *lon2 != 0 && *lon1 != 0){                     // Crashpilot Errorcheck
			  if (CosLatScaleLon == 0.0f) GPS_calc_longitude_scaling();
			  dLat = *lat2 - *lat1;                                                      // difference of latitude in 1/10 000 000 degrees
        dLon = *lon2 - *lon1;
			  dLon = dLon * CosLatScaleLon;                                              // dlon scale
			  dLatsq = dLat * dLat;                                                      // square values
			  dLonsq = dLon * dLon;
        *dist = sqrtf(dLatsq + dLonsq) * MagicEarthNumber;                         // dist in cm
				*bearing = 9000.0f + atan2f(-dLat, dLon) * 5729.57795f;                    // Convert the output radians to 100xdeg
        if (*bearing < 0) *bearing += 36000;
		} else {                                                                       // Error!
			  *dist = 0;
			  *bearing = 0;
		}
}

////////////////////////////////////////////////////////////////////////////////////
// Calculate a location error between two gps coordinates. Crashpilot distance error in CM now!
static void GPS_calc_location_error(int32_t *target_lat, int32_t *target_lng, int32_t *gps_lat, int32_t *gps_lng){
	  if (CosLatScaleLon == 0.0f) GPS_calc_longitude_scaling();
		if (*target_lng != 0 && *target_lat != 0 && *gps_lng != 0 && *gps_lat != 0){
  	    error[LON] = (float)(*target_lng - *gps_lng) * MagicEarthNumber * CosLatScaleLon;   // X Error in cm not lon!
        error[LAT] = (float)(*target_lat - *gps_lat) * MagicEarthNumber;                    // Y Error in cm not lat!
		} else error[LON] = error[LAT] = 0;
}

////////////////////////////////////////////////////////////////////////////////////
// PID based GPS navigation functions
// Author : EOSBandi some functions pimped by Crashpilot
////////////////////////////////////////////////////////////////////////////////////
// Sets the waypoint to navigate, reset neccessary variables and calculate initial values
void GPS_set_next_wp(int32_t *lat, int32_t *lon){
	  float tmp0,tmp1;
    GPS_WP[LAT] = *lat;
    GPS_WP[LON] = *lon;
	
    GPS_calc_longitude_scaling();
    GPS_distance_cm_bearing(&GPS_coord[LAT], &GPS_coord[LON], &GPS_WP[LAT], &GPS_WP[LON], &wp_distance, &target_bearing);
    GPS_calc_location_error(&GPS_WP[LAT], &GPS_WP[LON], &GPS_coord[LAT], &GPS_coord[LON]);
    nav_bearing = target_bearing;
	  original_target_bearing = target_bearing;
    waypoint_speed_gov = (float)cfg.nav_speed_min;
	  crosstrack_error = 0;
    WP_Target_Alt = 0;
    WP_Desired_Climbrate = 0;

	  switch(nav_mode){
			  case NAV_MODE_POSHOLD:
				    PHcompletelySettled = false;
        break;			
			  case NAV_MODE_RTL:
			      WP_Fastcorner = false;                                            // This means: Dont break when approaching WP
        break;
        case NAV_MODE_WP:
		        tmp0 = (float)(WP_Target_Alt - EstAlt);                           // tmp0 = hightdifference in cm.  + is up
            tmp1 = ((float)wp_distance / (float)cfg.nav_speed_max) * 1.2f;    // tmp1 = Estimated Traveltime + 20% // Div Zero not possible
            if (tmp1 == 0.0f) WP_Desired_Climbrate = 0;                       // Avoid Div Zero
 	  		     else WP_Desired_Climbrate = tmp0 / tmp1;                         // Climbrate in cm/s
        break;
			  case NAV_MODE_CIRCLE:                                                 // Set some constants
//        Maybe some shit here later
//        Project a gps point x cm ahead the copter nose will look like this:
//        Project[LON] = Current[LON]+ (int32_t) ((Project_forward_cm * sin_yaw_y) * OneCmTo[LON]);
//        Project[LAT] = Current[LAT]+ (int32_t) ((Project_forward_cm * cos_yaw_x) * OneCmTo[LAT]);
        break;
		}
}

////////////////////////////////////////////////////////////////////////////////////
// Crashpilot NEW PH Logic
// #define GPS_X 1 // #define GPS_Y 0
// #define LON   1 // #define LAT   0
// VelEast;   // 1 // VelNorth;  // 0
// 0 is NICK part  // 1 is ROLL part
// actual_speed[GPS_Y] = VelNorth;
static void GPS_calc_posholdCrashpilot(bool useabsolutepos){
	  uint8_t axis;
  	float d, target_speed, maxbank100new, tmp0, tmp1;
	
    for (axis = 0; axis < 2; axis++) {
	      if (actual_speed[axis] !=0 && !useabsolutepos){                            // Calculate tiltanglerestriction at overspeed based on totalspeed
            tmp0 = (float)cfg.gps_ph_targetsqrt / actual_speed[axis];              // do something like sqrt(10/x) 10 is the unproblematic speed for max tiltangle
            tmp1 = constrain(sqrtf(abs(tmp0)), MinAngleFactor, 1.0f);              // tmp1 contains now the factor for maximal angle
            maxbank100new = tmp1 * (float)maxbank100;
        } else maxbank100new = maxbank100;                                         // dont restrict further at 0 speed

			  if (useabsolutepos) target_speed = get_P(error[axis], &posholdPID_PARAM);  // Calculate Rate Error
			   else target_speed = 0;
        rate_error[axis] = target_speed - actual_speed[axis];
			  rate_error[axis] = constrain(rate_error[axis], -1000, 1000);               // +- 10m/s
        nav[axis]        = get_P(rate_error[axis],                                 &poshold_ratePID_PARAM) +  //try negative for I? Because it just works like shit
                           get_I(rate_error[axis], &dTnav, &poshold_ratePID[axis], &poshold_ratePID_PARAM);
                       d = get_D(rate_error[axis], &dTnav, &poshold_ratePID[axis], &poshold_ratePID_PARAM);
                       d = constrain(d, -2000, 2000);
    		nav[axis]        = constrain(nav[axis] + d, -maxbank100new, maxbank100new);
        navPID[axis].integrator = poshold_ratePID[axis].integrator;
    }
}

static void GPS_calc_posholdAPM(void){
    float d;
    float target_speed;
    uint8_t axis;
    for (axis = 0; axis < 2; axis++) {
        target_speed = get_P(error[axis], &posholdPID_PARAM);              // calculate desired speed from lon error
        rate_error[axis] = target_speed - actual_speed[axis];              // calc the speed error
        nav[axis] = get_P(rate_error[axis], &poshold_ratePID_PARAM) +
                    get_I(rate_error[axis] + error[axis], &dTnav, &poshold_ratePID[axis], &poshold_ratePID_PARAM);
        d = get_D(error[axis], &dTnav, &poshold_ratePID[axis], &poshold_ratePID_PARAM);
        d = constrain(d, -2000, 2000);
        if (abs(actual_speed[axis]) < 50) d = 0;                           // get rid of noise
        nav[axis] +=d;
        nav[axis] = constrain(nav[axis], -maxbank100, maxbank100);
        navPID[axis].integrator = poshold_ratePID[axis].integrator;
    }
}

////////////////////////////////////////////////////////////////////////////////////
// Calculate the desired nav_lat and nav_lon for distance flying such as RTH
static void GPS_calc_nav_rate(int16_t max_speed){
    float trig[2];
    float temp;
    uint8_t axis;
    if (abs(wrap_18000(target_bearing - original_target_bearing)) < 4500) {          // If we are too far off or too close we don't do track following
        float temp = (target_bearing - original_target_bearing) * RADX100;
        crosstrack_error = sinf(temp) * wp_distance;                                 // Meters we are off track line
        nav_bearing = target_bearing + constrain(crosstrack_error, -3000, 3000);
        nav_bearing = wrap_36000(nav_bearing);
    } else nav_bearing = target_bearing;
	  temp = (9000l - nav_bearing) * RADX100;                                          // nav_bearing includes crosstrack
	  trig[GPS_X] = cosf(temp);
    trig[GPS_Y] = sinf(temp);
	  for (axis = 0; axis < 2; axis++) {
        rate_error[axis] = (trig[axis] * (float)max_speed) - actual_speed[axis];
        rate_error[axis] = constrain(rate_error[axis], -1000, 1000);
        nav[axis] = get_P(rate_error[axis],                        &navPID_PARAM) +  // P + I + D
                    get_I(rate_error[axis], &dTnav, &navPID[axis], &navPID_PARAM) +
                    get_D(rate_error[axis], &dTnav, &navPID[axis], &navPID_PARAM);
        nav[axis] = constrain(nav[axis], -maxbank100, maxbank100);
        poshold_ratePID[axis].integrator = navPID[axis].integrator;
    }
}

static int16_t GPS_calc_desired_speed(void){
    int16_t max_speed = cfg.nav_speed_max;
    if (!WP_Fastcorner) max_speed = min(max_speed, wp_distance / 3);        // Arducopter 2.8.1 do div by 3
     else max_speed = min(max_speed, wp_distance);
    if (max_speed > waypoint_speed_gov) {
        waypoint_speed_gov += (100.0f * dTnav);                             // increase speed
        max_speed = waypoint_speed_gov;
    }
	  max_speed = constrain(max_speed, cfg.nav_speed_min, cfg.nav_speed_max); // Put output in desired range
    return max_speed;
}

////////////////////////////////////////////////////////////////////////////////////
// ***   GPS INIT   ***
////////////////////////////////////////////////////////////////////////////////////
void gpsInit(uint32_t baudrate){                              // Called in Main
  uint8_t i;
	uint32_t timeout;

  GPS_set_pids();
	GPS_Present = 0;
	delay(2000);                                                // let it init
  timeout = micros()+10000000;					                      // 10 sec timeout
	while (GPS_Present == 0){                                   // Repeat while no GPS Data
		if (micros()>timeout) break;                              // Stop that after timeout
    uart2Init(baudrate, GPS_NewData, false);
  	switch (cfg.gps_type){ 	                                  // GPS_NMEA = 0, GPS_UBLOX = 1, GPS_MTK16 = 2, GPS_MTK19 = 3, GPS_UBLOX_DUMB = 4
      case 0:                                                 // GPS_NMEA
      break;
      case 1:                                                 // GPS_UBLOX
        for (i = 0; i < 5; i++) {
          uart2ChangeBaud(init_speed[i]);
          delay(200);
          switch (baudrate){
            case 19200:
              gpsPrint("$PUBX,41,1,0003,0001,19200,0*23\r\n");
            break;
            case 38400:
              gpsPrint("$PUBX,41,1,0003,0001,38400,0*26\r\n");
            break;
            case 57600:
              gpsPrint("$PUBX,41,1,0003,0001,57600,0*2D\r\n");
            break;
            case 115200:
              gpsPrint("$PUBX,41,1,0003,0001,115200,0*1E\r\n");
            break;
          }
        }
        uart2ChangeBaud(baudrate);
        delay(200);
        for (i = 0; i < sizeof(ubloxInit); i++){
          delay(6);
          uart2Write(ubloxInit[i]);                                      // send ubx init binary
        }
      break;
      case 2:                                                            // GPS_MTK16
      case 3:                                                            // GPS_MTK19
        for (i = 0; i < 5; i++){
          uart2ChangeBaud(init_speed[i]);
          delay(200);
          gpsPrint(MTK_BAUD_RATE_57600);    
        }
 		    uart2ChangeBaud(57600);
        delay(200);
	      gpsPrint(MTK_SET_BINARY);
        delay(200);
        gpsPrint(MTK_OUTPUT_5HZ);
        delay(200);
        gpsPrint(MTK_SBAS_INTEGRITYMODE);
        delay(200);
        gpsPrint(MTK_NAVTHRES_OFF);
        delay(200);
        gpsPrint(MTK_SBAS_ON);
        delay(200);
        gpsPrint(MTK_WAAS_ON);
      break;
      case 4:                                                             // GPS_UBLOX_DUMB = 4
      break;				
    }
    delay(1000);
  }
  if (GPS_Present) sensorsSet(SENSOR_GPS);                                // Do we get Data? Is GPS present?
  maxbank100     = (int16_t)cfg.gps_maxangle * 100;                       // Initialize some values here
	MinAngleFactor = (float)cfg.gps_minanglepercent * 0.01f;
}

static void gpsPrint(const char *str){
    while (*str) {
        if (cfg.gps_type == 1) delay(6);                 // GPS_NMEA = 0, GPS_UBLOX = 1, GPS_MTK16 = 2, GPS_MTK19 = 3,
        uart2Write(*str);            
        str++;
    }
    while (!uart2TransmitEmpty());                       // wait to send all
    delay(30);
}

////////////////////////////////////////////////////////////////////////////////////
// ***   Utilities   ***
////////////////////////////////////////////////////////////////////////////////////
void GPS_set_pids(void){                                              // Get the relevant P I D values and set the PID controllers
#define POSHOLD_RATE_IMAX      20                                     // degrees
    posholdPID_PARAM.kP   = (float)cfg.P8[PIDPOS] /  100.0f;          // Original Scale 
    posholdPID_PARAM.kI   = (float)cfg.I8[PIDPOS] /  100.0f;          // Not used but initialized
    posholdPID_PARAM.kD   =	(float)cfg.D8[PIDPOS] / 1000.0f;          // Not used but initialized
    posholdPID_PARAM.Imax = POSHOLD_RATE_IMAX * 100;
		
		if (cfg.gps_ph_apm == 0){
        poshold_ratePID_PARAM.kP = (float)cfg.P8[PIDPOSR] /       5;  // Need more P
        poshold_ratePID_PARAM.kI = (float)cfg.I8[PIDPOSR] / 1000.0f;  // "I" is evil, leads to circeling
        poshold_ratePID_PARAM.kD = (float)cfg.D8[PIDPOSR] /  100.0f;  // Crashpilot needs bigger values, i think that is actually the real apm D scaling
        poshold_ratePID_PARAM.Imax = POSHOLD_RATE_IMAX * 100;
		} else {
        poshold_ratePID_PARAM.kP = (float)cfg.P8[PIDPOSR] /   10.0f;  //  Original Scale 
        poshold_ratePID_PARAM.kI = (float)cfg.I8[PIDPOSR] /  100.0f;
        poshold_ratePID_PARAM.kD = (float)cfg.D8[PIDPOSR] / 1000.0f;
        poshold_ratePID_PARAM.Imax = POSHOLD_RATE_IMAX * 100;
		}
    navPID_PARAM.kP = (float)cfg.P8[PIDNAVR] / 10.0f;
    navPID_PARAM.kI = (float)cfg.I8[PIDNAVR] / 100.0f;
    navPID_PARAM.kD = (float)cfg.D8[PIDNAVR] / 1000.0f;
    navPID_PARAM.Imax = POSHOLD_RATE_IMAX * 100;
}

static float get_P(float error, struct PID_PARAM_* pid){
    return error * pid->kP;
}

static float get_I(float error, float* dt, struct PID_* pid, struct PID_PARAM_* pid_param){
    pid->integrator += (error * pid_param->kI) * *dt;
    pid->integrator  = constrain(pid->integrator, -pid_param->Imax, pid_param->Imax);
    return pid->integrator;
}

static float get_D(float input, float* dt, struct PID_* pid, struct PID_PARAM_* pid_param){
    pid->derivative = (input - pid->last_input) / *dt;
		// Examples for _filter:
    // f_cut = 10 Hz -> _filter = 15.9155e-3 // f_cut = 15 Hz -> _filter = 10.6103e-3 // f_cut = 20 Hz -> _filter =  7.9577e-3
    // f_cut = 25 Hz -> _filter =  6.3662e-3 // f_cut = 30 Hz -> _filter =  5.3052e-3
	  float filter = 7.9577e-3;
    pid->derivative = pid->last_derivative + (*dt / (filter + *dt)) * (pid->derivative - pid->last_derivative);
	  pid->last_input = input;
    pid->last_derivative = pid->derivative;
    return pid_param->kD * pid->derivative;
}

static void reset_PID(struct PID_* pid){
    pid->integrator      = 0;
    pid->last_input      = 0;
    pid->last_derivative = 0;
}

void GPS_reset_home_position(void){
    if (f.GPS_FIX && GPS_numSat >= 5) {
			  GPS_calc_longitude_scaling();
        GPS_home[LAT] = Real_GPS_coord[LAT];
        GPS_home[LON] = Real_GPS_coord[LON];
        nav_takeoff_bearing = heading;              // save takeoff heading
        // Set ground altitude
        f.GPS_FIX_HOME = 1;
    }
}

void GPS_reset_nav(void){                                             //reset navigation (stop the navigation processor, and clear nav)
    uint8_t i;
    for (i = 0; i < 2; i++) {
			  error[i]        = 0;
        GPS_angle[i]    = 0;
        nav_rated[i]    = 0;
        nav[i]          = 0;
        reset_PID(&posholdPID[i]);
        reset_PID(&poshold_ratePID[i]);
        reset_PID(&navPID[i]);
    }
    waypoint_speed_gov = (float)cfg.nav_speed_min;
	  crosstrack_error = 0;
		WP_Fastcorner = false;
		SpikefilterGPSOutput(true);
}

bool DoingGPS(void){
    if (f.GPS_HOLD_MODE || f.GPS_HOME_MODE) return true;
	   else return false;
}

static bool check_missed_wp(void){
    float temp;
    temp = (float)(target_bearing - original_target_bearing);
    temp = wrap_18000(temp);
    return (abs(temp) > 10000); // we passed the waypoint by 100 degrees
}

static void GPS_calc_longitude_scaling(){
	  float rads = ((float)Real_GPS_coord[LAT] / 10000000.0f) * RADX;
    rads = fabs(rads);
    CosLatScaleLon = cosf(rads);                              // can only be 0 at 90 degree, perhaps at the poles?
	  if (CosLatScaleLon == 0) CosLatScaleLon = 0.001745328f;   // Avoid divzero (value is cos of 89.9 Degree)
    OneCmTo[LAT] = 1.0f / MagicEarthNumber;                   // Moves North one cm
    OneCmTo[LON] = 1.0f / MagicEarthNumber / CosLatScaleLon;  // Moves EAST  one cm
}

////////////////////////////////////////////////////////////////////////////////////
// Crashpilot Spikefilter the GPS Output so a higher slewrate is possible, spikefilter produces minimal lag if any
void SpikefilterGPSOutput(bool reset){
    static float nav0[5];
    static float nav1[5];
  	float        extmp;
	  uint8_t      i;

    if (reset) for (i = 0; i < 5; i++) nav0[i] = nav1[i] = 0;
	   else {
        uint8_t  rdy,sortidx,maxsortidx;
		    extmp =  nav[0];
        nav0[4] = extmp;
        nav0[0] = extmp;
		    extmp =  nav[1];
        nav1[4] = extmp;
        nav1[0] = extmp;
        rdy = 0; maxsortidx=4;
        while(rdy == 0){
            rdy = 1;
            for (sortidx = 0; sortidx<maxsortidx;sortidx++){
                extmp = nav0[sortidx];
                if (extmp > nav0[sortidx+1]) {nav0[sortidx] = nav0[sortidx+1]; nav0[sortidx+1] = extmp; rdy = 0;}
            }
            maxsortidx --;
        }
        rdy = 0; maxsortidx=4;
        while(rdy == 0){
            rdy = 1;
            for (sortidx = 0; sortidx<maxsortidx;sortidx++){
                extmp = nav1[sortidx];
                if (extmp > nav1[sortidx+1]) {nav1[sortidx] = nav1[sortidx+1]; nav1[sortidx+1] = extmp; rdy = 0;}
            }
            maxsortidx --;
        }
        nav[0] = nav0[2];
        nav[1] = nav1[2];
	  }
}

float wrap_18000(float error){
    while (error >  18000) error -= 36000;
    while (error < -18000) error += 36000;
    return error;
}

static int32_t wrap_36000(int32_t angle){
    while (angle > 36000) angle -= 36000;
    while (angle <     0) angle += 36000;
    return angle;
}

int16_t RCDeadband(int16_t rcvalue, uint8_t rcdead){
    if (abs(rcvalue) < rcdead) rcvalue = 0;
     else if (rcvalue > 0) rcvalue = rcvalue - (int16_t)rcdead;
           else rcvalue = rcvalue + (int16_t)rcdead;
    return rcvalue;
}

#define DIGIT_TO_VAL(_x)    (_x - '0')                            // This code is used for parsing NMEA data
uint32_t GPS_coord_to_degrees(char* s)
{
    char *p, *q;
    uint8_t deg = 0, min = 0;
    unsigned int frac_min = 0;
    int i;
    for (p = s; isdigit(*p); p++);                                // scan for decimal point or end of field
    q = s;
    while ((p - q) > 2) {                                         // convert degrees
        if (deg)
            deg *= 10;
        deg += DIGIT_TO_VAL(*q++);
    }
    while (p > q) {                                               // convert minutes
        if (min)
            min *= 10;
        min += DIGIT_TO_VAL(*q++);
    }
    if (*p == '.') {                                              // convert fractional minutes expect up to four digits, result is in ten-thousandths of a minute
        q = p + 1;
        for (i = 0; i < 4; i++) {
            frac_min *= 10;
            if (isdigit(*q))
                frac_min += *q++ - '0';
        }
    }
    return deg * 10000000UL + (min * 1000000UL + frac_min * 100UL) / 6;
}

// helper functions
static uint32_t grab_fields(char *src, uint8_t mult){            // convert string to uint32
    uint8_t i;
    uint32_t tmp = 0;
    for (i = 0; src[i] != 0; i++){
        if (src[i] == '.'){
            i++;
            if (mult == 0) break;
            else src[i + mult] = 0;
        }
        tmp *= 10;
        if (src[i] >= '0' && src[i] <= '9')
            tmp += src[i] - '0';
    }
    return tmp;
}

static uint8_t hex_c(uint8_t n){                                // convert '0'..'9','A'..'F' to 0..15
    n -= '0';
    if (n > 9) n -= 7;
    n &= 0x0F;
    return n;
}

static bool GPS_newFrame(char c){                     // Crashpilot
  	switch (cfg.gps_type){	                          // GPS_NMEA = 0, GPS_UBLOX = 1, GPS_MTK16 = 2, GPS_MTK19 = 3, GPS_UBLOX_DUMB = 4
      case 0:                                         // NMEA
      return GPS_NMEA_newFrame(c);					
      case 1:                                         // UBX
      case 4:				
      return GPS_UBLOX_newFrame(c);
			case 2:                                         // Dealing with old, faulty and new, correct binary protocol
			case 3: 
      return GPS_MTK_newFrame(c);                     // GPS_MTK_newFrame handles both 1.6 and 1.9 3drobotics nomenclature
    }
    return false;
}

static bool GPS_MTK_newFrame(uint8_t data){           // Crashpilot: This code is stupid but works
   static uint8_t pstep;                              // Parse Step
   static uint8_t lastbyte;                           // Last Byte for Sync detection
   static uint8_t LSLshifter;                         // Bitshiftvalue
   static uint8_t chkA,count;
   static int32_t lat;                                // MTK Dataset
   static int32_t lon;                                // MTK Dataset
   static int32_t alt;                                // MTK Dataset
   static int32_t grspeed;                            // MTK Dataset
   static int32_t grcourse;                           // MTK Dataset
   static uint8_t satn,fixtype;                       // MTK Dataset
   int32_t tmp32;
   uint8_t startbyte;                                 // must be unsigned
   bool parsed;
   parsed = false;
   startbyte = 0;                                     // Initialize to be on the safe side so casemachine will bail out on VERY unlikely error
   // GPS_NMEA = 0, GPS_UBLOX = 1, GPS_MTK16 = 2, GPS_MTK19 = 3,
	 if (cfg.gps_type == 2) startbyte = 0xd0;           // 3drobotics 1.6 FW and clones have $d0 preamblebyte no 1
	 if (cfg.gps_type == 3) startbyte = 0xd1;           // 3drobotics 1.9 FW and clones have $d1 preamblebyte no 1
 	 if (pstep == 0 && data == 0xdd && lastbyte == startbyte) pstep = 100; // Detect Sync "0xD1,0xDD" Only search for Sync when not already decoding
   lastbyte = data;
   switch(pstep) {
    case 0:                                           // Special Case: Do Nothing
    break;
    case 100:                                         // Special Case: Prepare next decoding run
    pstep = 1;                                        // Jump into decoding on next run
    chkA = 0; count = 0;                              // Reset values
    break;
    case 1:                                           // Payload Byte is always $20! (This is the first Byte after sync preamble)
    if (data == 0x20) pstep++;                        // Since it is always $20 we take it as extended, undocumented syncword "preamble3"
     else pstep =0;                                   // Error! Wait for sync
    chkA = chkA + data;
    count ++;
    break;
    case 2:                                           // Read Dataset Latitude
    lat = data;
    LSLshifter = 0;
    pstep++;
    chkA = chkA + data;
    count ++;
    break;
    case 3:
    LSLshifter = LSLshifter+8;
    tmp32 = data;
    tmp32 = tmp32<<LSLshifter;
    lat = lat | tmp32;
    if (LSLshifter == 24) pstep++;                    // FW APM TEST //    if (LSLshifter == 24){lat = lat * 10; pstep++;}
    chkA = chkA + data;
    count ++;
    break;    
    case 4:                                           // Read Dataset Longitude
    lon = data;
    LSLshifter = 0;
    pstep++;
    chkA = chkA + data;
    count ++;
    break;
    case 5:
    LSLshifter = LSLshifter+8;
    tmp32 = data;
    tmp32 = tmp32<<LSLshifter;
    lon = lon | tmp32;
    if (LSLshifter == 24) pstep++;                    // FW APM TEST //    if (LSLshifter == 24){lon = lon * 10; pstep++;}
    chkA = chkA + data;
    count ++;
    break;
    case 6:                                           // Read Dataset MSL Altitude
    alt = data;
    LSLshifter = 0;
    pstep++;
    chkA = chkA + data;
    count ++;
    break;
    case 7:
    LSLshifter = LSLshifter+8;
    tmp32 = data;
    tmp32 = tmp32<<LSLshifter;
    alt = alt | tmp32;
    if (LSLshifter == 24){alt = alt/100; pstep++;}    // altitude in meter
    chkA = chkA + data;
    count ++;
    break;
    case 8:                                           // Read Dataset Ground Speed
    grspeed = data;
    LSLshifter = 0;
    pstep++;
    chkA = chkA + data;
    count ++;
    break;
    case 9:
    LSLshifter = LSLshifter+8;
    tmp32 = data;
    tmp32 = tmp32<<LSLshifter;
    grspeed = grspeed | tmp32;
    if (LSLshifter == 24) pstep++;
    chkA = chkA + data;
    count ++;
    break;
    case 10:                                          // Read Dataset Heading
    grcourse = data;
    LSLshifter = 0;
    pstep++;
    chkA = chkA + data;
    count ++;
    break;
    case 11:
    LSLshifter = LSLshifter+8;
    tmp32 = data;
    tmp32 = tmp32<<LSLshifter;
    grcourse = grcourse | tmp32;
    if (LSLshifter == 24) pstep++;
    chkA = chkA + data;
    count ++;
    break;
    case 12:                                          // Read number of satellites in view
    satn = data;
    pstep++;
    chkA = chkA + data;
    count ++;
    break;
    case 13:                                          // Read Fix Type
    fixtype = data;                                   // FIX_NONE = 1, FIX_2D = 2, FIX_3D = 3, FIX_2D_SBAS = 6, FIX_3D_SBAS = 7
    pstep++;
    chkA = chkA + data;
    count ++;
    break;
    case 14:                                          // Wait for cheksum A
    if (count == 33){                                 // 33 = 0x21
      if (chkA == data) pstep++;                      // ChecksumA reached. Correct? than go on
       else pstep = 0;                                // Error?
    }else{
      chkA = chkA + data;
      count ++;}
    break;
    case 15:                                          // Dataset RDY !! Cheksum B omitted, ChkA was OK
		if (fixtype > 1) f.GPS_FIX = true;
		else f.GPS_FIX = false;
	  if (startbyte == 0xd0){                           // We are dealing with old and faulty binary protocol here
      lat = lat * 10;                                 // so we have to multiply by 10 lat and lon
		  lon = lon * 10;}
		Real_GPS_coord[LAT] = lat;                        // GPS_read[LAT] = lat; GPS_read[LON] = lon;
		Real_GPS_coord[LON] = lon;
		GPS_altitude = alt;                               // i2c_dataset.altitude = alt;
    GPS_speed = grspeed;                              // i2c_dataset.ground_speed = grspeed;
    GPS_ground_course = grcourse;                     // i2c_dataset.ground_course = grcourse;
    GPS_numSat = satn;                                // i2c_dataset.status.numsats = satn;
    GPS_Present = 1;                                  // Naze: Show GPS is working
		parsed = true;                                    // RDY
    pstep = 0;                                        // Do nothing
    break;
   }
 return parsed;
}

#define FRAME_GGA  1
#define FRAME_RMC  2
static bool GPS_NMEA_newFrame(char c){                //mwii1320
    uint8_t frameOK = 0;
    static uint8_t param = 0, offset = 0, parity = 0;
    static char string[15];
    static uint8_t checksum_param, frame = 0;
  
    if (c == '$') {
      param = 0; offset = 0; parity = 0;
    } else if (c == ',' || c == '*') {
      string[offset] = 0;
      if (param == 0) { //frame identification
        frame = 0;
        if (string[0] == 'G' && string[1] == 'P' && string[2] == 'G' && string[3] == 'G' && string[4] == 'A') frame = FRAME_GGA;
        if (string[0] == 'G' && string[1] == 'P' && string[2] == 'R' && string[3] == 'M' && string[4] == 'C') frame = FRAME_RMC;
      } else if (frame == FRAME_GGA) {
        if      (param == 2)                     {Real_GPS_coord[LAT] = GPS_coord_to_degrees(string);}
        else if (param == 3 && string[0] == 'S') Real_GPS_coord[LAT] = -Real_GPS_coord[LAT];
        else if (param == 4)                     {Real_GPS_coord[LON] = GPS_coord_to_degrees(string);}
        else if (param == 5 && string[0] == 'W') Real_GPS_coord[LON] = -Real_GPS_coord[LON];
        else if (param == 6)                     {f.GPS_FIX = (string[0]  > '0');}
        else if (param == 7)                     {GPS_numSat = grab_fields(string,0);}
        else if (param == 9)                     {GPS_altitude = grab_fields(string,0);}  // altitude in meters added by Mis
      } else if (frame == FRAME_RMC) {
        if      (param == 7)                     {GPS_speed = ((uint32_t)grab_fields(string,1)*5144L)/1000L;}  //gps speed in cm/s will be used for navigation
        else if (param == 8)                     {GPS_ground_course = grab_fields(string,1); }                 //ground course deg*10 
      }
      param++; offset = 0;
      if (c == '*') checksum_param=1;
      else parity ^= c;
    } else if (c == '\r' || c == '\n') {
      if (checksum_param) { //parity checksum
        uint8_t checksum = hex_c(string[0]);
        checksum <<= 4;
        checksum += hex_c(string[1]);
        if (checksum == parity) frameOK = 1;
      }
      checksum_param=0;
    } else {
       if (offset < 15) string[offset++] = c;
       if (!checksum_param) parity ^= c;
    }
    if (frame) GPS_Present = 1;
    return frameOK && (frame==FRAME_GGA);
}

// UBX support Mwii 2.2 pre 3/3/13
typedef struct {
    uint8_t preamble1;
    uint8_t preamble2;
    uint8_t msg_class;
    uint8_t msg_id;
    uint16_t length;
  }ubx_header;

typedef struct {
    uint32_t time;  // GPS msToW
    int32_t longitude;
    int32_t latitude;
    int32_t altitude_ellipsoid;
    int32_t altitude_msl;
    uint32_t horizontal_accuracy;
    uint32_t vertical_accuracy;
  }ubx_nav_posllh;

typedef struct {
    uint32_t time;
    int32_t time_nsec;
    int16_t week;
    uint8_t fix_type;
    uint8_t fix_status;
    int32_t ecef_x;
    int32_t ecef_y;
    int32_t ecef_z;
    uint32_t position_accuracy_3d;
    int32_t ecef_x_velocity;
    int32_t ecef_y_velocity;
    int32_t ecef_z_velocity;
    uint32_t speed_accuracy;
    uint16_t position_DOP;
    uint8_t res;
    uint8_t satellites;
    uint32_t res2;
  }ubx_nav_solution;

typedef struct {
    uint32_t time;  // GPS msToW
    int32_t ned_north;
    int32_t ned_east;
    int32_t ned_down;
    uint32_t speed_3d;
    uint32_t speed_2d;
    int32_t heading_2d;
    uint32_t speed_accuracy;
    uint32_t heading_accuracy;
  }ubx_nav_velned;

  enum {
    PREAMBLE1 = 0xb5,
    PREAMBLE2 = 0x62,
    CLASS_NAV = 0x01,
    CLASS_ACK = 0x05,
    CLASS_CFG = 0x06,
    MSG_ACK_NACK = 0x00,
    MSG_ACK_ACK = 0x01,
    MSG_POSLLH = 0x2,
    MSG_STATUS = 0x3,
    MSG_SOL = 0x6,
    MSG_VELNED = 0x12,
    MSG_CFG_PRT = 0x00,
    MSG_CFG_RATE = 0x08,
    MSG_CFG_SET_RATE = 0x01,
    MSG_CFG_NAV_SETTINGS = 0x24
  }ubs_protocol_bytes;
	
  enum {
    FIX_NONE = 0,
    FIX_DEAD_RECKONING = 1,
    FIX_2D = 2,
    FIX_3D = 3,
    FIX_GPS_DEAD_RECKONING = 4,
    FIX_TIME = 5
  }ubs_nav_fix_type;
	
  enum {
    NAV_STATUS_FIX_VALID = 1
  }ubx_nav_status_bits;

  static uint8_t _ck_a;                    // Packet checksum accumulators
  static uint8_t _ck_b;
  static uint8_t _step;                    // State machine state
  static uint8_t _msg_id;
  static uint16_t _payload_length;
  static uint16_t _payload_counter;
  static uint8_t _class;                   //  static bool next_fix;
  static uint8_t _fix_ok;

  static union {                           // Receive buffer
    ubx_nav_posllh posllh;                 // ubx_nav_status status;
    ubx_nav_solution solution;
    ubx_nav_velned velned;
    uint8_t bytes[64];
   } _buffer;
	
	void _update_checksum(uint8_t *data, uint8_t len, uint8_t *ck_a, uint8_t *ck_b){
    while (len--) {
        *ck_a += *data;
        *ck_b += *ck_a;
        data++;
    }
  }

static bool GPS_UBLOX_newFrame(uint8_t data){
    bool parsed = false;

    switch(_step) {
      case 1:
        if (PREAMBLE2 == data) {
          _step++;
          break;
        }
        _step = 0;
      case 0:
        if(PREAMBLE1 == data) _step++;
        break;
      case 2:
        _step++;
        _class = data;
        _ck_b = _ck_a = data;                // reset the checksum accumulators
        break;
      case 3:
        _step++;
        _ck_b += (_ck_a += data);            // checksum byte
        _msg_id = data;
        break;
      case 4:
        _step++;
        _ck_b += (_ck_a += data);           // checksum byte
        _payload_length = data;             // payload length low byte
        break;
      case 5:
        _step++;
        _ck_b += (_ck_a += data);           // checksum byte
        _payload_length += (uint16_t)(data<<8);
        if (_payload_length > 512) {
          _payload_length = 0;
          _step = 0;
        }
        _payload_counter = 0;               // prepare to receive payload
      break;
      case 6:
        _ck_b += (_ck_a += data);           // checksum byte
        if (_payload_counter < sizeof(_buffer)) {
          _buffer.bytes[_payload_counter] = data;
        }
        if (++_payload_counter == _payload_length)
          _step++;
        break;
      case 7:
        _step++;
        if (_ck_a != data) _step = 0;      // bad checksum
      break;
      case 8:
        _step = 0;
        if (_ck_b != data)  break;         // bad checksum
        GPS_Present = 1;
        if (UBLOX_parse_gps())  { parsed = true; }
    } //end switch
    return parsed;
  }

  bool UBLOX_parse_gps(void) {
    switch (_msg_id) {
    case MSG_POSLLH:
      if(_fix_ok) {
        Real_GPS_coord[LON] = _buffer.posllh.longitude;
        Real_GPS_coord[LAT] = _buffer.posllh.latitude;
        GPS_altitude        = _buffer.posllh.altitude_msl / 1000;      //alt in m
      }
      f.GPS_FIX = _fix_ok;
      return true;        // POSLLH message received, allow blink GUI icon and LED
      break;
    case MSG_SOL:
      _fix_ok = 0;
      if((_buffer.solution.fix_status & NAV_STATUS_FIX_VALID) && (_buffer.solution.fix_type == FIX_3D || _buffer.solution.fix_type == FIX_2D)) _fix_ok = 1;
      GPS_numSat = _buffer.solution.satellites;
      break;
    case MSG_VELNED:
      GPS_speed         = _buffer.velned.speed_2d;  // cm/s
      GPS_ground_course = (uint16_t)(_buffer.velned.heading_2d / 10000);  // Heading 2D deg * 100000 rescaled to deg * 10
      break;
    default:
      break;
    }
    return false;
  }
	
/*
////////////////////////////////////////////////////////////////////////////////////
// *** DIFFERENT STUFF THAT WORKED AS WELL *** POOL OF CODE THAT MIGHT BECOME USEFUL
////////////////////////////////////////////////////////////////////////////////////

GPS_coord[LAT]  = GPS_coord[LAT] + (int32_t)(InsPOS * (float)(Real_GPS_coord[LAT] - GPS_coord[LAT])); // These steps are historical, since gps_lag stuff
GPS_coord[LON]  = GPS_coord[LON] + (int32_t)(InsPOS * (float)(Real_GPS_coord[LON] - GPS_coord[LON]));	
	
            if (abs(gyroData[2]) > 2000){                                         // Just use GPS when too much Yaw
                InsVEL = 0.0f;
                InsPOS = 1.0f;
						}

static float SpecialSQRT(float value){        // Handles negative sqrt
	  if (value == 0) return 0;
	  if (value  > 0) return sqrtf(value);
	   else return -sqrtf(abs(value));
}
	
//////////
PLAN A OLD
//////////
////////////////////////////////////////////////////////////////////////////////////
// Calculate the desired nav_lat and nav_lon for distance flying such as RTH
static void GPS_calc_nav_rate(int16_t max_speed)
{
    float trig[2];
    float temp;
    uint8_t axis;
    if (abs(wrap_18000(target_bearing - original_target_bearing)) < 4500) {          // If we are too far off or too close we don't do track following
        float temp = (target_bearing - original_target_bearing) * RADX100;
        crosstrack_error = sinf(temp) * wp_distance;                                 // Meters we are off track line
        nav_bearing = target_bearing + constrain(crosstrack_error, -3000, 3000);
        nav_bearing = wrap_36000(nav_bearing);
    } else nav_bearing = target_bearing;
	  temp = (9000l - nav_bearing) * RADX100;                                          // nav_bearing includes crosstrack
	  trig[GPS_X] = cosf(temp);
    trig[GPS_Y] = sinf(temp);
	  for (axis = 0; axis < 2; axis++) {
        rate_error[axis] = (trig[axis] * (float)max_speed) - actual_speed[axis];
        rate_error[axis] = constrain(rate_error[axis], -1000, 1000);
        nav[axis] = get_P(rate_error[axis], &navPID_PARAM) +                         // P + I + D
                    get_I(rate_error[axis], &dTnav, &navPID[axis], &navPID_PARAM) +
                    get_D(rate_error[axis], &dTnav, &navPID[axis], &navPID_PARAM);
        nav[axis] = constrain(nav[axis], -maxbank100, maxbank100);
        poshold_ratePID[axis].integrator = navPID[axis].integrator;
    }
}

static int16_t GPS_calc_desired_speed(void)
{
		int16_t max_speed = cfg.nav_speed_max;                  // initialize maxspeed with max speed ...
    if (!WP_Fastcorner) {
        max_speed = min(max_speed, wp_distance / 3);
    } else {
        max_speed = min(max_speed, wp_distance);
        max_speed = max(max_speed, cfg.nav_speed_min);      // go at least 100cm/s
    }
    if (max_speed > waypoint_speed_gov) {
        waypoint_speed_gov += (100.0f * dTnav);             // increase at .5/ms
        max_speed = waypoint_speed_gov;
    }
    return max_speed;
}

//////////
PLAN B NEW
//////////
////////////////////////////////////////////////////////////////////////////////////
// Calculate the desired nav_lat and nav_lon for distance flying such as RTH
# define CROSSTRACK_GAIN                 .2    // APM default
# define CROSSTRACK_MIN_DISTANCE       1500    // APM default
static void GPS_calc_nav_rate(int16_t max_speed)
{
    float temp, temp_x, temp_y, cross_speed, x_target_speed, y_target_speed;
		
		if (wp_distance >= CROSSTRACK_MIN_DISTANCE && abs(wrap_18000(target_bearing - original_target_bearing)) < 4500) {
        temp = (target_bearing - original_target_bearing) * RADX100;
    	  crosstrack_error = sinf(temp) * wp_distance;                                      // m?? we are off track line
        nav_bearing = target_bearing + constrain(crosstrack_error, -3000, 3000);          // For Tailcontrol
        nav_bearing = wrap_36000(nav_bearing);
    }else{
			  crosstrack_error = 0;
		    nav_bearing      = target_bearing;
		}
    cross_speed       = constrain((float)crosstrack_error * -CROSSTRACK_GAIN, -150, 150);
    temp              = (9000l - target_bearing) * RADX100;                               // rotate by 90 to deal with trig functions
    temp_x            = cosf(temp);
    temp_y            = sinf(temp);
    x_target_speed    = (float)max_speed * temp_x - cross_speed * temp_y;                 // rotate desired speed vector
    y_target_speed    = cross_speed * temp_x + (float)max_speed * temp_y;
	  rate_error[GPS_X] = constrain(x_target_speed - actual_speed[GPS_X], -500, 500);       // was +-1000 before
		rate_error[GPS_Y] = constrain(y_target_speed - actual_speed[GPS_Y], -500, 500);       // was +-1000 before
    nav[LON]          = get_P(rate_error[GPS_X], &navPID_PARAM) +
                        get_I(rate_error[GPS_X], &dTnav, &navPID[GPS_X], &navPID_PARAM) +
                        get_D(rate_error[GPS_X], &dTnav, &navPID[GPS_X], &navPID_PARAM);
    nav[LAT]          = get_P(rate_error[GPS_Y], &navPID_PARAM) +
                        get_I(rate_error[GPS_Y], &dTnav, &navPID[GPS_Y], &navPID_PARAM) +
                        get_D(rate_error[GPS_Y], &dTnav, &navPID[GPS_Y], &navPID_PARAM);
    nav[LON]          = constrain(nav[LON], -maxbank100, maxbank100);
    nav[LAT]          = constrain(nav[LAT], -maxbank100, maxbank100);

    poshold_ratePID[GPS_X].integrator = navPID[GPS_X].integrator;                         //Obsolete feed back to PH
    poshold_ratePID[GPS_Y].integrator = navPID[GPS_Y].integrator;                         //Obsolete feed back to PH
	
//  I dont think this is necessary any more
//  if(x_target_speed < 0) tilt = -tilt;
//  nav_lon                 += tilt;
//  if(y_target_speed < 0) tilt = -tilt;
//  nav_lat                 += tilt;
}

////////////////////////////////////////////////////////////////////////////////////
//	Based on Equation by Bill Premerlani & Robert Lefebvre
//    	(sq(V2)-sq(V1))/2 = A(X2-X1)
//   So: V1 = sqrt(sq(V2) - 2*A*(X2-X1))
static int16_t GPS_calc_desired_speed(void)                               // Ramp up / brake
{
	  int16_t max_speed = cfg.nav_speed_max;                                // initialize maxspeed with max speed ...
	  bool    braking   = false;
  	if(!WP_Fastcorner && wp_distance < 20000) {                           // don't slow down if WP_Fastcorner is true && limit numbers to avoid overflow
        float temp 	  = 200.0f * (float)(wp_distance - cfg.gps_wp_radius);
    	  float s_min   = cfg.nav_speed_min;
    	  temp         += s_min * s_min;
        if(temp < 0 ) temp = 0;                                           // check sqrt gets positive number
    		max_speed 		= sqrt(temp);
        max_speed 		= min(max_speed, cfg.nav_speed_max);                // Limit max_speed to nav_speed_max lowest speed is not limited
        if (max_speed < cfg.nav_speed_max) braking = true;
    }

		if (!braking && waypoint_speed_gov < cfg.nav_speed_max) {             // Rampup if not barking and if rampup is not done
        waypoint_speed_gov += (100.0f * dTnav);                           // increase at 1m/s /s
        max_speed = waypoint_speed_gov;
    }

		max_speed = constrain(max_speed,cfg.nav_speed_min,cfg.nav_speed_max); // Put output in desired range
    return max_speed;
}
////////////////////////////////////////////////////////////////////////////////////
// *** DIFFERENT STUFF THAT WORKED AS WELL ***
////////////////////////////////////////////////////////////////////////////////////
*/