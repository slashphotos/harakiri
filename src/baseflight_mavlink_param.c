
#include "board.h"
#include "mw.h"
#include "baseflight_mavlink.h"

void baseflight_mavlink_set_param (mavlink_param_set_t *packet) {
	if (strcmp(packet->param_id, "") == 0) {
	} else if (strcmp(packet->param_id, "RC_DBAND") == 0) {
		uint8_t val = (uint8_t)packet->param_value;
		if (val >= 0 && val <= 32) {
			cfg.deadband = val;
		}
	} else if (strcmp(packet->param_id, "YAW_DBAND") == 0) {
		uint8_t val = (uint8_t)packet->param_value;
		if (val >= 0 && val <= 100) {
			cfg.yawdeadband = val;
		}
	} else if (strcmp(packet->param_id, "THR_ALTHLD_NTL") == 0) {
		uint8_t val = (uint8_t)packet->param_value;
		if (val >= 1 && val <= 20) {
			cfg.alt_hold_throttle_neutral = val;
		}
	} else if (strcmp(packet->param_id, "NAV_PHDBAND") == 0) {
		uint8_t val = (uint8_t)packet->param_value;
		if (val >= 1 && val <= 200) {
			cfg.phdeadband = val;
		}
	} else if (strcmp(packet->param_id, "RC_MID") == 0) {
		uint16_t val = (uint16_t)packet->param_value;
		if (val >= 1200 && val <= 1700) {
			cfg.midrc = val;
		}
	} else if (strcmp(packet->param_id, "RC_RATE") == 0) {
		uint8_t val = (uint8_t)packet->param_value;
		if (val >= 0 && val <= 250) {
			cfg.rcRate8 = val;
		}
	} else if (strcmp(packet->param_id, "RC_EXPO") == 0) {
		uint8_t val = (uint8_t)packet->param_value;
		if (val >= 0 && val <= 100) {
			cfg.rcExpo8 = val;
		}
	} else if (strcmp(packet->param_id, "THR_MID") == 0) {
		uint8_t val = (uint8_t)packet->param_value;
		if (val >= 0 && val <= 100) {
			cfg.thrMid8 = val;
		}
	} else if (strcmp(packet->param_id, "THR_EXPO") == 0) {
		uint8_t val = (uint8_t)packet->param_value;
		if (val >= 0 && val <= 250) {
			cfg.thrExpo8 = val;
		}
	} else if (strcmp(packet->param_id, "ROLL_PITCH_RATE") == 0) {
		uint8_t val = (uint8_t)packet->param_value;
		if (val >= 0 && val <= 100) {
			cfg.rollPitchRate = val;
		}
	} else if (strcmp(packet->param_id, "YAW_RATE") == 0) {
		uint8_t val = (uint8_t)packet->param_value;
		if (val >= 0 && val <= 100) {
			cfg.yawRate = val;
		}
	} else if (strcmp(packet->param_id, "THR_MIN") == 0) {
		uint16_t val = (uint16_t)packet->param_value;
		if (val >= 0 && val <= 2000) {
			cfg.minthrottle = val;
		}
	} else if (strcmp(packet->param_id, "THR_MAX") == 0) {
		uint16_t val = (uint16_t)packet->param_value;
		if (val >= 0 && val <= 2000) {
			cfg.maxthrottle = val;
		}
	} else if (strcmp(packet->param_id, "ETC_PASSMOTOR") == 0) {
		uint8_t val = (uint8_t)packet->param_value;
		if (val >= 0 && val <= 10) {
			cfg.passmotor = val;
		}
	} else if (strcmp(packet->param_id, "RC_MINCMD") == 0) {
		uint16_t val = (uint16_t)packet->param_value;
		if (val >= 0 && val <= 2000) {
			cfg.mincommand = val;
		}
	} else if (strcmp(packet->param_id, "RC_MINCHK") == 0) {
		uint16_t val = (uint16_t)packet->param_value;
		if (val >= 0 && val <= 2000) {
			cfg.mincheck = val;
		}
	} else if (strcmp(packet->param_id, "RC_MAXCHK") == 0) {
		uint16_t val = (uint16_t)packet->param_value;
		if (val >= 0 && val <= 2000) {
			cfg.maxcheck = val;
		}
	} else if (strcmp(packet->param_id, "ETC_RTA_ARM") == 0) {
		uint8_t val = (uint8_t)packet->param_value;
		if (val >= 0 && val <= 1) {
			cfg.retarded_arm = val;
		}
	} else if (strcmp(packet->param_id, "RC_KILLSWIME") == 0) {
		uint16_t val = (uint16_t)packet->param_value;
		if (val >= 0 && val <= 10000) {
			cfg.killswitchtime = val;
		}
	} else if (strcmp(packet->param_id, "RC_FS_DELAY") == 0) {
		uint8_t val = (uint8_t)packet->param_value;
		if (val >= 0 && val <= 40) {
			cfg.failsafe_delay = val;
		}
	} else if (strcmp(packet->param_id, "RC_FS_OFF_DELAY") == 0) {
		uint8_t val = (uint8_t)packet->param_value;
		if (val >= 0 && val <= 200) {
			cfg.failsafe_off_delay = val;
		}
	} else if (strcmp(packet->param_id, "RC_FS_THR") == 0) {
		uint16_t val = (uint16_t)packet->param_value;
		if (val >= 1000 && val <= 2000) {
			cfg.failsafe_throttle = val;
		}
	} else if (strcmp(packet->param_id, "RC_FS_DEADPILOT") == 0) {
		uint8_t val = (uint8_t)packet->param_value;
		if (val >= 0 && val <= 250) {
			cfg.failsafe_deadpilot = val;
		}
	} else if (strcmp(packet->param_id, "RC_FS_JUSTPH") == 0) {
		uint8_t val = (uint8_t)packet->param_value;
		if (val >= 0 && val <= 1) {
			cfg.failsafe_justph = val;
		}
	} else if (strcmp(packet->param_id, "ETC_MOTOR_PWMR") == 0) {
		uint16_t val = (uint16_t)packet->param_value;
		if (val >= 50 && val <= 498) {
			cfg.motor_pwm_rate = val;
		}
	} else if (strcmp(packet->param_id, "ETC_SERVO_PWMR") == 0) {
		uint16_t val = (uint16_t)packet->param_value;
		if (val >= 50 && val <= 498) {
			cfg.servo_pwm_rate = val;
		}
	} else if (strcmp(packet->param_id, "ETC_SERIAL_BAUD") == 0) {
		uint32_t val = (uint32_t)packet->param_value;
		if (val >= 1200 && val <= 115200) {
			cfg.serial_baudrate = val;
		}
	} else if (strcmp(packet->param_id, "RC_SPTR_HIRES") == 0) {
		uint8_t val = (uint8_t)packet->param_value;
		if (val >= 0 && val <= 1) {
			cfg.spektrum_hires = val;
		}
	} else if (strcmp(packet->param_id, "VBAT_SCALE") == 0) {
		uint8_t val = (uint8_t)packet->param_value;
		if (val >= 10 && val <= 200) {
			cfg.vbatscale = val;
		}
	} else if (strcmp(packet->param_id, "VBAT_MAXCLVOLT") == 0) {
		uint8_t val = (uint8_t)packet->param_value;
		if (val >= 10 && val <= 50) {
			cfg.vbatmaxcellvoltage = val;
		}
	} else if (strcmp(packet->param_id, "VBAT_MINCLVOLT") == 0) {
		uint8_t val = (uint8_t)packet->param_value;
		if (val >= 10 && val <= 50) {
			cfg.vbatmincellvoltage = val;
		}
	} else if (strcmp(packet->param_id, "ETC_PWR_ADC") == 0) {
		uint8_t val = (uint8_t)packet->param_value;
		if (val >= 0 && val <= 9) {
			cfg.power_adc_channel = val;
		}
	} else if (strcmp(packet->param_id, "YAW_DIR") == 0) {
		int8_t val = (int8_t)packet->param_value;
		if (val >= -1 && val <= 1) {
			cfg.yaw_direction = val;
		}
	} else if (strcmp(packet->param_id, "TRI_YAW_MIDDLE") == 0) {
		uint16_t val = (uint16_t)packet->param_value;
		if (val >= 0 && val <= 2000) {
			cfg.tri_yaw_middle = val;
		}
	} else if (strcmp(packet->param_id, "TRI_YAW_MIN") == 0) {
		uint16_t val = (uint16_t)packet->param_value;
		if (val >= 0 && val <= 2000) {
			cfg.tri_yaw_min = val;
		}
	} else if (strcmp(packet->param_id, "TRI_YAW_MAX") == 0) {
		uint16_t val = (uint16_t)packet->param_value;
		if (val >= 0 && val <= 2000) {
			cfg.tri_yaw_max = val;
		}
	} else if (strcmp(packet->param_id, "WING_LEFT_MIN") == 0) {
		uint16_t val = (uint16_t)packet->param_value;
		if (val >= 0 && val <= 2000) {
			cfg.wing_left_min = val;
		}
	} else if (strcmp(packet->param_id, "WING_LEFT_MID") == 0) {
		uint16_t val = (uint16_t)packet->param_value;
		if (val >= 0 && val <= 2000) {
			cfg.wing_left_mid = val;
		}
	} else if (strcmp(packet->param_id, "WING_LEFT_MAX") == 0) {
		uint16_t val = (uint16_t)packet->param_value;
		if (val >= 0 && val <= 2000) {
			cfg.wing_left_max = val;
		}
	} else if (strcmp(packet->param_id, "WING_RIGHT_MIN") == 0) {
		uint16_t val = (uint16_t)packet->param_value;
		if (val >= 0 && val <= 2000) {
			cfg.wing_right_min = val;
		}
	} else if (strcmp(packet->param_id, "WING_RIGHT_MID") == 0) {
		uint16_t val = (uint16_t)packet->param_value;
		if (val >= 0 && val <= 2000) {
			cfg.wing_right_mid = val;
		}
	} else if (strcmp(packet->param_id, "WING_RIGHT_MAX") == 0) {
		uint16_t val = (uint16_t)packet->param_value;
		if (val >= 0 && val <= 2000) {
			cfg.wing_right_max = val;
		}
	} else if (strcmp(packet->param_id, "PITCH_DIR_L") == 0) {
		int8_t val = (int8_t)packet->param_value;
		if (val >= -1 && val <= 1) {
			cfg.pitch_direction_l = val;
		}
	} else if (strcmp(packet->param_id, "PITCH_DIR_R") == 0) {
		int8_t val = (int8_t)packet->param_value;
		if (val >= -1 && val <= 1) {
			cfg.pitch_direction_r = val;
		}
	} else if (strcmp(packet->param_id, "ROLL_DIR_L") == 0) {
		int8_t val = (int8_t)packet->param_value;
		if (val >= -1 && val <= 1) {
			cfg.roll_direction_l = val;
		}
	} else if (strcmp(packet->param_id, "ROLL_DIR_R") == 0) {
		int8_t val = (int8_t)packet->param_value;
		if (val >= -1 && val <= 1) {
			cfg.roll_direction_r = val;
		}
	} else if (strcmp(packet->param_id, "GMBL_FLAGS") == 0) {
		uint8_t val = (uint8_t)packet->param_value;
		if (val >= 0 && val <= 255) {
			cfg.gimbal_flags = val;
		}
	} else if (strcmp(packet->param_id, "GMBL_PITCH_GAIN") == 0) {
		int8_t val = (int8_t)packet->param_value;
		if (val >= -100 && val <= 100) {
			cfg.gimbal_pitch_gain = val;
		}
	} else if (strcmp(packet->param_id, "GMBL_ROLL_GAIN") == 0) {
		int8_t val = (int8_t)packet->param_value;
		if (val >= -100 && val <= 100) {
			cfg.gimbal_roll_gain = val;
		}
	} else if (strcmp(packet->param_id, "GMBL_PITCH_MIN") == 0) {
		uint16_t val = (uint16_t)packet->param_value;
		if (val >= 100 && val <= 3000) {
			cfg.gimbal_pitch_min = val;
		}
	} else if (strcmp(packet->param_id, "GMBL_PITCH_MAX") == 0) {
		uint16_t val = (uint16_t)packet->param_value;
		if (val >= 100 && val <= 3000) {
			cfg.gimbal_pitch_max = val;
		}
	} else if (strcmp(packet->param_id, "GMBL_PITCH_MID") == 0) {
		uint16_t val = (uint16_t)packet->param_value;
		if (val >= 100 && val <= 3000) {
			cfg.gimbal_pitch_mid = val;
		}
	} else if (strcmp(packet->param_id, "GMBL_ROLL_MIN") == 0) {
		uint16_t val = (uint16_t)packet->param_value;
		if (val >= 100 && val <= 3000) {
			cfg.gimbal_roll_min = val;
		}
	} else if (strcmp(packet->param_id, "GMBL_ROLL_MAX") == 0) {
		uint16_t val = (uint16_t)packet->param_value;
		if (val >= 100 && val <= 3000) {
			cfg.gimbal_roll_max = val;
		}
	} else if (strcmp(packet->param_id, "GMBL_ROLL_MID") == 0) {
		uint16_t val = (uint16_t)packet->param_value;
		if (val >= 100 && val <= 3000) {
			cfg.gimbal_roll_mid = val;
		}
	} else if (strcmp(packet->param_id, "ETC_AL_RATE") == 0) {
		uint8_t val = (uint8_t)packet->param_value;
		if (val >= 30 && val <= 200) {
			cfg.autolandrate = val;
		}
	} else if (strcmp(packet->param_id, "GYRO_X_ALIGN") == 0) {
		int8_t val = (int8_t)packet->param_value;
		if (val >= -3 && val <= 3) {
			cfg.align[ALIGN_GYRO][0] = val;
		}
	} else if (strcmp(packet->param_id, "GYRO_Y_ALIGN") == 0) {
		int8_t val = (int8_t)packet->param_value;
		if (val >= -3 && val <= 3) {
			cfg.align[ALIGN_GYRO][1] = val;
		}
	} else if (strcmp(packet->param_id, "GYRO_Z_ALIGN") == 0) {
		int8_t val = (int8_t)packet->param_value;
		if (val >= -3 && val <= 3) {
			cfg.align[ALIGN_GYRO][2] = val;
		}
	} else if (strcmp(packet->param_id, "ACC_X_ALIGN") == 0) {
		int8_t val = (int8_t)packet->param_value;
		if (val >= -3 && val <= 3) {
			cfg.align[ALIGN_ACCEL][0] = val;
		}
	} else if (strcmp(packet->param_id, "ACC_Y_ALIGN") == 0) {
		int8_t val = (int8_t)packet->param_value;
		if (val >= -3 && val <= 3) {
			cfg.align[ALIGN_ACCEL][1] = val;
		}
	} else if (strcmp(packet->param_id, "ACC_Z_ALIGN") == 0) {
		int8_t val = (int8_t)packet->param_value;
		if (val >= -3 && val <= 3) {
			cfg.align[ALIGN_ACCEL][2] = val;
		}
	} else if (strcmp(packet->param_id, "MAG_X_ALIGN") == 0) {
		int8_t val = (int8_t)packet->param_value;
		if (val >= -3 && val <= 3) {
			cfg.align[ALIGN_MAG][0] = val;
		}
	} else if (strcmp(packet->param_id, "MAG_Y_ALIGN") == 0) {
		int8_t val = (int8_t)packet->param_value;
		if (val >= -3 && val <= 3) {
			cfg.align[ALIGN_MAG][1] = val;
		}
	} else if (strcmp(packet->param_id, "MAG_Z_ALIGN") == 0) {
		int8_t val = (int8_t)packet->param_value;
		if (val >= -3 && val <= 3) {
			cfg.align[ALIGN_MAG][2] = val;
		}
	} else if (strcmp(packet->param_id, "ACC_HARDWARE") == 0) {
		uint8_t val = (uint8_t)packet->param_value;
		if (val >= 0 && val <= 3) {
			cfg.acc_hardware = val;
		}
	} else if (strcmp(packet->param_id, "ACC_LPF_FACTOR") == 0) {
		uint8_t val = (uint8_t)packet->param_value;
		if (val >= 1 && val <= 250) {
			cfg.acc_lpf_factor = val;
		}
	} else if (strcmp(packet->param_id, "ACC_INS_LPF") == 0) {
		uint8_t val = (uint8_t)packet->param_value;
		if (val >= 1 && val <= 250) {
			cfg.acc_ins_lpf = val;
		}
	} else if (strcmp(packet->param_id, "ACC_TRIM_PITCH") == 0) {
		int16_t val = (int16_t)packet->param_value;
		if (val >= -300 && val <= 300) {
			cfg.angleTrim[PITCH] = val;
		}
	} else if (strcmp(packet->param_id, "ACC_TRIM_ROLL") == 0) {
		int16_t val = (int16_t)packet->param_value;
		if (val >= -300 && val <= 300) {
			cfg.angleTrim[ROLL] = val;
		}
	} else if (strcmp(packet->param_id, "GYRO_LPF") == 0) {
		uint16_t val = (uint16_t)packet->param_value;
		if (val >= 0 && val <= 256) {
			cfg.gyro_lpf = val;
		}
	} else if (strcmp(packet->param_id, "GYRO_CMPF_FACTOR") == 0) {
		uint16_t val = (uint16_t)packet->param_value;
		if (val >= 100 && val <= 1000) {
			cfg.gyro_cmpf_factor = val;
		}
	} else if (strcmp(packet->param_id, "ACC_Z_VEL_CF") == 0) {
		float val = (float)packet->param_value;
		if (val >= 0 && val <= 1) {
			cfg.accz_vel_cf = val;
		}
	} else if (strcmp(packet->param_id, "ACC_Z_ALT_CF") == 0) {
		float val = (float)packet->param_value;
		if (val >= 0 && val <= 1) {
			cfg.accz_alt_cf = val;
		}
	} else if (strcmp(packet->param_id, "BARO_LAG") == 0) {
		float val = (float)packet->param_value;
		if (val >= 0 && val <= 10) {
			cfg.baro_lag = val;
		}
	} else if (strcmp(packet->param_id, "BARO_SONAR_CF") == 0) {
		float val = (float)packet->param_value;
		if (val >= 0 && val <= 1) {
			cfg.baro_sonar_cf = val;
		}
	} else if (strcmp(packet->param_id, "BARO_DOWNSCALE") == 0) {
		float val = (float)packet->param_value;
		if (val >= 0 && val <= 1) {
			cfg.barodownscale = val;
		}
	} else if (strcmp(packet->param_id, "ETC_NAZEDBG") == 0) {
		uint8_t val = (uint8_t)packet->param_value;
		if (val >= 0 && val <= 5) {
			cfg.nazedebug = val;
		}
	} else if (strcmp(packet->param_id, "ETC_MORON_THRH") == 0) {
		uint8_t val = (uint8_t)packet->param_value;
		if (val >= 0 && val <= 128) {
			cfg.moron_threshold = val;
		}
	} else if (strcmp(packet->param_id, "MAG_DECL") == 0) {
		int16_t val = (int16_t)packet->param_value;
		if (val >= -18000 && val <= 18000) {
			cfg.mag_declination = val;
		}
	} else if (strcmp(packet->param_id, "MAG_OLDCALIB") == 0) {
		uint8_t val = (uint8_t)packet->param_value;
		if (val >= 0 && val <= 1) {
			cfg.mag_oldcalib = val;
		}
	} else if (strcmp(packet->param_id, "GPS_BRATE") == 0) {
		uint32_t val = (uint32_t)packet->param_value;
		if (val >= 1200 && val <= 115200) {
			cfg.gps_baudrate = val;
		}
	} else if (strcmp(packet->param_id, "GPS_TYPE") == 0) {
		uint8_t val = (uint8_t)packet->param_value;
		if (val >= 0 && val <= 9) {
			cfg.gps_type = val;
		}
	} else if (strcmp(packet->param_id, "GPS_INS_VEL") == 0) {
		float val = (float)packet->param_value;
		if (val >= 0 && val <= 1) {
			cfg.gps_ins_vel = val;
		}
	} else if (strcmp(packet->param_id, "GPS_LAG") == 0) {
		float val = (float)packet->param_value;
		if (val >= 0 && val <= 20) {
			cfg.gps_lag = val;
		}
	} else if (strcmp(packet->param_id, "GPS_SPEEDFILTER") == 0) {
		float val = (float)packet->param_value;
		if (val >= 0 && val <= 1) {
			cfg.gps_speedfilter = val;
		}
	} else if (strcmp(packet->param_id, "GPS_PHASE") == 0) {
		float val = (float)packet->param_value;
		if (val >= -30 && val <= 30) {
			cfg.gps_phase = val;
		}
	} else if (strcmp(packet->param_id, "GPS_PH_MINSAT") == 0) {
		uint8_t val = (uint8_t)packet->param_value;
		if (val >= 5 && val <= 10) {
			cfg.gps_ph_minsat = val;
		}
	} else if (strcmp(packet->param_id, "GPS_PH_APM") == 0) {
		uint8_t val = (uint8_t)packet->param_value;
		if (val >= 0 && val <= 1) {
			cfg.gps_ph_apm = val;
		}
	} else if (strcmp(packet->param_id, "GPS_PH_STLSPD") == 0) {
		uint16_t val = (uint16_t)packet->param_value;
		if (val >= 0 && val <= 1000) {
			cfg.gps_ph_settlespeed = val;
		}
	} else if (strcmp(packet->param_id, "GPS_PH_TRGTSQRT") == 0) {
		uint16_t val = (uint16_t)packet->param_value;
		if (val >= 1 && val <= 60000) {
			cfg.gps_ph_targetsqrt = val;
		}
	} else if (strcmp(packet->param_id, "GPS_PHMOVE_SPD") == 0) {
		float val = (float)packet->param_value;
		if (val >= 0 && val <= 100) {
			cfg.gps_phmove_speed = val;
		}
	} else if (strcmp(packet->param_id, "GPS_MAXANG") == 0) {
		uint8_t val = (uint8_t)packet->param_value;
		if (val >= 10 && val <= 45) {
			cfg.gps_maxangle = val;
		}
	} else if (strcmp(packet->param_id, "GPS_MINANGPRC") == 0) {
		uint8_t val = (uint8_t)packet->param_value;
		if (val >= 1 && val <= 100) {
			cfg.gps_minanglepercent = val;
		}
	} else if (strcmp(packet->param_id, "GPS_WP_RADIUS") == 0) {
		uint16_t val = (uint16_t)packet->param_value;
		if (val >= 0 && val <= 2000) {
			cfg.gps_wp_radius = val;
		}
	} else if (strcmp(packet->param_id, "GPS_RTL_MINHGT") == 0) {
		uint16_t val = (uint16_t)packet->param_value;
		if (val >= 0 && val <= 200) {
			cfg.gps_rtl_minhight = val;
		}
	} else if (strcmp(packet->param_id, "GPS_RTL_MINDIST") == 0) {
		uint8_t val = (uint8_t)packet->param_value;
		if (val >= 0 && val <= 50) {
			cfg.gps_rtl_mindist = val;
		}
	} else if (strcmp(packet->param_id, "GPS_RTL_FLYAWAY") == 0) {
		uint8_t val = (uint8_t)packet->param_value;
		if (val >= 0 && val <= 100) {
			cfg.gps_rtl_flyaway = val;
		}
	} else if (strcmp(packet->param_id, "GPS_YAW") == 0) {
		uint8_t val = (uint8_t)packet->param_value;
		if (val >= 20 && val <= 150) {
			cfg.gps_yaw = val;
		}
	} else if (strcmp(packet->param_id, "NAV_RTL_LASTTRN") == 0) {
		uint8_t val = (uint8_t)packet->param_value;
		if (val >= 0 && val <= 1) {
			cfg.nav_rtl_lastturn = val;
		}
	} else if (strcmp(packet->param_id, "NAV_SPEED_MIN") == 0) {
		int16_t val = (int16_t)packet->param_value;
		if (val >= 10 && val <= 2000) {
			cfg.nav_speed_min = val;
		}
	} else if (strcmp(packet->param_id, "NAV_SPEED_MAX") == 0) {
		int16_t val = (int16_t)packet->param_value;
		if (val >= 50 && val <= 2000) {
			cfg.nav_speed_max = val;
		}
	} else if (strcmp(packet->param_id, "NAV_SLEW_RATE") == 0) {
		uint8_t val = (uint8_t)packet->param_value;
		if (val >= 10 && val <= 200) {
			cfg.nav_slew_rate = val;
		}
	} else if (strcmp(packet->param_id, "NAV_CTRL_HEADNG") == 0) {
		uint8_t val = (uint8_t)packet->param_value;
		if (val >= 0 && val <= 1) {
			cfg.nav_controls_heading = val;
		}
	} else if (strcmp(packet->param_id, "NAV_TAIL_FIRST") == 0) {
		uint8_t val = (uint8_t)packet->param_value;
		if (val >= 0 && val <= 1) {
			cfg.nav_tail_first = val;
		}
	} else if (strcmp(packet->param_id, "GPS_POS_P") == 0) {
		uint8_t val = (uint8_t)packet->param_value;
		if (val >= 0 && val <= 200) {
			cfg.P8[PIDPOS] = val;
		}
	} else if (strcmp(packet->param_id, "GPS_POS_I") == 0) {
		uint8_t val = (uint8_t)packet->param_value;
		if (val >= 0 && val <= 200) {
			cfg.I8[PIDPOS] = val;
		}
	} else if (strcmp(packet->param_id, "GPS_POS_D") == 0) {
		uint8_t val = (uint8_t)packet->param_value;
		if (val >= 0 && val <= 200) {
			cfg.D8[PIDPOS] = val;
		}
	} else if (strcmp(packet->param_id, "GPS_POSR_P") == 0) {
		uint8_t val = (uint8_t)packet->param_value;
		if (val >= 0 && val <= 200) {
			cfg.P8[PIDPOSR] = val;
		}
	} else if (strcmp(packet->param_id, "GPS_POSR_I") == 0) {
		uint8_t val = (uint8_t)packet->param_value;
		if (val >= 0 && val <= 200) {
			cfg.I8[PIDPOSR] = val;
		}
	} else if (strcmp(packet->param_id, "GPS_POSR_D") == 0) {
		uint8_t val = (uint8_t)packet->param_value;
		if (val >= 0 && val <= 200) {
			cfg.D8[PIDPOSR] = val;
		}
	} else if (strcmp(packet->param_id, "GPS_NAV_P") == 0) {
		uint8_t val = (uint8_t)packet->param_value;
		if (val >= 0 && val <= 200) {
			cfg.P8[PIDNAVR] = val;
		}
	} else if (strcmp(packet->param_id, "GPS_NAV_I") == 0) {
		uint8_t val = (uint8_t)packet->param_value;
		if (val >= 0 && val <= 200) {
			cfg.I8[PIDNAVR] = val;
		}
	} else if (strcmp(packet->param_id, "GPS_NAV_D") == 0) {
		uint8_t val = (uint8_t)packet->param_value;
		if (val >= 0 && val <= 200) {
			cfg.D8[PIDNAVR] = val;
		}
	} else if (strcmp(packet->param_id, "LED_INVERT") == 0) {
		uint8_t val = (uint8_t)packet->param_value;
		if (val >= 0 && val <= 1) {
			cfg.led_invert = val;
		}
	} else if (strcmp(packet->param_id, "ETC_LOOPTIME") == 0) {
		uint16_t val = (uint16_t)packet->param_value;
		if (val >= 0 && val <= 9000) {
			cfg.looptime = val;
		}
	} else if (strcmp(packet->param_id, "ETC_OLDCTRLR") == 0) {
		uint8_t val = (uint8_t)packet->param_value;
		if (val >= 0 && val <= 1) {
			cfg.oldcontroller = val;
		}
	} else if (strcmp(packet->param_id, "PITCH_P") == 0) {
		uint8_t val = (uint8_t)packet->param_value;
		if (val >= 0 && val <= 200) {
			cfg.P8[PITCH] = val;
		}
	} else if (strcmp(packet->param_id, "PITCH_I") == 0) {
		uint8_t val = (uint8_t)packet->param_value;
		if (val >= 0 && val <= 200) {
			cfg.I8[PITCH] = val;
		}
	} else if (strcmp(packet->param_id, "PITCH_D") == 0) {
		uint8_t val = (uint8_t)packet->param_value;
		if (val >= 0 && val <= 200) {
			cfg.D8[PITCH] = val;
		}
	} else if (strcmp(packet->param_id, "ROLL_P") == 0) {
		uint8_t val = (uint8_t)packet->param_value;
		if (val >= 0 && val <= 200) {
			cfg.P8[ROLL] = val;
		}
	} else if (strcmp(packet->param_id, "ROLL_I") == 0) {
		uint8_t val = (uint8_t)packet->param_value;
		if (val >= 0 && val <= 200) {
			cfg.I8[ROLL] = val;
		}
	} else if (strcmp(packet->param_id, "ROLL_D") == 0) {
		uint8_t val = (uint8_t)packet->param_value;
		if (val >= 0 && val <= 200) {
			cfg.D8[ROLL] = val;
		}
	} else if (strcmp(packet->param_id, "YAW_P") == 0) {
		uint8_t val = (uint8_t)packet->param_value;
		if (val >= 0 && val <= 200) {
			cfg.P8[YAW] = val;
		}
	} else if (strcmp(packet->param_id, "YAW_I") == 0) {
		uint8_t val = (uint8_t)packet->param_value;
		if (val >= 0 && val <= 200) {
			cfg.I8[YAW] = val;
		}
	} else if (strcmp(packet->param_id, "YAW_D") == 0) {
		uint8_t val = (uint8_t)packet->param_value;
		if (val >= 0 && val <= 200) {
			cfg.D8[YAW] = val;
		}
	} else if (strcmp(packet->param_id, "ALT_P") == 0) {
		uint8_t val = (uint8_t)packet->param_value;
		if (val >= 0 && val <= 200) {
			cfg.P8[PIDALT] = val;
		}
	} else if (strcmp(packet->param_id, "ALT_I") == 0) {
		uint8_t val = (uint8_t)packet->param_value;
		if (val >= 0 && val <= 200) {
			cfg.I8[PIDALT] = val;
		}
	} else if (strcmp(packet->param_id, "ALT_D") == 0) {
		uint8_t val = (uint8_t)packet->param_value;
		if (val >= 0 && val <= 200) {
			cfg.D8[PIDALT] = val;
		}
	} else if (strcmp(packet->param_id, "LEVEL_P") == 0) {
		uint8_t val = (uint8_t)packet->param_value;
		if (val >= 0 && val <= 200) {
			cfg.P8[PIDLEVEL] = val;
		}
	} else if (strcmp(packet->param_id, "LEVEL_I") == 0) {
		uint8_t val = (uint8_t)packet->param_value;
		if (val >= 0 && val <= 200) {
			cfg.I8[PIDLEVEL] = val;
		}
	} else if (strcmp(packet->param_id, "LEVEL_D") == 0) {
		uint8_t val = (uint8_t)packet->param_value;
		if (val >= 0 && val <= 200) {
			cfg.D8[PIDLEVEL] = val;
		}
	} else if (strcmp(packet->param_id, "RC_AUXCHS") == 0) {
		uint8_t val = (uint8_t)packet->param_value;
		if (val >= 4 && val <= 14) {
			cfg.auxChannels = val;
		}
	} else if (strcmp(packet->param_id, "SONAR_PINOUT") == 0) {
		uint8_t val = (uint8_t)packet->param_value;
		if (val >= 0 && val <= 2) {
			cfg.SONAR_Pinout = val;
		}
	} else if (strcmp(packet->param_id, "SONAR_MIN") == 0) {
		uint8_t val = (uint8_t)packet->param_value;
		if (val >= 0 && val <= 200) {
			cfg.sonar_min = val;
		}
	} else if (strcmp(packet->param_id, "SONAR_MAX") == 0) {
		uint16_t val = (uint16_t)packet->param_value;
		if (val >= 50 && val <= 700) {
			cfg.sonar_max = val;
		}
	} else if (strcmp(packet->param_id, "SONAR_DEBUG") == 0) {
		uint8_t val = (uint8_t)packet->param_value;
		if (val >= 0 && val <= 1) {
			cfg.sonar_debug = val;
		}
	} else if (strcmp(packet->param_id, "SONAR_TILT") == 0) {
		uint8_t val = (uint8_t)packet->param_value;
		if (val >= 10 && val <= 50) {
			cfg.sonar_tilt = val;
		}
	} else if (strcmp(packet->param_id, "ETC_TELEPROTO") == 0) {
		uint8_t val = (uint8_t)packet->param_value;
		if (val >= 0 && val <= 1) {
			cfg.tele_protocol = val;
		}
	} else if (strcmp(packet->param_id, "LED_TYPE") == 0) {
		uint8_t val = (uint8_t)packet->param_value;
		if (val >= 0 && val <= 3) {
			cfg.LED_Type = val;
		}
	} else if (strcmp(packet->param_id, "LED_PINOUT") == 0) {
		uint8_t val = (uint8_t)packet->param_value;
		if (val >= 0 && val <= 1) {
			cfg.LED_Pinout = val;
		}
	} else if (strcmp(packet->param_id, "LED_CTRLCH") == 0) {
		uint8_t val = (uint8_t)packet->param_value;
		if (val >= 1 && val <= 12) {
			cfg.LED_ControlChannel = val;
		}
	} else if (strcmp(packet->param_id, "LED_ARMED") == 0) {
		uint8_t val = (uint8_t)packet->param_value;
		if (val >= 0 && val <= 1) {
			cfg.LED_Armed = val;
		}
	} else if (strcmp(packet->param_id, "LED_TGL_DELAY") == 0) {
		uint16_t val = (uint16_t)packet->param_value;
		if (val >= 0 && val <= 65535) {
			cfg.LED_Toggle_Delay = val;
		}
	} else if (strcmp(packet->param_id, "LED_PTRN1") == 0) {
		uint32_t val = (uint32_t)packet->param_value;
		if (val >= 0 && val <= 0x7FFFFFFF) {
			cfg.LED_Pattern1 = val;
		}
	} else if (strcmp(packet->param_id, "LED_PTRN2") == 0) {
		uint32_t val = (uint32_t)packet->param_value;
		if (val >= 0 && val <= 0x7FFFFFFF) {
			cfg.LED_Pattern2 = val;
		}
	} else if (strcmp(packet->param_id, "LED_PTRN3") == 0) {
		uint32_t val = (uint32_t)packet->param_value;
		if (val >= 0 && val <= 0x7FFFFFFF) {
			cfg.LED_Pattern3 = val;
		}
	} else if (strcmp(packet->param_id, "FEA_PPM") == 0) {
		if (packet->param_value == 1.0) {
			cfg.enabledFeatures |= (1<<0);
		} else {
			cfg.enabledFeatures &= ~(1<<0);
		}
	} else if (strcmp(packet->param_id, "FEA_VBAT") == 0) {
		if (packet->param_value == 1.0) {
			cfg.enabledFeatures |= (1<<1);
		} else {
			cfg.enabledFeatures &= ~(1<<1);
		}
	} else if (strcmp(packet->param_id, "FEA_FLT_ACCCAL") == 0) {
		if (packet->param_value == 1.0) {
			cfg.enabledFeatures |= (1<<2);
		} else {
			cfg.enabledFeatures &= ~(1<<2);
		}
	} else if (strcmp(packet->param_id, "FEA_SPEKTRUM") == 0) {
		if (packet->param_value == 1.0) {
			cfg.enabledFeatures |= (1<<3);
		} else {
			cfg.enabledFeatures &= ~(1<<3);
		}
	} else if (strcmp(packet->param_id, "FEA_MOTOR_STOP") == 0) {
		if (packet->param_value == 1.0) {
			cfg.enabledFeatures |= (1<<4);
		} else {
			cfg.enabledFeatures &= ~(1<<4);
		}
	} else if (strcmp(packet->param_id, "FEA_SERVO_TILT") == 0) {
		if (packet->param_value == 1.0) {
			cfg.enabledFeatures |= (1<<5);
		} else {
			cfg.enabledFeatures &= ~(1<<5);
		}
	} else if (strcmp(packet->param_id, "FEA_GYRO_SMOOT") == 0) {
		if (packet->param_value == 1.0) {
			cfg.enabledFeatures |= (1<<6);
		} else {
			cfg.enabledFeatures &= ~(1<<6);
		}
	} else if (strcmp(packet->param_id, "FEA_LED") == 0) {
		if (packet->param_value == 1.0) {
			cfg.enabledFeatures |= (1<<7);
		} else {
			cfg.enabledFeatures &= ~(1<<7);
		}
	} else if (strcmp(packet->param_id, "FEA_GPS") == 0) {
		if (packet->param_value == 1.0) {
			cfg.enabledFeatures |= (1<<8);
		} else {
			cfg.enabledFeatures &= ~(1<<8);
		}
	} else if (strcmp(packet->param_id, "FEA_FAILSAFE") == 0) {
		if (packet->param_value == 1.0) {
			cfg.enabledFeatures |= (1<<9);
		} else {
			cfg.enabledFeatures &= ~(1<<9);
		}
	} else if (strcmp(packet->param_id, "FEA_SONAR") == 0) {
		if (packet->param_value == 1.0) {
			cfg.enabledFeatures |= (1<<10);
		} else {
			cfg.enabledFeatures &= ~(1<<10);
		}
	} else if (strcmp(packet->param_id, "FEA_TELEMETRY") == 0) {
		if (packet->param_value == 1.0) {
			cfg.enabledFeatures |= (1<<11);
		} else {
			cfg.enabledFeatures &= ~(1<<11);
		}
	} else if (strcmp(packet->param_id, "FEA_PASS") == 0) {
		if (packet->param_value == 1.0) {
			cfg.enabledFeatures |= (1<<12);
		} else {
			cfg.enabledFeatures &= ~(1<<12);
		}
	} else if (strcmp(packet->param_id, "FEA_POWERMETER") == 0) {
		if (packet->param_value == 1.0) {
			cfg.enabledFeatures |= (1<<13);
		} else {
			cfg.enabledFeatures &= ~(1<<13);
		}
	} else if (strcmp(packet->param_id, "FEA_LCD") == 0) {
		if (packet->param_value == 1.0) {
			cfg.enabledFeatures |= (1<<14);
		} else {
			cfg.enabledFeatures &= ~(1<<14);
		}
	} else if (strcmp(packet->param_id, "MIXER") == 0) {
		cfg.mixerConfiguration = (uint8_t)packet->param_value;
	} else if (strcmp(packet->param_id, "MAP_ROLL") == 0) {
		cfg.rcmap[0] = (uint8_t)packet->param_value;
	} else if (strcmp(packet->param_id, "MAP_PITCH") == 0) {
		cfg.rcmap[1] = (uint8_t)packet->param_value;
	} else if (strcmp(packet->param_id, "MAP_YAW") == 0) {
		cfg.rcmap[2] = (uint8_t)packet->param_value;
	} else if (strcmp(packet->param_id, "MAP_THROTTLE") == 0) {
		cfg.rcmap[3] = (uint8_t)packet->param_value;
	} else if (strcmp(packet->param_id, "MAP_AUX1") == 0) {
		cfg.rcmap[4] = (uint8_t)packet->param_value;
	} else if (strcmp(packet->param_id, "MAP_AUX2") == 0) {
		cfg.rcmap[5] = (uint8_t)packet->param_value;
	} else if (strcmp(packet->param_id, "MAP_AUX3") == 0) {
		cfg.rcmap[6] = (uint8_t)packet->param_value;
	} else if (strcmp(packet->param_id, "MAP_AUX4") == 0) {
		cfg.rcmap[7] = (uint8_t)packet->param_value;
	} else if (strcmp(packet->param_id, "BOX_ANGLE") == 0) {
		cfg.activate[0] = (uint32_t)packet->param_value;
	} else if (strcmp(packet->param_id, "BOX_HORIZON") == 0) {
		cfg.activate[1] = (uint32_t)packet->param_value;
	} else if (strcmp(packet->param_id, "BOX_BARO") == 0) {
		cfg.activate[2] = (uint32_t)packet->param_value;
	} else if (strcmp(packet->param_id, "BOX_MAG") == 0) {
		cfg.activate[3] = (uint32_t)packet->param_value;
	} else if (strcmp(packet->param_id, "BOX_CAMSTAB") == 0) {
		cfg.activate[4] = (uint32_t)packet->param_value;
	} else if (strcmp(packet->param_id, "BOX_CAMTRIG") == 0) {
		cfg.activate[5] = (uint32_t)packet->param_value;
	} else if (strcmp(packet->param_id, "BOX_ARM") == 0) {
		cfg.activate[6] = (uint32_t)packet->param_value;
	} else if (strcmp(packet->param_id, "BOX_GPSHOME") == 0) {
		cfg.activate[7] = (uint32_t)packet->param_value;
	} else if (strcmp(packet->param_id, "BOX_GPSHOLD") == 0) {
		cfg.activate[8] = (uint32_t)packet->param_value;
	} else if (strcmp(packet->param_id, "BOX_PASSTHRU") == 0) {
		cfg.activate[9] = (uint32_t)packet->param_value;
	} else if (strcmp(packet->param_id, "BOX_HEADFREE") == 0) {
		cfg.activate[10] = (uint32_t)packet->param_value;
	} else if (strcmp(packet->param_id, "BOX_BEEPERON") == 0) {
		cfg.activate[11] = (uint32_t)packet->param_value;
	} else if (strcmp(packet->param_id, "BOX_LEDMAX") == 0) {
		cfg.activate[12] = (uint32_t)packet->param_value;
	} else if (strcmp(packet->param_id, "BOX_LLIGHTS") == 0) {
		cfg.activate[13] = (uint32_t)packet->param_value;
	} else if (strcmp(packet->param_id, "BOX_HEADADJ") == 0) {
		cfg.activate[14] = (uint32_t)packet->param_value;
	}
}

uint8_t baseflight_mavlink_send_param (uint8_t num) {
	mavlink_message_t msg;
	switch (num) {
		case 1: {
			mavlink_msg_param_value_pack(1, 200, &msg, "RC_DBAND", (float)cfg.deadband, 1, 186, num - 1);
			baseflight_mavlink_send_message(&msg);
			break;
		}
		case 2: {
			mavlink_msg_param_value_pack(1, 200, &msg, "YAW_DBAND", (float)cfg.yawdeadband, 2, 186, num - 1);
			baseflight_mavlink_send_message(&msg);
			break;
		}
		case 3: {
			mavlink_msg_param_value_pack(1, 200, &msg, "THR_ALTHLD_NTL", (float)cfg.alt_hold_throttle_neutral, 3, 186, num - 1);
			baseflight_mavlink_send_message(&msg);
			break;
		}
		case 4: {
			mavlink_msg_param_value_pack(1, 200, &msg, "NAV_PHDBAND", (float)cfg.phdeadband, 4, 186, num - 1);
			baseflight_mavlink_send_message(&msg);
			break;
		}
		case 5: {
			mavlink_msg_param_value_pack(1, 200, &msg, "RC_MID", (float)cfg.midrc, 5, 186, num - 1);
			baseflight_mavlink_send_message(&msg);
			break;
		}
		case 6: {
			mavlink_msg_param_value_pack(1, 200, &msg, "RC_RATE", (float)cfg.rcRate8, 6, 186, num - 1);
			baseflight_mavlink_send_message(&msg);
			break;
		}
		case 7: {
			mavlink_msg_param_value_pack(1, 200, &msg, "RC_EXPO", (float)cfg.rcExpo8, 7, 186, num - 1);
			baseflight_mavlink_send_message(&msg);
			break;
		}
		case 8: {
			mavlink_msg_param_value_pack(1, 200, &msg, "THR_MID", (float)cfg.thrMid8, 8, 186, num - 1);
			baseflight_mavlink_send_message(&msg);
			break;
		}
		case 9: {
			mavlink_msg_param_value_pack(1, 200, &msg, "THR_EXPO", (float)cfg.thrExpo8, 9, 186, num - 1);
			baseflight_mavlink_send_message(&msg);
			break;
		}
		case 10: {
			mavlink_msg_param_value_pack(1, 200, &msg, "ROLL_PITCH_RATE", (float)cfg.rollPitchRate, 10, 186, num - 1);
			baseflight_mavlink_send_message(&msg);
			break;
		}
		case 11: {
			mavlink_msg_param_value_pack(1, 200, &msg, "YAW_RATE", (float)cfg.yawRate, 11, 186, num - 1);
			baseflight_mavlink_send_message(&msg);
			break;
		}
		case 12: {
			mavlink_msg_param_value_pack(1, 200, &msg, "THR_MIN", (float)cfg.minthrottle, 12, 186, num - 1);
			baseflight_mavlink_send_message(&msg);
			break;
		}
		case 13: {
			mavlink_msg_param_value_pack(1, 200, &msg, "THR_MAX", (float)cfg.maxthrottle, 13, 186, num - 1);
			baseflight_mavlink_send_message(&msg);
			break;
		}
		case 14: {
			mavlink_msg_param_value_pack(1, 200, &msg, "ETC_PASSMOTOR", (float)cfg.passmotor, 14, 186, num - 1);
			baseflight_mavlink_send_message(&msg);
			break;
		}
		case 15: {
			mavlink_msg_param_value_pack(1, 200, &msg, "RC_MINCMD", (float)cfg.mincommand, 15, 186, num - 1);
			baseflight_mavlink_send_message(&msg);
			break;
		}
		case 16: {
			mavlink_msg_param_value_pack(1, 200, &msg, "RC_MINCHK", (float)cfg.mincheck, 16, 186, num - 1);
			baseflight_mavlink_send_message(&msg);
			break;
		}
		case 17: {
			mavlink_msg_param_value_pack(1, 200, &msg, "RC_MAXCHK", (float)cfg.maxcheck, 17, 186, num - 1);
			baseflight_mavlink_send_message(&msg);
			break;
		}
		case 18: {
			mavlink_msg_param_value_pack(1, 200, &msg, "ETC_RTA_ARM", (float)cfg.retarded_arm, 18, 186, num - 1);
			baseflight_mavlink_send_message(&msg);
			break;
		}
		case 19: {
			mavlink_msg_param_value_pack(1, 200, &msg, "RC_KILLSWIME", (float)cfg.killswitchtime, 19, 186, num - 1);
			baseflight_mavlink_send_message(&msg);
			break;
		}
		case 20: {
			mavlink_msg_param_value_pack(1, 200, &msg, "RC_FS_DELAY", (float)cfg.failsafe_delay, 20, 186, num - 1);
			baseflight_mavlink_send_message(&msg);
			break;
		}
		case 21: {
			mavlink_msg_param_value_pack(1, 200, &msg, "RC_FS_OFF_DELAY", (float)cfg.failsafe_off_delay, 21, 186, num - 1);
			baseflight_mavlink_send_message(&msg);
			break;
		}
		case 22: {
			mavlink_msg_param_value_pack(1, 200, &msg, "RC_FS_THR", (float)cfg.failsafe_throttle, 22, 186, num - 1);
			baseflight_mavlink_send_message(&msg);
			break;
		}
		case 23: {
			mavlink_msg_param_value_pack(1, 200, &msg, "RC_FS_DEADPILOT", (float)cfg.failsafe_deadpilot, 23, 186, num - 1);
			baseflight_mavlink_send_message(&msg);
			break;
		}
		case 24: {
			mavlink_msg_param_value_pack(1, 200, &msg, "RC_FS_JUSTPH", (float)cfg.failsafe_justph, 24, 186, num - 1);
			baseflight_mavlink_send_message(&msg);
			break;
		}
		case 25: {
			mavlink_msg_param_value_pack(1, 200, &msg, "ETC_MOTOR_PWMR", (float)cfg.motor_pwm_rate, 25, 186, num - 1);
			baseflight_mavlink_send_message(&msg);
			break;
		}
		case 26: {
			mavlink_msg_param_value_pack(1, 200, &msg, "ETC_SERVO_PWMR", (float)cfg.servo_pwm_rate, 26, 186, num - 1);
			baseflight_mavlink_send_message(&msg);
			break;
		}
		case 27: {
			mavlink_msg_param_value_pack(1, 200, &msg, "ETC_SERIAL_BAUD", (float)cfg.serial_baudrate, 27, 186, num - 1);
			baseflight_mavlink_send_message(&msg);
			break;
		}
		case 28: {
			mavlink_msg_param_value_pack(1, 200, &msg, "RC_SPTR_HIRES", (float)cfg.spektrum_hires, 28, 186, num - 1);
			baseflight_mavlink_send_message(&msg);
			break;
		}
		case 29: {
			mavlink_msg_param_value_pack(1, 200, &msg, "VBAT_SCALE", (float)cfg.vbatscale, 29, 186, num - 1);
			baseflight_mavlink_send_message(&msg);
			break;
		}
		case 30: {
			mavlink_msg_param_value_pack(1, 200, &msg, "VBAT_MAXCLVOLT", (float)cfg.vbatmaxcellvoltage, 30, 186, num - 1);
			baseflight_mavlink_send_message(&msg);
			break;
		}
		case 31: {
			mavlink_msg_param_value_pack(1, 200, &msg, "VBAT_MINCLVOLT", (float)cfg.vbatmincellvoltage, 31, 186, num - 1);
			baseflight_mavlink_send_message(&msg);
			break;
		}
		case 32: {
			mavlink_msg_param_value_pack(1, 200, &msg, "ETC_PWR_ADC", (float)cfg.power_adc_channel, 32, 186, num - 1);
			baseflight_mavlink_send_message(&msg);
			break;
		}
		case 33: {
			mavlink_msg_param_value_pack(1, 200, &msg, "YAW_DIR", (float)cfg.yaw_direction, 33, 186, num - 1);
			baseflight_mavlink_send_message(&msg);
			break;
		}
		case 34: {
			mavlink_msg_param_value_pack(1, 200, &msg, "TRI_YAW_MIDDLE", (float)cfg.tri_yaw_middle, 34, 186, num - 1);
			baseflight_mavlink_send_message(&msg);
			break;
		}
		case 35: {
			mavlink_msg_param_value_pack(1, 200, &msg, "TRI_YAW_MIN", (float)cfg.tri_yaw_min, 35, 186, num - 1);
			baseflight_mavlink_send_message(&msg);
			break;
		}
		case 36: {
			mavlink_msg_param_value_pack(1, 200, &msg, "TRI_YAW_MAX", (float)cfg.tri_yaw_max, 36, 186, num - 1);
			baseflight_mavlink_send_message(&msg);
			break;
		}
		case 37: {
			mavlink_msg_param_value_pack(1, 200, &msg, "WING_LEFT_MIN", (float)cfg.wing_left_min, 37, 186, num - 1);
			baseflight_mavlink_send_message(&msg);
			break;
		}
		case 38: {
			mavlink_msg_param_value_pack(1, 200, &msg, "WING_LEFT_MID", (float)cfg.wing_left_mid, 38, 186, num - 1);
			baseflight_mavlink_send_message(&msg);
			break;
		}
		case 39: {
			mavlink_msg_param_value_pack(1, 200, &msg, "WING_LEFT_MAX", (float)cfg.wing_left_max, 39, 186, num - 1);
			baseflight_mavlink_send_message(&msg);
			break;
		}
		case 40: {
			mavlink_msg_param_value_pack(1, 200, &msg, "WING_RIGHT_MIN", (float)cfg.wing_right_min, 40, 186, num - 1);
			baseflight_mavlink_send_message(&msg);
			break;
		}
		case 41: {
			mavlink_msg_param_value_pack(1, 200, &msg, "WING_RIGHT_MID", (float)cfg.wing_right_mid, 41, 186, num - 1);
			baseflight_mavlink_send_message(&msg);
			break;
		}
		case 42: {
			mavlink_msg_param_value_pack(1, 200, &msg, "WING_RIGHT_MAX", (float)cfg.wing_right_max, 42, 186, num - 1);
			baseflight_mavlink_send_message(&msg);
			break;
		}
		case 43: {
			mavlink_msg_param_value_pack(1, 200, &msg, "PITCH_DIR_L", (float)cfg.pitch_direction_l, 43, 186, num - 1);
			baseflight_mavlink_send_message(&msg);
			break;
		}
		case 44: {
			mavlink_msg_param_value_pack(1, 200, &msg, "PITCH_DIR_R", (float)cfg.pitch_direction_r, 44, 186, num - 1);
			baseflight_mavlink_send_message(&msg);
			break;
		}
		case 45: {
			mavlink_msg_param_value_pack(1, 200, &msg, "ROLL_DIR_L", (float)cfg.roll_direction_l, 45, 186, num - 1);
			baseflight_mavlink_send_message(&msg);
			break;
		}
		case 46: {
			mavlink_msg_param_value_pack(1, 200, &msg, "ROLL_DIR_R", (float)cfg.roll_direction_r, 46, 186, num - 1);
			baseflight_mavlink_send_message(&msg);
			break;
		}
		case 47: {
			mavlink_msg_param_value_pack(1, 200, &msg, "GMBL_FLAGS", (float)cfg.gimbal_flags, 47, 186, num - 1);
			baseflight_mavlink_send_message(&msg);
			break;
		}
		case 48: {
			mavlink_msg_param_value_pack(1, 200, &msg, "GMBL_PITCH_GAIN", (float)cfg.gimbal_pitch_gain, 48, 186, num - 1);
			baseflight_mavlink_send_message(&msg);
			break;
		}
		case 49: {
			mavlink_msg_param_value_pack(1, 200, &msg, "GMBL_ROLL_GAIN", (float)cfg.gimbal_roll_gain, 49, 186, num - 1);
			baseflight_mavlink_send_message(&msg);
			break;
		}
		case 50: {
			mavlink_msg_param_value_pack(1, 200, &msg, "GMBL_PITCH_MIN", (float)cfg.gimbal_pitch_min, 50, 186, num - 1);
			baseflight_mavlink_send_message(&msg);
			break;
		}
		case 51: {
			mavlink_msg_param_value_pack(1, 200, &msg, "GMBL_PITCH_MAX", (float)cfg.gimbal_pitch_max, 51, 186, num - 1);
			baseflight_mavlink_send_message(&msg);
			break;
		}
		case 52: {
			mavlink_msg_param_value_pack(1, 200, &msg, "GMBL_PITCH_MID", (float)cfg.gimbal_pitch_mid, 52, 186, num - 1);
			baseflight_mavlink_send_message(&msg);
			break;
		}
		case 53: {
			mavlink_msg_param_value_pack(1, 200, &msg, "GMBL_ROLL_MIN", (float)cfg.gimbal_roll_min, 53, 186, num - 1);
			baseflight_mavlink_send_message(&msg);
			break;
		}
		case 54: {
			mavlink_msg_param_value_pack(1, 200, &msg, "GMBL_ROLL_MAX", (float)cfg.gimbal_roll_max, 54, 186, num - 1);
			baseflight_mavlink_send_message(&msg);
			break;
		}
		case 55: {
			mavlink_msg_param_value_pack(1, 200, &msg, "GMBL_ROLL_MID", (float)cfg.gimbal_roll_mid, 55, 186, num - 1);
			baseflight_mavlink_send_message(&msg);
			break;
		}
		case 56: {
			mavlink_msg_param_value_pack(1, 200, &msg, "ETC_AL_RATE", (float)cfg.autolandrate, 56, 186, num - 1);
			baseflight_mavlink_send_message(&msg);
			break;
		}
		case 57: {
			mavlink_msg_param_value_pack(1, 200, &msg, "GYRO_X_ALIGN", (float)cfg.align[ALIGN_GYRO][0], 57, 186, num - 1);
			baseflight_mavlink_send_message(&msg);
			break;
		}
		case 58: {
			mavlink_msg_param_value_pack(1, 200, &msg, "GYRO_Y_ALIGN", (float)cfg.align[ALIGN_GYRO][1], 58, 186, num - 1);
			baseflight_mavlink_send_message(&msg);
			break;
		}
		case 59: {
			mavlink_msg_param_value_pack(1, 200, &msg, "GYRO_Z_ALIGN", (float)cfg.align[ALIGN_GYRO][2], 59, 186, num - 1);
			baseflight_mavlink_send_message(&msg);
			break;
		}
		case 60: {
			mavlink_msg_param_value_pack(1, 200, &msg, "ACC_X_ALIGN", (float)cfg.align[ALIGN_ACCEL][0], 60, 186, num - 1);
			baseflight_mavlink_send_message(&msg);
			break;
		}
		case 61: {
			mavlink_msg_param_value_pack(1, 200, &msg, "ACC_Y_ALIGN", (float)cfg.align[ALIGN_ACCEL][1], 61, 186, num - 1);
			baseflight_mavlink_send_message(&msg);
			break;
		}
		case 62: {
			mavlink_msg_param_value_pack(1, 200, &msg, "ACC_Z_ALIGN", (float)cfg.align[ALIGN_ACCEL][2], 62, 186, num - 1);
			baseflight_mavlink_send_message(&msg);
			break;
		}
		case 63: {
			mavlink_msg_param_value_pack(1, 200, &msg, "MAG_X_ALIGN", (float)cfg.align[ALIGN_MAG][0], 63, 186, num - 1);
			baseflight_mavlink_send_message(&msg);
			break;
		}
		case 64: {
			mavlink_msg_param_value_pack(1, 200, &msg, "MAG_Y_ALIGN", (float)cfg.align[ALIGN_MAG][1], 64, 186, num - 1);
			baseflight_mavlink_send_message(&msg);
			break;
		}
		case 65: {
			mavlink_msg_param_value_pack(1, 200, &msg, "MAG_Z_ALIGN", (float)cfg.align[ALIGN_MAG][2], 65, 186, num - 1);
			baseflight_mavlink_send_message(&msg);
			break;
		}
		case 66: {
			mavlink_msg_param_value_pack(1, 200, &msg, "ACC_HARDWARE", (float)cfg.acc_hardware, 66, 186, num - 1);
			baseflight_mavlink_send_message(&msg);
			break;
		}
		case 67: {
			mavlink_msg_param_value_pack(1, 200, &msg, "ACC_LPF_FACTOR", (float)cfg.acc_lpf_factor, 67, 186, num - 1);
			baseflight_mavlink_send_message(&msg);
			break;
		}
		case 68: {
			mavlink_msg_param_value_pack(1, 200, &msg, "ACC_INS_LPF", (float)cfg.acc_ins_lpf, 68, 186, num - 1);
			baseflight_mavlink_send_message(&msg);
			break;
		}
		case 69: {
			mavlink_msg_param_value_pack(1, 200, &msg, "ACC_TRIM_PITCH", (float)cfg.angleTrim[PITCH], 69, 186, num - 1);
			baseflight_mavlink_send_message(&msg);
			break;
		}
		case 70: {
			mavlink_msg_param_value_pack(1, 200, &msg, "ACC_TRIM_ROLL", (float)cfg.angleTrim[ROLL], 70, 186, num - 1);
			baseflight_mavlink_send_message(&msg);
			break;
		}
		case 71: {
			mavlink_msg_param_value_pack(1, 200, &msg, "GYRO_LPF", (float)cfg.gyro_lpf, 71, 186, num - 1);
			baseflight_mavlink_send_message(&msg);
			break;
		}
		case 72: {
			mavlink_msg_param_value_pack(1, 200, &msg, "GYRO_CMPF_FACTOR", (float)cfg.gyro_cmpf_factor, 72, 186, num - 1);
			baseflight_mavlink_send_message(&msg);
			break;
		}
		case 73: {
			mavlink_msg_param_value_pack(1, 200, &msg, "ACC_Z_VEL_CF", (float)cfg.accz_vel_cf, 73, 186, num - 1);
			baseflight_mavlink_send_message(&msg);
			break;
		}
		case 74: {
			mavlink_msg_param_value_pack(1, 200, &msg, "ACC_Z_ALT_CF", (float)cfg.accz_alt_cf, 74, 186, num - 1);
			baseflight_mavlink_send_message(&msg);
			break;
		}
		case 75: {
			mavlink_msg_param_value_pack(1, 200, &msg, "BARO_LAG", (float)cfg.baro_lag, 75, 186, num - 1);
			baseflight_mavlink_send_message(&msg);
			break;
		}
		case 76: {
			mavlink_msg_param_value_pack(1, 200, &msg, "BARO_SONAR_CF", (float)cfg.baro_sonar_cf, 76, 186, num - 1);
			baseflight_mavlink_send_message(&msg);
			break;
		}
		case 77: {
			mavlink_msg_param_value_pack(1, 200, &msg, "BARO_DOWNSCALE", (float)cfg.barodownscale, 77, 186, num - 1);
			baseflight_mavlink_send_message(&msg);
			break;
		}
		case 78: {
			mavlink_msg_param_value_pack(1, 200, &msg, "ETC_NAZEDBG", (float)cfg.nazedebug, 78, 186, num - 1);
			baseflight_mavlink_send_message(&msg);
			break;
		}
		case 79: {
			mavlink_msg_param_value_pack(1, 200, &msg, "ETC_MORON_THRH", (float)cfg.moron_threshold, 79, 186, num - 1);
			baseflight_mavlink_send_message(&msg);
			break;
		}
		case 80: {
			mavlink_msg_param_value_pack(1, 200, &msg, "MAG_DECL", (float)cfg.mag_declination, 80, 186, num - 1);
			baseflight_mavlink_send_message(&msg);
			break;
		}
		case 81: {
			mavlink_msg_param_value_pack(1, 200, &msg, "MAG_OLDCALIB", (float)cfg.mag_oldcalib, 81, 186, num - 1);
			baseflight_mavlink_send_message(&msg);
			break;
		}
		case 82: {
			mavlink_msg_param_value_pack(1, 200, &msg, "GPS_BRATE", (float)cfg.gps_baudrate, 82, 186, num - 1);
			baseflight_mavlink_send_message(&msg);
			break;
		}
		case 83: {
			mavlink_msg_param_value_pack(1, 200, &msg, "GPS_TYPE", (float)cfg.gps_type, 83, 186, num - 1);
			baseflight_mavlink_send_message(&msg);
			break;
		}
		case 84: {
			mavlink_msg_param_value_pack(1, 200, &msg, "GPS_INS_VEL", (float)cfg.gps_ins_vel, 84, 186, num - 1);
			baseflight_mavlink_send_message(&msg);
			break;
		}
		case 85: {
			mavlink_msg_param_value_pack(1, 200, &msg, "GPS_LAG", (float)cfg.gps_lag, 85, 186, num - 1);
			baseflight_mavlink_send_message(&msg);
			break;
		}
		case 86: {
			mavlink_msg_param_value_pack(1, 200, &msg, "GPS_SPEEDFILTER", (float)cfg.gps_speedfilter, 86, 186, num - 1);
			baseflight_mavlink_send_message(&msg);
			break;
		}
		case 87: {
			mavlink_msg_param_value_pack(1, 200, &msg, "GPS_PHASE", (float)cfg.gps_phase, 87, 186, num - 1);
			baseflight_mavlink_send_message(&msg);
			break;
		}
		case 88: {
			mavlink_msg_param_value_pack(1, 200, &msg, "GPS_PH_MINSAT", (float)cfg.gps_ph_minsat, 88, 186, num - 1);
			baseflight_mavlink_send_message(&msg);
			break;
		}
		case 89: {
			mavlink_msg_param_value_pack(1, 200, &msg, "GPS_PH_APM", (float)cfg.gps_ph_apm, 89, 186, num - 1);
			baseflight_mavlink_send_message(&msg);
			break;
		}
		case 90: {
			mavlink_msg_param_value_pack(1, 200, &msg, "GPS_PH_STLSPD", (float)cfg.gps_ph_settlespeed, 90, 186, num - 1);
			baseflight_mavlink_send_message(&msg);
			break;
		}
		case 91: {
			mavlink_msg_param_value_pack(1, 200, &msg, "GPS_PH_TRGTSQRT", (float)cfg.gps_ph_targetsqrt, 91, 186, num - 1);
			baseflight_mavlink_send_message(&msg);
			break;
		}
		case 92: {
			mavlink_msg_param_value_pack(1, 200, &msg, "GPS_PHMOVE_SPD", (float)cfg.gps_phmove_speed, 92, 186, num - 1);
			baseflight_mavlink_send_message(&msg);
			break;
		}
		case 93: {
			mavlink_msg_param_value_pack(1, 200, &msg, "GPS_MAXANG", (float)cfg.gps_maxangle, 93, 186, num - 1);
			baseflight_mavlink_send_message(&msg);
			break;
		}
		case 94: {
			mavlink_msg_param_value_pack(1, 200, &msg, "GPS_MINANGPRC", (float)cfg.gps_minanglepercent, 94, 186, num - 1);
			baseflight_mavlink_send_message(&msg);
			break;
		}
		case 95: {
			mavlink_msg_param_value_pack(1, 200, &msg, "GPS_WP_RADIUS", (float)cfg.gps_wp_radius, 95, 186, num - 1);
			baseflight_mavlink_send_message(&msg);
			break;
		}
		case 96: {
			mavlink_msg_param_value_pack(1, 200, &msg, "GPS_RTL_MINHGT", (float)cfg.gps_rtl_minhight, 96, 186, num - 1);
			baseflight_mavlink_send_message(&msg);
			break;
		}
		case 97: {
			mavlink_msg_param_value_pack(1, 200, &msg, "GPS_RTL_MINDIST", (float)cfg.gps_rtl_mindist, 97, 186, num - 1);
			baseflight_mavlink_send_message(&msg);
			break;
		}
		case 98: {
			mavlink_msg_param_value_pack(1, 200, &msg, "GPS_RTL_FLYAWAY", (float)cfg.gps_rtl_flyaway, 98, 186, num - 1);
			baseflight_mavlink_send_message(&msg);
			break;
		}
		case 99: {
			mavlink_msg_param_value_pack(1, 200, &msg, "GPS_YAW", (float)cfg.gps_yaw, 99, 186, num - 1);
			baseflight_mavlink_send_message(&msg);
			break;
		}
		case 100: {
			mavlink_msg_param_value_pack(1, 200, &msg, "NAV_RTL_LASTTRN", (float)cfg.nav_rtl_lastturn, 100, 186, num - 1);
			baseflight_mavlink_send_message(&msg);
			break;
		}
		case 101: {
			mavlink_msg_param_value_pack(1, 200, &msg, "NAV_SPEED_MIN", (float)cfg.nav_speed_min, 101, 186, num - 1);
			baseflight_mavlink_send_message(&msg);
			break;
		}
		case 102: {
			mavlink_msg_param_value_pack(1, 200, &msg, "NAV_SPEED_MAX", (float)cfg.nav_speed_max, 102, 186, num - 1);
			baseflight_mavlink_send_message(&msg);
			break;
		}
		case 103: {
			mavlink_msg_param_value_pack(1, 200, &msg, "NAV_SLEW_RATE", (float)cfg.nav_slew_rate, 103, 186, num - 1);
			baseflight_mavlink_send_message(&msg);
			break;
		}
		case 104: {
			mavlink_msg_param_value_pack(1, 200, &msg, "NAV_CTRL_HEADNG", (float)cfg.nav_controls_heading, 104, 186, num - 1);
			baseflight_mavlink_send_message(&msg);
			break;
		}
		case 105: {
			mavlink_msg_param_value_pack(1, 200, &msg, "NAV_TAIL_FIRST", (float)cfg.nav_tail_first, 105, 186, num - 1);
			baseflight_mavlink_send_message(&msg);
			break;
		}
		case 106: {
			mavlink_msg_param_value_pack(1, 200, &msg, "GPS_POS_P", (float)cfg.P8[PIDPOS], 106, 186, num - 1);
			baseflight_mavlink_send_message(&msg);
			break;
		}
		case 107: {
			mavlink_msg_param_value_pack(1, 200, &msg, "GPS_POS_I", (float)cfg.I8[PIDPOS], 107, 186, num - 1);
			baseflight_mavlink_send_message(&msg);
			break;
		}
		case 108: {
			mavlink_msg_param_value_pack(1, 200, &msg, "GPS_POS_D", (float)cfg.D8[PIDPOS], 108, 186, num - 1);
			baseflight_mavlink_send_message(&msg);
			break;
		}
		case 109: {
			mavlink_msg_param_value_pack(1, 200, &msg, "GPS_POSR_P", (float)cfg.P8[PIDPOSR], 109, 186, num - 1);
			baseflight_mavlink_send_message(&msg);
			break;
		}
		case 110: {
			mavlink_msg_param_value_pack(1, 200, &msg, "GPS_POSR_I", (float)cfg.I8[PIDPOSR], 110, 186, num - 1);
			baseflight_mavlink_send_message(&msg);
			break;
		}
		case 111: {
			mavlink_msg_param_value_pack(1, 200, &msg, "GPS_POSR_D", (float)cfg.D8[PIDPOSR], 111, 186, num - 1);
			baseflight_mavlink_send_message(&msg);
			break;
		}
		case 112: {
			mavlink_msg_param_value_pack(1, 200, &msg, "GPS_NAV_P", (float)cfg.P8[PIDNAVR], 112, 186, num - 1);
			baseflight_mavlink_send_message(&msg);
			break;
		}
		case 113: {
			mavlink_msg_param_value_pack(1, 200, &msg, "GPS_NAV_I", (float)cfg.I8[PIDNAVR], 113, 186, num - 1);
			baseflight_mavlink_send_message(&msg);
			break;
		}
		case 114: {
			mavlink_msg_param_value_pack(1, 200, &msg, "GPS_NAV_D", (float)cfg.D8[PIDNAVR], 114, 186, num - 1);
			baseflight_mavlink_send_message(&msg);
			break;
		}
		case 115: {
			mavlink_msg_param_value_pack(1, 200, &msg, "LED_INVERT", (float)cfg.led_invert, 115, 186, num - 1);
			baseflight_mavlink_send_message(&msg);
			break;
		}
		case 116: {
			mavlink_msg_param_value_pack(1, 200, &msg, "ETC_LOOPTIME", (float)cfg.looptime, 116, 186, num - 1);
			baseflight_mavlink_send_message(&msg);
			break;
		}
		case 117: {
			mavlink_msg_param_value_pack(1, 200, &msg, "ETC_OLDCTRLR", (float)cfg.oldcontroller, 117, 186, num - 1);
			baseflight_mavlink_send_message(&msg);
			break;
		}
		case 118: {
			mavlink_msg_param_value_pack(1, 200, &msg, "PITCH_P", (float)cfg.P8[PITCH], 118, 186, num - 1);
			baseflight_mavlink_send_message(&msg);
			break;
		}
		case 119: {
			mavlink_msg_param_value_pack(1, 200, &msg, "PITCH_I", (float)cfg.I8[PITCH], 119, 186, num - 1);
			baseflight_mavlink_send_message(&msg);
			break;
		}
		case 120: {
			mavlink_msg_param_value_pack(1, 200, &msg, "PITCH_D", (float)cfg.D8[PITCH], 120, 186, num - 1);
			baseflight_mavlink_send_message(&msg);
			break;
		}
		case 121: {
			mavlink_msg_param_value_pack(1, 200, &msg, "ROLL_P", (float)cfg.P8[ROLL], 121, 186, num - 1);
			baseflight_mavlink_send_message(&msg);
			break;
		}
		case 122: {
			mavlink_msg_param_value_pack(1, 200, &msg, "ROLL_I", (float)cfg.I8[ROLL], 122, 186, num - 1);
			baseflight_mavlink_send_message(&msg);
			break;
		}
		case 123: {
			mavlink_msg_param_value_pack(1, 200, &msg, "ROLL_D", (float)cfg.D8[ROLL], 123, 186, num - 1);
			baseflight_mavlink_send_message(&msg);
			break;
		}
		case 124: {
			mavlink_msg_param_value_pack(1, 200, &msg, "YAW_P", (float)cfg.P8[YAW], 124, 186, num - 1);
			baseflight_mavlink_send_message(&msg);
			break;
		}
		case 125: {
			mavlink_msg_param_value_pack(1, 200, &msg, "YAW_I", (float)cfg.I8[YAW], 125, 186, num - 1);
			baseflight_mavlink_send_message(&msg);
			break;
		}
		case 126: {
			mavlink_msg_param_value_pack(1, 200, &msg, "YAW_D", (float)cfg.D8[YAW], 126, 186, num - 1);
			baseflight_mavlink_send_message(&msg);
			break;
		}
		case 127: {
			mavlink_msg_param_value_pack(1, 200, &msg, "ALT_P", (float)cfg.P8[PIDALT], 127, 186, num - 1);
			baseflight_mavlink_send_message(&msg);
			break;
		}
		case 128: {
			mavlink_msg_param_value_pack(1, 200, &msg, "ALT_I", (float)cfg.I8[PIDALT], 128, 186, num - 1);
			baseflight_mavlink_send_message(&msg);
			break;
		}
		case 129: {
			mavlink_msg_param_value_pack(1, 200, &msg, "ALT_D", (float)cfg.D8[PIDALT], 129, 186, num - 1);
			baseflight_mavlink_send_message(&msg);
			break;
		}
		case 130: {
			mavlink_msg_param_value_pack(1, 200, &msg, "LEVEL_P", (float)cfg.P8[PIDLEVEL], 130, 186, num - 1);
			baseflight_mavlink_send_message(&msg);
			break;
		}
		case 131: {
			mavlink_msg_param_value_pack(1, 200, &msg, "LEVEL_I", (float)cfg.I8[PIDLEVEL], 131, 186, num - 1);
			baseflight_mavlink_send_message(&msg);
			break;
		}
		case 132: {
			mavlink_msg_param_value_pack(1, 200, &msg, "LEVEL_D", (float)cfg.D8[PIDLEVEL], 132, 186, num - 1);
			baseflight_mavlink_send_message(&msg);
			break;
		}
		case 133: {
			mavlink_msg_param_value_pack(1, 200, &msg, "RC_AUXCHS", (float)cfg.auxChannels, 133, 186, num - 1);
			baseflight_mavlink_send_message(&msg);
			break;
		}
		case 134: {
			mavlink_msg_param_value_pack(1, 200, &msg, "SONAR_PINOUT", (float)cfg.SONAR_Pinout, 134, 186, num - 1);
			baseflight_mavlink_send_message(&msg);
			break;
		}
		case 135: {
			mavlink_msg_param_value_pack(1, 200, &msg, "SONAR_MIN", (float)cfg.sonar_min, 135, 186, num - 1);
			baseflight_mavlink_send_message(&msg);
			break;
		}
		case 136: {
			mavlink_msg_param_value_pack(1, 200, &msg, "SONAR_MAX", (float)cfg.sonar_max, 136, 186, num - 1);
			baseflight_mavlink_send_message(&msg);
			break;
		}
		case 137: {
			mavlink_msg_param_value_pack(1, 200, &msg, "SONAR_DEBUG", (float)cfg.sonar_debug, 137, 186, num - 1);
			baseflight_mavlink_send_message(&msg);
			break;
		}
		case 138: {
			mavlink_msg_param_value_pack(1, 200, &msg, "SONAR_TILT", (float)cfg.sonar_tilt, 138, 186, num - 1);
			baseflight_mavlink_send_message(&msg);
			break;
		}
		case 139: {
			mavlink_msg_param_value_pack(1, 200, &msg, "ETC_TELEPROTO", (float)cfg.tele_protocol, 139, 186, num - 1);
			baseflight_mavlink_send_message(&msg);
			break;
		}
		case 140: {
			mavlink_msg_param_value_pack(1, 200, &msg, "LED_TYPE", (float)cfg.LED_Type, 140, 186, num - 1);
			baseflight_mavlink_send_message(&msg);
			break;
		}
		case 141: {
			mavlink_msg_param_value_pack(1, 200, &msg, "LED_PINOUT", (float)cfg.LED_Pinout, 141, 186, num - 1);
			baseflight_mavlink_send_message(&msg);
			break;
		}
		case 142: {
			mavlink_msg_param_value_pack(1, 200, &msg, "LED_CTRLCH", (float)cfg.LED_ControlChannel, 142, 186, num - 1);
			baseflight_mavlink_send_message(&msg);
			break;
		}
		case 143: {
			mavlink_msg_param_value_pack(1, 200, &msg, "LED_ARMED", (float)cfg.LED_Armed, 143, 186, num - 1);
			baseflight_mavlink_send_message(&msg);
			break;
		}
		case 144: {
			mavlink_msg_param_value_pack(1, 200, &msg, "LED_TGL_DELAY", (float)cfg.LED_Toggle_Delay, 144, 186, num - 1);
			baseflight_mavlink_send_message(&msg);
			break;
		}
		case 145: {
			mavlink_msg_param_value_pack(1, 200, &msg, "LED_PTRN1", (float)cfg.LED_Pattern1, 145, 186, num - 1);
			baseflight_mavlink_send_message(&msg);
			break;
		}
		case 146: {
			mavlink_msg_param_value_pack(1, 200, &msg, "LED_PTRN2", (float)cfg.LED_Pattern2, 146, 186, num - 1);
			baseflight_mavlink_send_message(&msg);
			break;
		}
		case 147: {
			mavlink_msg_param_value_pack(1, 200, &msg, "LED_PTRN3", (float)cfg.LED_Pattern3, 147, 186, num - 1);
			baseflight_mavlink_send_message(&msg);
			break;
		}
		case 148: {
			float val = 0.0;
			if (cfg.enabledFeatures & (1<<0)) {
				val = 1.0;
			}
			mavlink_msg_param_value_pack(1, 200, &msg, "FEA_PPM", val, 148, 186, num - 1);
			baseflight_mavlink_send_message(&msg);
			break;
		}
		case 149: {
			float val = 0.0;
			if (cfg.enabledFeatures & (1<<1)) {
				val = 1.0;
			}
			mavlink_msg_param_value_pack(1, 200, &msg, "FEA_VBAT", val, 149, 186, num - 1);
			baseflight_mavlink_send_message(&msg);
			break;
		}
		case 150: {
			float val = 0.0;
			if (cfg.enabledFeatures & (1<<2)) {
				val = 1.0;
			}
			mavlink_msg_param_value_pack(1, 200, &msg, "FEA_FLT_ACCCAL", val, 150, 186, num - 1);
			baseflight_mavlink_send_message(&msg);
			break;
		}
		case 151: {
			float val = 0.0;
			if (cfg.enabledFeatures & (1<<3)) {
				val = 1.0;
			}
			mavlink_msg_param_value_pack(1, 200, &msg, "FEA_SPEKTRUM", val, 151, 186, num - 1);
			baseflight_mavlink_send_message(&msg);
			break;
		}
		case 152: {
			float val = 0.0;
			if (cfg.enabledFeatures & (1<<4)) {
				val = 1.0;
			}
			mavlink_msg_param_value_pack(1, 200, &msg, "FEA_MOTOR_STOP", val, 152, 186, num - 1);
			baseflight_mavlink_send_message(&msg);
			break;
		}
		case 153: {
			float val = 0.0;
			if (cfg.enabledFeatures & (1<<5)) {
				val = 1.0;
			}
			mavlink_msg_param_value_pack(1, 200, &msg, "FEA_SERVO_TILT", val, 153, 186, num - 1);
			baseflight_mavlink_send_message(&msg);
			break;
		}
		case 154: {
			float val = 0.0;
			if (cfg.enabledFeatures & (1<<6)) {
				val = 1.0;
			}
			mavlink_msg_param_value_pack(1, 200, &msg, "FEA_GYRO_SMOOT", val, 154, 186, num - 1);
			baseflight_mavlink_send_message(&msg);
			break;
		}
		case 155: {
			float val = 0.0;
			if (cfg.enabledFeatures & (1<<7)) {
				val = 1.0;
			}
			mavlink_msg_param_value_pack(1, 200, &msg, "FEA_LED", val, 155, 186, num - 1);
			baseflight_mavlink_send_message(&msg);
			break;
		}
		case 156: {
			float val = 0.0;
			if (cfg.enabledFeatures & (1<<8)) {
				val = 1.0;
			}
			mavlink_msg_param_value_pack(1, 200, &msg, "FEA_GPS", val, 156, 186, num - 1);
			baseflight_mavlink_send_message(&msg);
			break;
		}
		case 157: {
			float val = 0.0;
			if (cfg.enabledFeatures & (1<<9)) {
				val = 1.0;
			}
			mavlink_msg_param_value_pack(1, 200, &msg, "FEA_FAILSAFE", val, 157, 186, num - 1);
			baseflight_mavlink_send_message(&msg);
			break;
		}
		case 158: {
			float val = 0.0;
			if (cfg.enabledFeatures & (1<<10)) {
				val = 1.0;
			}
			mavlink_msg_param_value_pack(1, 200, &msg, "FEA_SONAR", val, 158, 186, num - 1);
			baseflight_mavlink_send_message(&msg);
			break;
		}
		case 159: {
			float val = 0.0;
			if (cfg.enabledFeatures & (1<<11)) {
				val = 1.0;
			}
			mavlink_msg_param_value_pack(1, 200, &msg, "FEA_TELEMETRY", val, 159, 186, num - 1);
			baseflight_mavlink_send_message(&msg);
			break;
		}
		case 160: {
			float val = 0.0;
			if (cfg.enabledFeatures & (1<<12)) {
				val = 1.0;
			}
			mavlink_msg_param_value_pack(1, 200, &msg, "FEA_PASS", val, 160, 186, num - 1);
			baseflight_mavlink_send_message(&msg);
			break;
		}
		case 161: {
			float val = 0.0;
			if (cfg.enabledFeatures & (1<<13)) {
				val = 1.0;
			}
			mavlink_msg_param_value_pack(1, 200, &msg, "FEA_POWERMETER", val, 161, 186, num - 1);
			baseflight_mavlink_send_message(&msg);
			break;
		}
		case 162: {
			float val = 0.0;
			if (cfg.enabledFeatures & (1<<14)) {
				val = 1.0;
			}
			mavlink_msg_param_value_pack(1, 200, &msg, "FEA_LCD", val, 162, 186, num - 1);
			baseflight_mavlink_send_message(&msg);
			break;
		}
		case 163: {
			mavlink_msg_param_value_pack(1, 200, &msg, "MIXER", (float)cfg.mixerConfiguration, 163, 186, num - 1);
			baseflight_mavlink_send_message(&msg);
			break;
		}
		case 164: {
			mavlink_msg_param_value_pack(1, 200, &msg, "MAP_ROLL", (float)cfg.rcmap[0], 164, 186, num - 1);
			baseflight_mavlink_send_message(&msg);
			break;
		}
		case 165: {
			mavlink_msg_param_value_pack(1, 200, &msg, "MAP_PITCH", (float)cfg.rcmap[1], 165, 186, num - 1);
			baseflight_mavlink_send_message(&msg);
			break;
		}
		case 166: {
			mavlink_msg_param_value_pack(1, 200, &msg, "MAP_YAW", (float)cfg.rcmap[2], 166, 186, num - 1);
			baseflight_mavlink_send_message(&msg);
			break;
		}
		case 167: {
			mavlink_msg_param_value_pack(1, 200, &msg, "MAP_THROTTLE", (float)cfg.rcmap[3], 167, 186, num - 1);
			baseflight_mavlink_send_message(&msg);
			break;
		}
		case 168: {
			mavlink_msg_param_value_pack(1, 200, &msg, "MAP_AUX1", (float)cfg.rcmap[4], 168, 186, num - 1);
			baseflight_mavlink_send_message(&msg);
			break;
		}
		case 169: {
			mavlink_msg_param_value_pack(1, 200, &msg, "MAP_AUX2", (float)cfg.rcmap[5], 169, 186, num - 1);
			baseflight_mavlink_send_message(&msg);
			break;
		}
		case 170: {
			mavlink_msg_param_value_pack(1, 200, &msg, "MAP_AUX3", (float)cfg.rcmap[6], 170, 186, num - 1);
			baseflight_mavlink_send_message(&msg);
			break;
		}
		case 171: {
			mavlink_msg_param_value_pack(1, 200, &msg, "MAP_AUX4", (float)cfg.rcmap[7], 171, 186, num - 1);
			baseflight_mavlink_send_message(&msg);
			break;
		}
		case 172: {
			mavlink_msg_param_value_pack(1, 200, &msg, "BOX_ANGLE", (float)cfg.activate[0], 172, 186, num - 1);
			baseflight_mavlink_send_message(&msg);
			break;
		}
		case 173: {
			mavlink_msg_param_value_pack(1, 200, &msg, "BOX_HORIZON", (float)cfg.activate[1], 173, 186, num - 1);
			baseflight_mavlink_send_message(&msg);
			break;
		}
		case 174: {
			mavlink_msg_param_value_pack(1, 200, &msg, "BOX_BARO", (float)cfg.activate[2], 174, 186, num - 1);
			baseflight_mavlink_send_message(&msg);
			break;
		}
		case 175: {
			mavlink_msg_param_value_pack(1, 200, &msg, "BOX_MAG", (float)cfg.activate[3], 175, 186, num - 1);
			baseflight_mavlink_send_message(&msg);
			break;
		}
		case 176: {
			mavlink_msg_param_value_pack(1, 200, &msg, "BOX_CAMSTAB", (float)cfg.activate[4], 176, 186, num - 1);
			baseflight_mavlink_send_message(&msg);
			break;
		}
		case 177: {
			mavlink_msg_param_value_pack(1, 200, &msg, "BOX_CAMTRIG", (float)cfg.activate[5], 177, 186, num - 1);
			baseflight_mavlink_send_message(&msg);
			break;
		}
		case 178: {
			mavlink_msg_param_value_pack(1, 200, &msg, "BOX_ARM", (float)cfg.activate[6], 178, 186, num - 1);
			baseflight_mavlink_send_message(&msg);
			break;
		}
		case 179: {
			mavlink_msg_param_value_pack(1, 200, &msg, "BOX_GPSHOME", (float)cfg.activate[7], 179, 186, num - 1);
			baseflight_mavlink_send_message(&msg);
			break;
		}
		case 180: {
			mavlink_msg_param_value_pack(1, 200, &msg, "BOX_GPSHOLD", (float)cfg.activate[8], 180, 186, num - 1);
			baseflight_mavlink_send_message(&msg);
			break;
		}
		case 181: {
			mavlink_msg_param_value_pack(1, 200, &msg, "BOX_PASSTHRU", (float)cfg.activate[9], 181, 186, num - 1);
			baseflight_mavlink_send_message(&msg);
			break;
		}
		case 182: {
			mavlink_msg_param_value_pack(1, 200, &msg, "BOX_HEADFREE", (float)cfg.activate[10], 182, 186, num - 1);
			baseflight_mavlink_send_message(&msg);
			break;
		}
		case 183: {
			mavlink_msg_param_value_pack(1, 200, &msg, "BOX_BEEPERON", (float)cfg.activate[11], 183, 186, num - 1);
			baseflight_mavlink_send_message(&msg);
			break;
		}
		case 184: {
			mavlink_msg_param_value_pack(1, 200, &msg, "BOX_LEDMAX", (float)cfg.activate[12], 184, 186, num - 1);
			baseflight_mavlink_send_message(&msg);
			break;
		}
		case 185: {
			mavlink_msg_param_value_pack(1, 200, &msg, "BOX_LLIGHTS", (float)cfg.activate[13], 185, 186, num - 1);
			baseflight_mavlink_send_message(&msg);
			break;
		}
		case 186: {
			mavlink_msg_param_value_pack(1, 200, &msg, "BOX_HEADADJ", (float)cfg.activate[14], 186, 186, num - 1);
			baseflight_mavlink_send_message(&msg);
			break;
		}
	}
	return 188;
}

