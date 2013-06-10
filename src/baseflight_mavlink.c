
#include "board.h"
#include "mw.h"
#include "baseflight_mavlink.h"


static uint8_t mavlink_send_parameter = 0;
static uint8_t mavlink_send_mission = 0;
static uint8_t mavlink_get_mission = 0;
static uint16_t mission_max = 0;

void cliSave(char *cmdline);

#define MAX_WAYPOINTS 15
typedef struct {
	float p_lat;
	float p_long;
	float p_alt;
	float yaw;
	float radius;
	float wait;
	float orbit;
	char name[128];
	char command[128];
	uint8_t type;
} WayPoint;

volatile WayPoint WayPoints[MAX_WAYPOINTS];



void baseflight_mavlink_send_message (mavlink_message_t* msg) {
	uint8_t buf[MAVLINK_MAX_PACKET_LEN];
	uint16_t len = mavlink_msg_to_send_buffer(buf, msg);
	uint16_t n = 0;
	for (n = 0; n < len; n++) {
		uartWrite(buf[n]);
	}
}

void baseflight_mavlink_receive (char new) {
	static mavlink_message_t msg;
	static mavlink_status_t status;
	if (mavlink_parse_char(0, new, &msg, &status)) {
		baseflight_mavlink_handleMessage(&msg);
	}
}

void baseflight_mavlink_handleMessage (mavlink_message_t *msg) {
	mavlink_message_t msg2;
	switch (msg->msgid) {
		case MAVLINK_MSG_ID_PARAM_REQUEST_LIST: {
			//printf("## MSG_ID: PARAM_REQUEST ##\r\n");
			if (mavlink_send_parameter == 0) {
				mavlink_send_parameter = 1;
			}
			break;
		}

		case MAVLINK_MSG_ID_MISSION_REQUEST_LIST: {
			//printf("## MSG_ID: MISSION_REQUEST_LIST ##\r\n");
			if (mavlink_send_mission == 0) {
				mavlink_send_mission = 1;
			}
			break;
		}

		case MAVLINK_MSG_ID_COMMAND_LONG: {
			mavlink_command_long_t packet;
			mavlink_msg_command_long_decode(msg, &packet);
			if (packet.command == MAV_CMD_PREFLIGHT_STORAGE && packet.param1 == 1.0) {
				cliSave("");
			}
			break;
		}

		case MAVLINK_MSG_ID_MISSION_COUNT: {
			//printf("## MSG_ID: MISSION_COUNT ##\r\n");
			if (mavlink_get_mission == 0) {
				mavlink_mission_count_t packet;
				mavlink_msg_mission_count_decode(msg, &packet);

				mission_max = packet.count;

				mavlink_msg_mission_request_pack(1, 200, &msg2, 1, 200, 0);
				baseflight_mavlink_send_message(&msg2);
			}
			break;
		}

		case MAVLINK_MSG_ID_MISSION_ITEM: {
			mavlink_mission_item_t packet;
			mavlink_msg_mission_item_decode(msg, &packet);

			//printf("RECEIVED MISSION_ITEM: %i/%i: %f, %f, %f (%i)\n", packet.seq, mission_max, packet.x, packet.y, packet.z, packet.frame);

			if (packet.seq < mission_max - 1) {
				mavlink_msg_mission_request_pack(127, 0, &msg2, 1, 200, packet.seq + 1);
				baseflight_mavlink_send_message(&msg2);
			} else {
				mavlink_msg_mission_ack_pack(127, 0, &msg2, 1, 200, 15);
				baseflight_mavlink_send_message(&msg2);
			}

			if (packet.seq > 0) {
				packet.seq = packet.seq - 1;
			}

			//printf("getting WP(%i): %f, %f\n", packet.seq, packet.x, packet.y);

			switch (packet.command) {
				case MAV_CMD_NAV_WAYPOINT: {
					strcpy((char *)WayPoints[1 + packet.seq].command, "WAYPOINT");
					break;
				}
				case MAV_CMD_NAV_LOITER_UNLIM: {
					strcpy((char *)WayPoints[1 + packet.seq].command, "LOITER_UNLIM");
					break;
				}
				case MAV_CMD_NAV_LOITER_TURNS: {
					strcpy((char *)WayPoints[1 + packet.seq].command, "LOITER_TURNS");
					break;
				}
				case MAV_CMD_NAV_LOITER_TIME: {
					strcpy((char *)WayPoints[1 + packet.seq].command, "LOITER_TIME");
					break;
				}
				case MAV_CMD_NAV_RETURN_TO_LAUNCH: {
					strcpy((char *)WayPoints[1 + packet.seq].command, "RTL");
					break;
				}
				case MAV_CMD_NAV_LAND: {
					strcpy((char *)WayPoints[1 + packet.seq].command, "LAND");
					break;
				}
				case MAV_CMD_NAV_TAKEOFF: {
					strcpy((char *)WayPoints[1 + packet.seq].command, "TAKEOFF");
					break;
				}
				default: {
					sprintf((char *)WayPoints[1 + packet.seq].command, "CMD:%i", packet.command);
					break;
				}
			}

			if (packet.x == 0.0) {
				packet.x = 0.00001;
			}
			if (packet.y == 0.0) {
				packet.y = 0.00001;
			}
			if (packet.z == 0.0) {
				packet.z = 0.00001;
			}
			WayPoints[1 + packet.seq].p_lat = packet.x;
			WayPoints[1 + packet.seq].p_long = packet.y;
			WayPoints[1 + packet.seq].p_alt = packet.z;
			WayPoints[1 + packet.seq].yaw = packet.param4;
			sprintf((char *)WayPoints[1 + packet.seq].name, "WP%i", packet.seq + 1);

			WayPoints[1 + packet.seq + 1].p_lat = 0.0;
			WayPoints[1 + packet.seq + 1].p_long = 0.0;
			WayPoints[1 + packet.seq + 1].p_alt = 0.0;
			WayPoints[1 + packet.seq + 1].yaw = 0.0;
			strcpy((char *)WayPoints[1 + packet.seq + 1].name, "");
			strcpy((char *)WayPoints[1 + packet.seq + 1].command, "");
			break;
		}

		case MAVLINK_MSG_ID_PARAM_SET: {
			mavlink_param_set_t packet;
			mavlink_msg_param_set_decode(msg, &packet);

			//printf("## MSG_VALUE: %s %f ##\r\n", packet.param_id, packet.param_value);

			baseflight_mavlink_set_param(&packet);

			break;
		}
		default: {
			//printf("## MSG_ID: %i ##\r\n", msg->msgid);
			break;
		}
	}
}

void baseflight_mavlink_send_updates (void) {
	static uint16_t timer_n = 0;
	mavlink_message_t msg2;
	uint8_t system_type = MAV_TYPE_QUADROTOR;
	uint8_t autopilot_type = MAV_AUTOPILOT_ARDUPILOTMEGA;
	uint8_t system_mode = MAV_MODE_MANUAL_ARMED;
	uint32_t custom_mode = 0;
	uint8_t system_state = MAV_STATE_ACTIVE;



	if (timer_n++ > 20) {
		timer_n = 0;
		mavlink_msg_heartbeat_pack(1, 200, &msg2, system_type, autopilot_type, system_mode, custom_mode, system_state);
		baseflight_mavlink_send_message(&msg2);
	}

	if (mavlink_send_parameter > 0) {
		if (baseflight_mavlink_send_param(mavlink_send_parameter) == mavlink_send_parameter) {
			mavlink_send_parameter = 0;
		} else {
			mavlink_send_parameter++;
		}
	}


	mavlink_msg_attitude_pack(1, 200, &msg2, timer_n, toRad((float)angle[0] / 10.0), toRad((float)angle[1] / -10.0), toRad((float)heading / 10.0), 0.0, 0.0, 0.0);
	baseflight_mavlink_send_message(&msg2);

	mavlink_msg_rc_channels_scaled_pack(1, 200, &msg2, timer_n, 0, (float)((rcDataSAVE[0]) - 1500) * 20, (float)((rcDataSAVE[1]) - 1500) * 20, (float)((rcDataSAVE[3]) - 1500) * 20, (float)((rcDataSAVE[2]) - 1500) * 20, (float)((rcDataSAVE[4]) - 1500) * 20, (float)((rcDataSAVE[5]) - 1500) * 20, (float)((rcDataSAVE[6]) - 1500) * 20, (float)((rcDataSAVE[7]) - 1500) * 20, 100);
	baseflight_mavlink_send_message(&msg2);

	mavlink_msg_gps_raw_int_pack(1, 200, &msg2, timer_n, f.GPS_FIX, GPS_coord[LAT] * 10000000, GPS_coord[LON] * 10000000, GPS_altitude, 0, 0, GPS_speed, 0, GPS_numSat);
	baseflight_mavlink_send_message(&msg2);

//	mavlink_msg_vfr_hud_pack(1, 200, &msg2, GPS_speed, GPS_speed, toRad((float)heading / 10.0), 0, GPS_altitude, 0.0);
//	baseflight_mavlink_send_message(&msg2);


}


