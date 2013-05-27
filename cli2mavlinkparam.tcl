#!/usr/bin/tclsh
#
#


set clifile [open "src/cli.c" r]
set file_data [read $clifile]
close $clifile

set mwhfile [open "src/mw.h" r]
set mwhfile_data [read $mwhfile]
close $mwhfile

set mapfile [open "mavlink.map" r]
set mapfile_data [read $mapfile]
close $mapfile


set REPLACE_MAP "CONTROLLER CTRLR"
append REPLACE_MAP " CONTROL CTRL"
append REPLACE_MAP " CHANNEL CH"
append REPLACE_MAP " TOGGLE TGL"
append REPLACE_MAP " PATTERN PTRN"
append REPLACE_MAP " POWER PWR"
append REPLACE_MAP " THROTTLE THR"
append REPLACE_MAP " ANGLE ANG"
append REPLACE_MAP " PERCENT PRC"
append REPLACE_MAP " BAUDRATE BRATE"
append REPLACE_MAP " DECLINATION DECL"
append REPLACE_MAP " DEADBAND DBAND"
append REPLACE_MAP " COMMAND CMD"
append REPLACE_MAP " FAILSAFE RC_FS"
append REPLACE_MAP " SWITCHT SW"
append REPLACE_MAP " VOLTAGE VOLT"
append REPLACE_MAP " CHECK CHK"
append REPLACE_MAP " GIMBAL GMBL"
append REPLACE_MAP " DIRECTION DIR"
append REPLACE_MAP " VBAT VBAT_"


proc param_autorename {PARAM} {
	set NEW_PARAM $PARAM
	if {[string match "YAW_*" $PARAM]} {
	} elseif {[string match "YAW*" $PARAM]} {
		set NEW_PARAM [string map {YAW YAW_} $PARAM]
	} elseif {[string match "GPS_*" $PARAM]} {
	} elseif {[string match "ACC_*" $PARAM]} {
	} elseif {[string match "GYRO_*" $PARAM]} {
	} elseif {[string match "MAG_*" $PARAM]} {
	} elseif {[string match "*_YAW" $PARAM]} {
		set NEW_PARAM "[join [lrange [split $PARAM "_"] 1 end] "_"]_[lindex [split $PARAM "_"] 0]"
	} elseif {[string match "*_ROLL" $PARAM]} {
		set NEW_PARAM "[join [lrange [split $PARAM "_"] 1 end] "_"]_[lindex [split $PARAM "_"] 0]"
	} elseif {[string match "*_PITCH" $PARAM]} {
		set NEW_PARAM "[join [lrange [split $PARAM "_"] 1 end] "_"]_[lindex [split $PARAM "_"] 0]"
	} elseif {[string match "*_ALT" $PARAM]} {
		set NEW_PARAM "[join [lrange [split $PARAM "_"] 1 end] "_"]_[lindex [split $PARAM "_"] 0]"
	} elseif {[string match "*_LEVEL" $PARAM]} {
		set NEW_PARAM "[join [lrange [split $PARAM "_"] 1 end] "_"]_[lindex [split $PARAM "_"] 0]"
	} elseif {[string match "ALIGN_*" $PARAM]} {
		set NEW_PARAM "[join [lrange [split $PARAM "_"] 1 end] "_"]_[lindex [split $PARAM "_"] 0]"
	}
	return $NEW_PARAM
} 


proc string_pad {orig {num 1} {char " "} {side front} } {
	set pad [string repeat $char $num]
	switch $side {
		beginning - front {return $pad$orig}
		back - end {return $orig$pad}
		default {return $orig}
	}
}


if {$argv == "-d"} {
	set mapfile [open "mavlink.map" a]
	foreach line [split $file_data "\n"] {
		if {[string match "*\{*,*VAR_*,*\}*" $line]} {
			set name "[string trim [lindex [split "$line" "\""] 1]]"
			set mavname [string toupper $name]
			set flag 0
			foreach line2 [split $mapfile_data "\n"] {
				set map_name [lindex $line2 1]
				if {$mavname == $map_name} {
					set flag 1
				}
			}
			if {$flag == 0} {
				puts "adding: $mavname"

				set new_mavname [string map $REPLACE_MAP [string toupper $name]]
				set new_mavname [param_autorename $new_mavname]

				if {[string length $new_mavname] > 16} {
					puts $mapfile "[string_pad $new_mavname [expr 40 - [string length $new_mavname]] { } back] $mavname	LONG"
				} else {
					puts $mapfile "[string_pad $new_mavname [expr 40 - [string length $new_mavname]] { } back] $mavname"
				}
			} else {
				puts "exist: $mavname"
			}
		}
	}
	close $mapfile
	exit 0
}



set features {"PPM" "VBAT" "INFLIGHT_ACC_CAL" "SPEKTRUM" "MOTOR_STOP" "SERVO_TILT" "GYRO_SMOOTHING" "LED" "GPS" "FAILSAFE" "SONAR" "TELEMETRY" "PASS" "POWERMETER" "LCD"}
set mixers {"TRI" "QUADP" "QUADX" "BI" "GIMBAL" "Y6" "HEX6" "FLYING_WING" "Y4" "HEX6X" "OCTOX8" "OCTOFLATP" "OCTOFLATX" "AIRPLANE" "HELI_120_CCPM" "HELI_90_DEG" "VTAIL4" "CUSTOM"}
set maps {"ROLL" "PITCH" "YAW" "THROTTLE" "AUX1" "AUX2" "AUX3" "AUX4"}
set auxnames {ANGLE HORIZON BARO MAG CAMSTAB CAMTRIG ARM GPSHOME GPSHOLD PASSTHRU HEADFREE BEEPERON LEDMAX LLIGHTS HEADADJ}


proc param_map {PARAM} {
	global mapfile_data
	set flag 0
	foreach line2 [split $mapfile_data "\n"] {
		set map_name [lindex $line2 1]
		if {$PARAM == $map_name} {
			set flag 1
			set NEW_PARAM [lindex $line2 0]
		}
	}
	if {$flag == 0} {
		set NEW_PARAM $PARAM
	}
	return $NEW_PARAM
} 

set metaxml [open "ParameterMetaData.xml" w]
puts $metaxml "<?xml version=\"1.0\"?>"
puts $metaxml "<Params>"
puts $metaxml "  <ArduCopter2>"
foreach line [split $file_data "\n"] {
	if {[string match "*\{*,*VAR_*,*\}*" $line]} {
		set name "[string trim [lindex [split "$line" "\""] 1]]"
		set type "[string trim [lindex [split "$line" ","] 1]]"
		set var "[string trim [string trim [lindex [split "$line" ","] 2]] "&"]"
		set min "[string trim [lindex [split "$line" ","] 3]]"
		set max "[string trim [lindex [split "$line" ","] 4]]"
		set mavname [param_map [string toupper $name]]
		set varname [string map {cfg. ""} [lindex [split $var "\["] 0]]
		set desc "$var"
		foreach line2 [split $mwhfile_data "\n"] {
			if {[string match "* $varname;*//*" $line2]} {
				set desc "[string trim [join [lrange [split $line2 "/"] 2 end] "/"]]"
			}
		}
		puts $metaxml "    <$mavname>"
		puts $metaxml "      <DisplayName>$name</DisplayName>"
		puts $metaxml "      <Description>[string map {& &amp;} $desc]</Description>"
		puts $metaxml "      <Units></Units>"
		puts $metaxml "      <Range>$min $max</Range>"
		if {[string match "*.*" $min] || [string match "*.*" $max]} {
			puts $metaxml "      <Increment>0.01</Increment>"
		} else {
			puts $metaxml "      <Increment>1</Increment>"
		}
		puts $metaxml "      <User>Advanced</User>"
		puts $metaxml "    </$mavname>"
	}
}
puts $metaxml "  </ArduCopter2>"
puts $metaxml "</Params>"
close $metaxml



puts ""
puts "#include \"board.h\""
puts "#include \"mw.h\""
puts "#include \"baseflight_mavlink.h\""
puts ""
puts "void baseflight_mavlink_set_param (mavlink_param_set_t *packet) {"
puts "	if (strcmp(packet->param_id, \"\") == 0) {"
set NUM 1
foreach line [split $file_data "\n"] {
	if {[string match "*\{*,*VAR_*,*\}*" $line]} {
#		puts "#$line#"
		set name "[string trim [lindex [split "$line" "\""] 1]]"
		set type "[string trim [lindex [split "$line" ","] 1]]"
		set var "[string trim [string trim [lindex [split "$line" ","] 2]] "&"]"
		set min "[string trim [lindex [split "$line" ","] 3]]"
		set max "[string trim [lindex [split "$line" ","] 4]]"

		set mavname [param_map [string toupper $name]]

#		puts "## $name - $type - $var - $min - $max ##"
		if {$type == "VAR_UINT8"} {
			set ctype "uint8_t"
		} elseif {$type == "VAR_UINT16"} {
			set ctype "uint16_t"
		} elseif {$type == "VAR_UINT32"} {
			set ctype "uint32_t"
		} elseif {$type == "VAR_INT8"} {
			set ctype "int8_t"
		} elseif {$type == "VAR_INT16"} {
			set ctype "int16_t"
		} elseif {$type == "VAR_INT32"} {
			set ctype "int32_t"
		} elseif {$type == "VAR_FLOAT"} {
			set ctype "float"
		} else {
			puts "// Unknown $type"
			exit 0
		}
		puts "	\} else if (strcmp(packet->param_id, \"$mavname\") == 0) \{"
		puts "		$ctype val = ($ctype)packet->param_value;"
		puts "		if (val >= $min && val <= $max) \{"
		puts "			$var = val;"
		puts "		\}"
		incr NUM
	}
}

set feature_num 0
foreach feature $features {
	set feature [string map {INFLIGHT_ACC_CAL FLT_ACCCAL} $feature]
	set feature [string map {GYRO_SMOOTHING GYRO_SMOOT} $feature]
	puts "	\} else if (strcmp(packet->param_id, \"FEA_[string toupper $feature]\") == 0) \{"
	puts "		if (packet->param_value == 1.0) {"
	puts "			cfg.enabledFeatures |= (1<<$feature_num);"
	puts "		} else {"
	puts "			cfg.enabledFeatures &= ~(1<<$feature_num);"
	puts "		}"
	incr feature_num
	incr NUM
}

puts "	\} else if (strcmp(packet->param_id, \"MIXER\") == 0) \{"
puts "		cfg.mixerConfiguration = (uint8_t)packet->param_value;"
incr feature_num
incr NUM

set map_num 0
foreach map $maps {
	puts "	\} else if (strcmp(packet->param_id, \"MAP_[string toupper $map]\") == 0) \{"
	puts "		cfg.rcmap\[$map_num\] = (uint8_t)packet->param_value;"
	incr map_num
	incr NUM
}

set aux_num 0
foreach aux $auxnames {
	puts "	\} else if (strcmp(packet->param_id, \"BOX_[string toupper $aux]\") == 0) \{"
	puts "		cfg.activate\[$aux_num\] = (uint32_t)packet->param_value;"
	incr aux_num
	incr NUM
}

puts "	}"
puts "}"
puts ""
set MAXPARAM [expr $NUM - 1]

puts "uint8_t baseflight_mavlink_send_param (uint8_t num) {"
puts "	mavlink_message_t msg;"
puts "	switch (num) {"
set NUM 1
foreach line [split $file_data "\n"] {
	if {[string match "*\{*,*VAR_*,*\}*" $line]} {
#		puts "#$line#"
		set name "[string trim [lindex [split "$line" "\""] 1]]"
		set type "[string trim [lindex [split "$line" ","] 1]]"
		set var "[string trim [string trim [lindex [split "$line" ","] 2]] "&"]"
		set min "[string trim [lindex [split "$line" ","] 3]]"
		set max "[string trim [lindex [split "$line" ","] 4]]"

		set mavname [param_map [string toupper $name]]


#		puts "## $name - $type - $var - $min - $max ##"
		if {$type == "VAR_UINT8"} {
			set ctype "uint8_t"
		} elseif {$type == "VAR_UINT16"} {
			set ctype "uint16_t"
		} elseif {$type == "VAR_UINT32"} {
			set ctype "uint32_t"
		} elseif {$type == "VAR_INT8"} {
			set ctype "int8_t"
		} elseif {$type == "VAR_INT16"} {
			set ctype "int16_t"
		} elseif {$type == "VAR_INT32"} {
			set ctype "int32_t"
		} elseif {$type == "VAR_FLOAT"} {
			set ctype "float"
		} else {
			puts "// Unknown $type"
			exit 0
		}
		puts "		case $NUM: {"
		puts "			mavlink_msg_param_value_pack(1, 200, &msg, \"$mavname\", (float)$var, $NUM, $MAXPARAM, num - 1);"
		puts "			baseflight_mavlink_send_message(&msg);"
		puts "			break;"
		puts "		}"
		incr NUM
	}
}

set feature_num 0
foreach feature $features {
	set feature [string map {INFLIGHT_ACC_CAL FLT_ACCCAL} $feature]
	set feature [string map {GYRO_SMOOTHING GYRO_SMOOT} $feature]
	puts "		case $NUM: {"
	puts "			float val = 0.0;"
	puts "			if (cfg.enabledFeatures & (1<<$feature_num)) {"
	puts "				val = 1.0;"
	puts "			}"
	puts "			mavlink_msg_param_value_pack(1, 200, &msg, \"FEA_[string toupper $feature]\", val, $NUM, $MAXPARAM, num - 1);"
	puts "			baseflight_mavlink_send_message(&msg);"
	puts "			break;"
	puts "		}"
	incr feature_num
	incr NUM
}

puts "		case $NUM: {"
puts "			mavlink_msg_param_value_pack(1, 200, &msg, \"MIXER\", (float)cfg.mixerConfiguration, $NUM, $MAXPARAM, num - 1);"
puts "			baseflight_mavlink_send_message(&msg);"
puts "			break;"
puts "		}"
incr NUM


set map_num 0
foreach map $maps {
	puts "		case $NUM: {"
	puts "			mavlink_msg_param_value_pack(1, 200, &msg, \"MAP_[string toupper $map]\", (float)cfg.rcmap\[$map_num\], $NUM, $MAXPARAM, num - 1);"
	puts "			baseflight_mavlink_send_message(&msg);"
	puts "			break;"
	puts "		}"
	incr map_num
	incr NUM
}

set aux_num 0
foreach aux $auxnames {
	puts "		case $NUM: {"
	puts "			mavlink_msg_param_value_pack(1, 200, &msg, \"BOX_[string toupper $aux]\", (float)cfg.activate\[$aux_num\], $NUM, $MAXPARAM, num - 1);"
	puts "			baseflight_mavlink_send_message(&msg);"
	puts "			break;"
	puts "		}"
	incr aux_num
	incr NUM
}



puts "	}"
puts "	return [expr $NUM + 1];"
puts "}"
puts ""



