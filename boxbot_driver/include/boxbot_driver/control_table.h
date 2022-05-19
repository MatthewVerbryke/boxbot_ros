// AX12 Servo Control Table Definitions
//
// Based on:
// https://github.com/Interbotix/arbotix_ros/blob/indigo-devel/arbotix_python/src/arbotix_python/ax12.py
//
// Copyright 2022 University of Cincinnati
// All rights reserved. See LICENSE file at:
// https://github.com/MatthewVerbryke/rse_dam
// Additional copyright may be held by others, as reflected in the commit history.

#ifndef AX12_CONTROL_TABLE
#define AX12_CONTROL_TABLE

// EEPROM Area Symbolic Constants
const int P_MODEL_NUMBER_L = 0;
const int P_MODEL_NUMBER_H = 1;
const int P_VERSION = 2;
const int P_ID = 3;
const int P_BAUD_RATE = 4;
const int P_RETURN_DELAY_TIME = 5;
const int P_CW_ANGLE_LIMIT_L = 6;
const int P_CW_ANGLE_LIMIT_H = 7;
const int P_CCW_ANGLE_LIMIT_L = 8;
const int P_CCW_ANGLE_LIMIT_H = 9;
const int P_SYSTEM_DATA2 = 10;
const int P_LIMIT_TEMPERATURE = 11;
const int P_DOWN_LIMIT_VOLTAGE = 12;
const int P_UP_LIMIT_VOLTAGE = 13;
const int P_MAX_TORQUE_L = 14;
const int P_MAX_TORQUE_H = 15;
const int P_RETURN_LEVEL = 16;
const int P_ALARM_LEVEL = 17;
const int P_ALARM_SHUTDOWN = 18;
const int P_OPERATING_MODE = 19;
const int P_DOWN_CALIBRATION_L = 20;
const int P_DOWN_CALIBRATION_H = 21;
const int P_UP_CALIBRATION_L = 22;
const int P_UP_CALIBRATION_H = 23;

// RAM Area Symbolic Contants
const int P_TORQUE_ENABLE = 24;
const int P_LED = 25;
const int P_CW_COMPLIANCE_MARGIN = 26;
const int P_CCW_COMPLIANCE_MARGIN = 27;
const int P_CW_COMPLIANCE_SLOPE = 28;
const int P_CCW_COMPLIANCE_SLOPE = 29;
const int P_GOAL_POSITION_L = 30;
const int P_GOAL_POSITION_H = 31;
const int P_GOAL_SPEED_L = 32;
const int P_GOAL_SPEED_H = 33;
const int P_TORQUE_LIMIT_L = 34;
const int P_TORQUE_LIMIT_H = 35;
const int P_PRESENT_POSITION_L = 36;
const int P_PRESENT_POSITION_H = 37;
const int P_PRESENT_SPEED_L = 38;
const int P_PRESENT_SPEED_H = 39;
const int P_PRESENT_LOAD_L = 40;
const int P_PRESENT_LOAD_H = 41;
const int P_PRESENT_VOLTAGE = 42;
const int P_PRESENT_TEMPERATURE = 43;
const int P_REGISTERED_INSTRUCTION = 44;
const int P_PAUSE_TIME = 45;
const int P_MOVING = 46;
const int P_LOCK = 47;
const int P_PUNCH_L = 48;
const int P_PUNCH_H = 49;

// Status Return Levels
const int AX_RETURN_NONE = 0;
const int AX_RETURN_READ = 1;
const int AX_RETURN_ALL = 2;

// Instruction Set
const int AX_PING = 1;
const int AX_READ_DATA = 2;
const int AX_WRITE_DATA = 3;
const int AX_REG_WRITE = 4;
const int AX_ACTION = 5;
const int AX_RESET = 6;
const int AX_SYNC_WRITE = 131;
const int AX_SYNC_READ = 132;

const int AX_CONTROL_SETUP = 26;
const int AX_CONTROL_WRITE = 27;
const int AX_CONTROL_STAT = 28;

#endif
