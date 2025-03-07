#pragma once

/* © Copyright CERN 2017. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Zsolt Pasztori CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

// CAN ID codes of devices
const unsigned int ERROR_FROM_DEVICE = 0x300;
const unsigned int MESSAGE_TO_DEVICE = 0x500;
const unsigned int MESSAGE_FROM_DEVICE = 0x700;
// The motor ID starts from 3, this is the first motor
const unsigned int MESSAGE_FROM_MOTOR3 = 0x703;

// Motor Codes
const int GET_DEVICE_CODE = 0x00ff;
const int JOINT1 = 0x3;
const int JOINT2 = 0x4;
const int JOINT3 = 0x5;
const int JOINT4 = 0x6;
const int JOINT5 = 0x7;
const int JOINT6 = 0x8;
const int GRIPPER = 0xC;

// Message codes
const int GET_CONFIG = 0x80;
const int SET_CONFIG = 0x81;
const int GET_CONFIG_EXT = 0x82;
const int SET_CONFIG_EXT = 0x83;
const int FRAG_START = 0x84;
const int FRAG_MIDDLE = 0x85;
const int FRAG_END = 0x86;
const int FRAG_ACK = 0x87;
const int CMD_ERROR = 0x88;
const int CMD_WARNING = 0x89;
const int CMD_INFO = 0x8A;
const int CMD_ACK = 0x8B;
const int CMD_FAST_STOP = 0x90;
const int CMD_STOP = 0x91;
const int CMD_REFERENCE = 0x92;
const int CMD_MOVE_BLOCKED = 0x93;
const int CMD_POS_REACHED = 0x94;
const int GET_STATE = 0x95;
const int GET_DETAILED_ERROR_INFO = 0x96;
const int CMD_REFERENCE_HAND = 0x97;
const int GET_STATE_AXIS = 0x98;
const int SET_TARGET_VEL = 0xA0;
const int SET_TARGET_ACC = 0xA1;
const int SET_TARGET_JERK = 0xA2;
const int SET_TARGET_CUR = 0xA3;
const int SET_TARGET_TIME = 0xA4;
const int SET_TARGET_POS = 0xA6;
const int SET_TARGET_POS_REL = 0xA7;
const int MOVE_POS = 0xB0;
const int MOVE_POS_TIME = 0xB1;
const int MOVE_CURR = 0xB3;
const int MOVE_VEL = 0xB5;
const int MOVE_GRIP = 0xB7;
const int MOVE_POS_REL = 0xB8;
const int MOVE_POS_TIME_REL = 0xB9;
const int CMD_REBOOT = 0xE0;

// Info codes
const int INFO_BOOT = 0x01;  // the device was rebooted just now
const int INFO_UNKNOWN_COMMAND = 0x04;
const int INFO_FAILED = 0x05;  // the command failed
const int NOT_REFERENCED = 0x06;
const int INFO_SEARCH_SINE_VECTOR = 0x07;
const int INFO_NO_ERROR = 0x08;
const int INFO_COMMUNICATION_ERROR = 0x09;
const int INFO_TIMOUT = 0x10;
const int INFO_CHECKSUM = 0x19;
const int INFO_MESSAGE_LENGTH = 0x1D;
const int INFO_WRONG_PARAMETER = 0x1E;

// Error codes
const int ERROR_TEMP_LOW = 0x70;
const int ERROR_TEMP_HIGH = 0x71;
const int ERROR_LOGIC_LOW = 0x72;
const int ERROR_LOGIC_HIGH = 0x73;
const int ERROR_MOTOR_VOLTAGE_LOW = 0x74;
const int ERROR_MOTOR_VOLTAGE_HIGH = 0x75;
const int ERROR_CABLE_BREAK = 0x76;
const int ERROR_OVERSHOOT = 0x82;
const int ERROR_WRONG_RAMP_TYPE = 0xC8;
const int ERROR_CONFIG_MEMORY = 0xD2;
const int ERROR_PROGRAM_MEMORY = 0xD3;
const int ERROR_INVALID_PHRASE = 0xD4;
const int ERROR_SOFT_LOW = 0xD5;
const int ERROR_SOFT_HIGH = 0xD6;
const int ERROR_SERVICE = 0xD8;
const int ERROR_FAST_STOP = 0xD9;
const int ERROR_TOW = 0xDA;
const int ERROR_VPC3 = 0xDB;
const int ERROR_FRAGMENTATION = 0xDC;
const int ERROR_COMMUTATION = 0xDD;
const int ERROR_CURRENT = 0xDE;
const int ERROR_I2T = 0xDF;
const int ERROR_INITIALIZE = 0xE0;
const int ERROR_INTERNAL = 0xE1;
const int ERROR_TOO_FAST = 0xE4;
const int ERROR_RESOLVER_CHECK_FAILED = 0xEB;
const int ERROR_MATH = 0xEC;

// Codes of ascii characters
const int ASCII_O = 0x4f;
const int ASCII_K = 0x4b;

// To use with GET_STATE command
const int GET_ONLY_STATE = 0x00;
const int GET_POS = 0x01;
const int GET_POS_VEL = 0x03;
const int GET_POS_VEL_CURR = 0x07;

// Answers from GET_STATE command
const int REFERENCED_BRAKEOFF = 0x01;
const int REFERENCED_BREAKON = 0x21;
const int STATE_NOT_REFERENCED = 0x00;
const int REFERENCED_ERROR = 0x11;
const int MOTION_BLOCKED = 0X40;

// To use with GET_CONFIG command
const int MAX_POS = 0x07;
const int MIN_POS = 0x08;
const int MAX_SPEED = 0x09;
const int MAX_ACCELERATION = 0x0A;
const int MAX_CURRENT = 0x0B;
