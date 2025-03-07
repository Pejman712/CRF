#pragma once

#include <unistd.h>
#include <string.h>
#include <stdio.h>
#include <fcntl.h>
#include <math.h>
#include <modbus/modbus.h>
#include <thread>

#include <Gripper/Gripper.hpp>

class RobotiqGripper140 : public RobotArmGripper::Gripper {	
	char* device_name;
	modbus_t *ctx;
	
	bool is_connected();
	
	
	bool is_active;
	bool is_going_to_position;
	int gripper_status;
	int object_detection;
	
	unsigned char actual_position_char;
	unsigned char actual_finger_current_char;
  
	unsigned char target_position_char;
	unsigned char target_speed_char;
	unsigned char target_force_char;
  
	std::thread gripper_thread;
	void run();
  
	bool deactivate_gripper();
	bool activate_gripper();
	bool get_gripper_status();
	bool set_gripper_status();
public:
    RobotiqGripper140(char* dev_name);
  
    float getFingerCurrent(); 
        
    virtual int is_grasping();
};
