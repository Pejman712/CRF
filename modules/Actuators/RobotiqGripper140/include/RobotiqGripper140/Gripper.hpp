#pragma once
//#ifndef GRIPPER_H
//#define GRIPPER_H

#define VELOCITY_GRIPPER_CONTROL 0x143
#define POSITION_GRIPPER_CONTROL 0x144

namespace RobotArmGripper {
    class Gripper
    {
    protected:
    	int control_mode;
    	
        float actual_position;
        float target_position;
        float target_velocity;
        
        float minimum_limit;
        float maximum_limit;
    public:
        Gripper(float minimum_limit, float maximum_limit);
      
        virtual float getActualPosition();
    	
        
        virtual void setPosition (float position);
        virtual void setVelocity (float velocity);
        
        virtual void close();
        virtual void open();
        
        virtual int is_grasping() = 0;
    	
    	inline float getMaximumLimit() { return maximum_limit; };
    	inline float getMinimumLimit() { return minimum_limit; }
    };
}

//#endif