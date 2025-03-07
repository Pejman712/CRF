#include <Gripper/Gripper.hpp>
#include <boost/concept_check.hpp>

RobotArmGripper::Gripper::Gripper(float minimum_limit, float maximum_limit)
{
  this->minimum_limit = minimum_limit;
  this->maximum_limit = maximum_limit;
}


void RobotArmGripper::Gripper::setPosition(float position)
{
	control_mode = POSITION_GRIPPER_CONTROL;
	if ((position >= minimum_limit)&&(position <= maximum_limit)) {
		target_position = position;
	}  
}

void RobotArmGripper::Gripper::setVelocity(float velocity)
{
	control_mode = VELOCITY_GRIPPER_CONTROL;
	target_velocity = velocity;
}

float RobotArmGripper::Gripper::getActualPosition()
{
  return actual_position;
}

void RobotArmGripper::Gripper::open()
{
  target_position = maximum_limit;
}


void RobotArmGripper::Gripper::close()
{
  target_position = minimum_limit;
}

