#include "RobotArm/RobotArm.hpp"
#include "utility/Matrix.hpp"

#include "youbot/YouBotManipulator.hpp"
#include "YoubotArm/InverseKinematic.hpp"
#include <iostream>
#include <vector>

class YoubotArm : public RobotArm {
	youbot::YouBotManipulator* arm;
	InverseKinematics * ik;
	
	std::vector<double>* min_angles, *max_angles;
	
	int arm_active;
public:
	YoubotArm();
	Matrix<float>* arm_position;

	virtual Matrix<float> getJointPositions();
	virtual Matrix<float> getJointVelocities();
	virtual Matrix<float> getJointsCurrent();

	virtual void setJointPositions(Matrix<float>);
	virtual void setJointVelocities(Matrix<float>);
	virtual void setJointsCurrent(Matrix<float>);

	virtual void openGripper();
	virtual void closeGripper();
	
	virtual void stopArm() {};
	
	virtual void setArmPosition(Matrix<float>) ;
	void setArmPositionIK(Matrix<float>);
	virtual void setArmVelocity( Matrix< float >,  Matrix< float >, bool TCP);
	
	virtual Matrix<float> getArmPosition();
	virtual Matrix<float> getArmVelocity();	
};
