/* © Copyright CERN */

#include "YoubotArm/YoubotArm.hpp"

using namespace std;  //NOLINT
using namespace youbot;  //NOLINT

YoubotArm::YoubotArm() : RobotArm(5, 0) {
    float offset_j1 =  169.0/180*M_PI;
    float offset_j2 =   65.0/180*M_PI + M_PI/2;
    float offset_j3 = -146.0/180*M_PI;
    float offset_j4 =  102.5/180*M_PI + M_PI/2;
    float offset_j5 =  167.5/180*M_PI + M_PI;

    joint_maximum_speed = 1;

    chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),
        KDL::Frame::DH(0.0570, M_PI/2, 0.115, 169.0/180*M_PI)));
    chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),
        KDL::Frame::DH(0.1550, .0, 0.0, 65.0/180*M_PI + M_PI/2)));
    chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),
        KDL::Frame::DH(0.1350, .0, 0.0, -146.0/180*M_PI)));
    chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),
        KDL::Frame::DH(0.0, M_PI/2, .0, 102.5/180*M_PI + M_PI/2)));
    chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),
        KDL::Frame::DH(0.0, .0, 0.130, 167.5/180*M_PI + M_PI)));

    min_angles = new std::vector<double>();
    min_angles->push_back(0.0100692);
    min_angles->push_back(0.0100692);
    min_angles->push_back(-5.02655);
    min_angles->push_back(0.0221239);
    min_angles->push_back(0.110619);
    max_angles = new std::vector<double>();
    max_angles->push_back(5.84014);
    max_angles->push_back(2.61799);
    max_angles->push_back(-0.015708);
    max_angles->push_back(3.4292);
    max_angles->push_back(5.64159);

    ik = new InverseKinematics(*min_angles, *max_angles);
    arm_position = new Matrix<float> (6, 1, .0);

    arm_active = 0;
    try {
        arm  = new YouBotManipulator("youbot-manipulator", "/home/youbot/youbot_driver/config");
        arm->doJointCommutation();
        arm->calibrateManipulator();
        arm_active = 1;
    } catch( exception& ex) {
        cout << "WHAT: " << ex.what() << endl;
    }
}

Matrix<float> YoubotArm::getJointPositions() {
    Matrix<float> joint_pos(5, 1, .0);
    if (arm_active == 1) {
        youbot::JointSensedAngle sensedValue[5];
        arm->getArmJoint(1).getData(sensedValue[0]);
        arm->getArmJoint(2).getData(sensedValue[1]);
        arm->getArmJoint(3).getData(sensedValue[2]);
        arm->getArmJoint(4).getData(sensedValue[3]);
        arm->getArmJoint(5).getData(sensedValue[4]);

        joint_pos(0, 0) = sensedValue[0].angle.value();
        joint_pos(1, 0) = sensedValue[1].angle.value();
        joint_pos(2, 0) = sensedValue[2].angle.value();
        joint_pos(3, 0) = sensedValue[3].angle.value();
        joint_pos(4, 0) = sensedValue[4].angle.value();
    }

    return joint_pos;
}

Matrix<float> YoubotArm::getJointVelocities() {
    Matrix<float> joint_pos(5, 1, .0);
    if (arm_active == 1) {
        youbot::JointSensedVelocity sensedValue[5];
        arm->getArmJoint(1).getData(sensedValue[0]);
        arm->getArmJoint(2).getData(sensedValue[1]);
        arm->getArmJoint(3).getData(sensedValue[2]);
        arm->getArmJoint(4).getData(sensedValue[3]);
        arm->getArmJoint(5).getData(sensedValue[4]);

        joint_pos(0, 0) = sensedValue[0].angularVelocity.value();
        joint_pos(1, 0) = sensedValue[1].angularVelocity.value();
        joint_pos(2, 0) = sensedValue[2].angularVelocity.value();
        joint_pos(3, 0) = sensedValue[3].angularVelocity.value();
        joint_pos(4, 0) = sensedValue[4].angularVelocity.value();
    }
    return joint_pos;
}

Matrix<float> YoubotArm::getJointsCurrent() {
    Matrix<float> joint_pos(5, 1, .0);
    if (arm_active == 1) {
        youbot::JointSensedCurrent sensedValue[5];
        arm->getArmJoint(1).getData(sensedValue[0]);
        arm->getArmJoint(2).getData(sensedValue[1]);
        arm->getArmJoint(3).getData(sensedValue[2]);
        arm->getArmJoint(4).getData(sensedValue[3]);
        arm->getArmJoint(5).getData(sensedValue[4]);

        joint_pos(0, 0) = sensedValue[0].current.value();
        joint_pos(1, 0) = sensedValue[1].current.value();
        joint_pos(2, 0) = sensedValue[2].current.value();
        joint_pos(3, 0) = sensedValue[3].current.value();
        joint_pos(4, 0) = sensedValue[4].current.value();
    }
    return joint_pos;
}

void YoubotArm::setJointPositions(Matrix<float> final_position) {
    if (arm_active) {
        youbot::JointAngleSetpoint setAngle;
        for (int i = 0; i < 5; i++) {
            setAngle.angle = final_position(i, 0) * radian;
            arm->getArmJoint(i+1).setData(setAngle);
        }
    }
}

void YoubotArm::setJointVelocities(Matrix<float> joint_velocity) {
    if (arm_active) {
        youbot::jointVelocitiesSetpoint jVelocity;
        for (int i = 0; i < 5; i++) {
            jVelocity.angularVelocity = joint_velocity(i, 0) * radians_per_second;
            arm->getArmJoint(i+1).setData(jVelocity);
        }
    }
}
void YoubotArm::setJointsCurrent(Matrix<float>) {
}

void YoubotArm::openGripper() {
    youbot::GripperBarSpacingSetPoint barSpacing;
    if (arm_active) {
        try {
            barSpacing.barSpacing = 0.0115 * meter;
            arm->getArmGripper().setData(barSpacing);
        } catch( exception& ex) {
            cout << "WHAT: " << ex.what() << endl;
        }
    }
}

void YoubotArm::closeGripper() {
    youbot::GripperBarSpacingSetPoint barSpacing;
    if (arm_active) {
        try {
            barSpacing.barSpacing = 0.0 * meter;
            arm->getArmGripper().setData(barSpacing);
        } catch( exception& ex) {
            cout << "WHAT: " << ex.what() << endl;
        }
    }
}

void YoubotArm::setArmPosition(Matrix<float> frame) {
    Matrix<float> actual_joint_pos = getJointPositions();
    Matrix<float> inverse = armPositionInverseKinematic(frame, actual_joint_pos);
    inverse.print();
    setJointPositions(inverse);
}

void YoubotArm::setArmPositionIK(Matrix<float> position) {
    Matrix<float> joint_position = getJointPositions();

    KDL::JntArray q_ini(5);
    for (int i = 0; i < 5; i++) {
        q_ini(i) = static_cast<double>(joint_position(i, 0));
    }

    KDL::Vector p;
    p.x(position(0, 0));
    p.y(position(1, 0));
    p.z(position(2, 0));

    KDL::Rotation r = KDL::Rotation::EulerZYX(position(3, 0), position(4, 0), position(5, 0));

    KDL::Frame p_in(r, p);

    vector<KDL::JntArray> q_out;
    int ret = ik->TaskToJnt(q_ini, p_in, q_out);
    if (ret > 0) {
        Matrix<float> final_position_matrix(5, 1, .0);
        for (int i = 0; i < 5; i++) {
            final_position_matrix(i, 0) = static_cast<float>(q_out[0])(i);
        }

        setJointPositions(final_position_matrix);

        for (int i = 0; i < 6; i++) {
            arm_position->at(i, 0) = position.at(i, 0);
        }
    } else {
        printf("No solution found\n");
    }
}

void YoubotArm::setArmVelocity(Matrix< float > velocity, Matrix< float > rotation, bool TCP) {
    Matrix<float> actual_joint_pos = getJointPositions();

    if (TCP) {
        Matrix<float> forwardKinematic = getArmPosition();
        velocity = forwardKinematic.getPart(0, 2, 0, 2)*velocity;
        rotation = forwardKinematic.getPart(0, 2, 0, 2)*rotation;
    }

    Matrix<float> inverse = armVelocityInverseKinematic(velocity, rotation, actual_joint_pos);
    for (int i = 0; i < 5; i++) {
        if (fabs(inverse(i, 0)) > joint_maximum_speed) {
            float division_factor = fabs(inverse(i, 0))/joint_maximum_speed;
            for (int i = 0; i < 6; i++) {
                inverse(i, 0) = inverse(i, 0)/division_factor;
            }
        }
    }
    gettimeofday(&last_command_time, NULL);

    setJointVelocities(inverse);
}

Matrix<float> YoubotArm::getArmPosition() {
    Matrix<float> actual_joint_pos = getJointPositions();
    Matrix<float> position = armPositionForwardKinematic(actual_joint_pos);
    return position;
}

Matrix<float> YoubotArm::getArmVelocity() {
    Matrix<float> actual_joint_pos = getJointPositions();
    Matrix<float> actual_joint_vel = getJointVelocities();
    Matrix<float> velocity = armVelocityForwardKinematic(actual_joint_vel, actual_joint_pos);
    return velocity;
}
