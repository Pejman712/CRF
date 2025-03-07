/* © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Laura Rodrigo Pérez CERN BE/CEM/MRO 2021
 *
 *  ================================================================================================================
*/

#include <Eigen/Dense>
#include <vector>

#include "ForwardKinematics/MathExprForwardKinematics/MathExprForwardKinematics.hpp"
#include "Robot/RobotConfiguration.hpp"

using crf::control::forwardkinematics::MathExprForwardKinematics;
using crf::utility::types::JointPositions;
using crf::utility::types::JointVelocities;
using crf::utility::types::JointAccelerations;
using crf::utility::types::TaskPose;
using crf::utility::types::TaskVelocity;
using crf::utility::types::TaskAcceleration;

int main(int argc, char* argv[]) {
    crf::utility::logger::EventLogger logger("MathExprForwardKinematicsSample");

    std::vector<double> lx, ly, lz;
    nlohmann::json jsonConfig;

    std::string dirName = __FILE__;
    dirName = dirName.substr(0, dirName.find("MathExprForwardKinematicsSample.cpp"));

    // // Option 1
    // std::ifstream robotData(dirName + "UR10ForwardKinematics.json");
    // jsonConfig = nlohmann::json::parse(robotData);
    // lx = {0.0, 0.0, 0.6127, 0.57155, 0.0, 0.0};
    // std::cout << "lx: {" << lx[0] << ", " << lx[1] << ", " << lx[2] << ", " << lx[3] << ", "
    //     << lx[4] << ", " << lx[5] << "}" << std::endl;
    // ly = {0.0, 0.0, 0.0, 0.0, 0.0, 0.11655};
    // std::cout << "ly: {" << ly[0] << ", " << ly[1] << ", " << ly[2] << ", " << ly[3] << ", "
    //     << ly[4] << ", " << ly[5] << "}" << std::endl;
    // lz = {0.0, 0.1807, 0.0, 0.0, 0.17415, 0.11985};
    // std::cout << "lz: {" << lz[0] << ", " << lz[1] << ", " << lz[2] << ", " << lz[3] << ", "
    //     << lz[4] << ", " << lz[5] << "}" << std::endl;

    // Option 2
    std::ifstream robotData(dirName + "UR10eSimFKLengthsJacobian.json");
    nlohmann::json robotJSON = nlohmann::json::parse(robotData);
    auto configuration = std::make_unique<crf::actuators::robot::RobotConfiguration>(robotJSON);
    auto forwardKinematics = configuration->getForwardKinematics();

    JointPositions q = JointPositions({0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    std::cout << std::endl << "q: {" << q[0] << ", " << q[1] << ", " << q[2] << ", "
        << q[3] << ", " << q[4] << ", " << q[5] << "}" << std::endl;
    const TaskPose z = forwardKinematics->getPose(q).value();
    std::cout << "z: " << z << std::endl;
    Eigen::Matrix4d zMatrix = z.getHomogeneousTransformationMatrix();
    std::cout << "zMatrix: {" << zMatrix(0, 0) << ", " << zMatrix(0, 1) << ", " << zMatrix(0, 2)
        << ", " << zMatrix(0, 3) << ",\n\t  " << zMatrix(1, 0) << ", " << zMatrix(1, 1) << ", "
        << zMatrix(1, 2) << ", " << zMatrix(1, 3) << ",\n\t  " << zMatrix(2, 0) << ", "
        << zMatrix(2, 1) << ", " << zMatrix(2, 2) << ", " << zMatrix(2, 3) << ",\n\t  "
        << zMatrix(3, 0) << ", " << zMatrix(3, 1) << ", " << zMatrix(3, 2) << ", "
        << zMatrix(3, 3) << "}" << std::endl;

    JointVelocities qd = JointVelocities({0.0, 0.4, 0.2, 0.0, 0.0, 0.0});
    std::cout << std::endl << "qd: {" << qd[0] << ", " << qd[1] << ", " << qd[2] << ", "
        << qd[3] << ", " << qd[4] << ", " << qd[5] << "}" << std::endl;
    std::optional<TaskVelocity> zDot = forwardKinematics->getVelocity(q, qd);
    std::cout << "zDot: {" << zDot.value()[0] << ", " << zDot.value()[1] << ", "
        << zDot.value()[2] << ", " << zDot.value()[3] << ", " << zDot.value()[4] << ", "
        << zDot.value()[5] << "}" << std::endl;

    JointAccelerations qdd = JointAccelerations({0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    std::cout << std::endl << "qdd: {" << qdd[0] << ", " << qdd[1] << ", " << qdd[2] << ", "
        << qdd[3] << ", " << qdd[4] << ", " << qdd[5] << "}" << std::endl;
    std::optional<TaskAcceleration> z2Dot = forwardKinematics->getAcceleration(q, qd, qdd);
    std::cout << "z2Dot: {" << z2Dot.value()[0] << ", " << z2Dot.value()[1] << ", "
        << z2Dot.value()[2] << ", " << z2Dot.value()[3] << ", " << z2Dot.value()[4] << ", "
        << z2Dot.value()[5] << "}" << std::endl << std::endl;

    return 0;
}
