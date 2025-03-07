/* © Copyright CERN 2022. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Laura Rodrigo Pérez CERN BE/CEM/MRO 2022
 *
 *  ================================================================================================================
*/

#include <math.h>

#include "InverseKinematics/IKinematicObjectiveFunction.hpp"
#include "InverseKinematics/OptCLIK/OptCLIK.hpp"
#include "Robot/RobotConfiguration.hpp"

using crf::control::inversekinematics::IClosedLoopInverseKinematics;
using crf::control::inversekinematics::OptCLIK;

int main(int argc, char* argv[]) {
    crf::utility::logger::EventLogger logger("BasicOptCLIKFunctionsSample");
    /**
     * @brief
     * 
     * This is a sample of use of both OptCLIK functions: getExtendedResults() and getResults().
     * 
     * zDesired corresponds to the configuration qInitial, thus the initial error is 0 and the flag
     * successful will be returned.
     */

    /**
     * 1. Configuration file reading
     * ==============================
     */
    // Path to the config file
    std::string dirName = __FILE__;
    dirName = dirName.substr(0, dirName.find("InverseKinematics"));
    dirName += "ForwardKinematics/samples/MathExprForwardKinematics/";
    std::ifstream robotData(dirName + "UR10eSimFKLengthsJacobian.json");
    nlohmann::json robotJSON = nlohmann::json::parse(robotData);

    /**
     * 2. Parameters of OptCLIK ctor
     * ==============================
     */
    JointPositions qInitial(
        {5*M_PI/180, 5*M_PI/180, 5*M_PI/180, 5*M_PI/180, 5*M_PI/180, 5*M_PI/180});
    std::chrono::microseconds cycleTime(2000);  // 0.002 seconds
    auto configuration = std::make_shared<crf::actuators::robot::RobotConfiguration>(robotJSON);
    std::vector<double> diagW({500, 500, 500, 0.05, 0.05, 0.05});
    std::vector<std::shared_ptr<crf::control::inversekinematics::IKinematicObjectiveFunction>> objFunVector(0);  // NOLINT
    TaskPose tolerance(
        Eigen::Vector3d({0.01, 0.01, 0.01}),
        crf::math::rotation::CardanXYZ({2*M_PI/180, 2*M_PI/180, 2*M_PI/180}));
    double K = 500;
    double kinManip0 = 0.00017;
    double alpha0 = 0.001;

    /**
     * 3. OptCLIK CTor
     * ================
     */
    std::unique_ptr<IClosedLoopInverseKinematics> optCLIK = std::make_unique<OptCLIK>(
        qInitial, cycleTime, configuration, diagW, objFunVector, tolerance, K, kinManip0, alpha0);

    /**
     * 4. Parameters of OptCLIK functions
     * ===================================
     */
    TaskPose z(
        Eigen::Vector3d({-1.1223464, -0.38955782, -0.090344234}),
        Eigen::AngleAxisd(0.97109227, Eigen::Vector3d({-0.16927254, 0.16830511, 1.6231879})));
    TaskVelocity zd({0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    JointPositions qAttr({0.0, 0.0, 0.0, 0.0, 0.0, 0.0});

    /**
     * 5. Function getResults()
     * =========================
     */
    std::cout << "    Function getResults() :" << std::endl << std::endl;
    std::tuple<JointPositions, JointVelocities, JointAccelerations,
        crf::control::inversekinematics::ResultFlags> result(optCLIK->getResults(qAttr, z, zd));
    if (std::get<3>(result) ==
        crf::control::inversekinematics::ResultFlags::workspaceViolation) {
        std::cout << "ERROR: The desired end-effector position is outside the workspace of the "
            "robot." << std::endl;
    } else if (std::get<3>(result) ==
        crf::control::inversekinematics::ResultFlags::endEffectorToleranceViolation) {
        std::cout << "ERROR: The desired end-effector position has NOT been achieved within the "
            "desired position tolerance during the cycleTime established." << std::endl;
    } else if (std::get<3>(result) ==
        crf::control::inversekinematics::ResultFlags::lowManipulability) {
        std::cout << "WARN: The robot is close to a singularity." << std::endl;
    } else if (std::get<3>(result) ==
        crf::control::inversekinematics::ResultFlags::success) {
        std::cout << "SUCCESS: The desired end-effector position has been achieved within the "
            "desired position tolerance during the cycleTime established." << std::endl;
    } else {
        std::cout << "Flag unexpected." << std::endl;
    }
    std::cout << "q = " << std::get<0>(result) << std::endl
        << "qd = " << std::get<1>(result) << std::endl << std::endl << std::endl;

    /**
     * 6. Function getExtendedResults()
     * =================================
     */
    std::cout << "    Function getExtendedResults() :" << std::endl << std::endl;
    crf::control::inversekinematics::ResultsIK resultWithDetails(
        optCLIK->getExtendedResults(qAttr, z, zd));
    if (resultWithDetails.flag() ==
        crf::control::inversekinematics::ResultFlags::workspaceViolation) {
        std::cout << "ERROR: The desired end-effector position is outside the workspace of the "
            "robot." << std::endl;
    } else if (resultWithDetails.flag() ==
        crf::control::inversekinematics::ResultFlags::endEffectorToleranceViolation) {
        std::cout << "ERROR: The desired end-effector position has NOT been achieved within the "
            "desired position tolerance during the cycleTime established." << std::endl;
    } else if (resultWithDetails.flag() ==
        crf::control::inversekinematics::ResultFlags::lowManipulability) {
        std::cout << "WARN: The robot is close to a singularity." << std::endl;
    } else if (resultWithDetails.flag() ==
        crf::control::inversekinematics::ResultFlags::success) {
        std::cout << "SUCCESS: The desired end-effector position has been achieved within the "
            "desired position tolerance during the cycleTime established." << std::endl;
    } else {
        std::cout << "Flag unexpected." << std::endl;
    }
    std::cout << "q = " << resultWithDetails.qResult() << std::endl
        << "qd = " << resultWithDetails.qdResult() << std::endl;

    return 0;
}
