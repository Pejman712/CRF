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
    crf::utility::logger::EventLogger logger("ExpertOptCLIKFunctionsSample");
    /**
     * @brief
     * 
     * This is a sample of use of both OptCLIK functions: getExtendedResults() and getResults().
     * You can see the different flags you can obtain depending on the input parametres.
     * 
     * In this sample also the time spent in each function can be appreciated. The first function
     * called is allways spending more time, independently of the OptCLIK function executed first.
     * 
     * 
     * @param sample
     *        Sample 1: zDesired corresponds to the configuration qInitial, thus the initial error
     *                  is 0 and the flag successful will be returned.
     *        Sample 2: The error between zDesired and zInitial (corresponding to qInitial) is 10cm
     *                  in x-direction, so the distance is too big and the error flag
     *                  endEffectorToleranceViolation will be returned.
     */

    int sample = 1;
    if (argc == 2) {
        sample = std::stoi(argv[1]);
    }
    if (sample == 1) {
        std::cout << "Sample 1: zDesired corresponds to the configuration qInitial, thus the "
            "initial error is 0 and the flag successful will be returned." << std::endl
            << std::endl;
    } else if (sample == 2) {
        std::cout << "Sample 2: The error between zDesired and zInitial (corresponding to "
            "qInitial) is 10cm in x-direction, so the distance is too big and the error flag "
            "endEffectorToleranceViolation will be returned." << std::endl << std::endl;
    } else {
        std::cout << "Sample number " << sample << " doesn't exist." << std::endl;
        return -1;
    }

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
    JointPositions qInitial;
    if (sample == 1) {
        qInitial = JointPositions({5*M_PI/180, 5*M_PI/180, 5*M_PI/180,
            5*M_PI/180, 5*M_PI/180, 5*M_PI/180});
    } else {  // Sample = 2
        qInitial = JointPositions({5*M_PI/180, -14.78*M_PI/180, 47.51*M_PI/180,
            -17.73*M_PI/180, 5*M_PI/180, 5*M_PI/180});
    }
    std::chrono::microseconds cycleTime(2000);  // 0.002 seconds
    auto configuration = std::make_shared<crf::actuators::robot::RobotConfiguration>(robotJSON);
    std::vector<double> diagW({500, 500, 500, 0.05, 0.05, 0.05});
    std::vector<std::shared_ptr<crf::control::inversekinematics::IKinematicObjectiveFunction>> objFunVector(0);  // NOLINT
    TaskPose tolerance;
    if (sample == 1) {
        tolerance = TaskPose({0.01, 0.01, 0.01},
            crf::math::rotation::CardanXYZ({2*M_PI/180, 2*M_PI/180, 2*M_PI/180}));
    } else {  // Sample = 2
        tolerance = TaskPose({0.0001, 0.0001, 0.0001},
            crf::math::rotation::CardanXYZ({0.0001, 0.0001, 0.0001}));
    }
    double K = 500;
    double kinManip0 = 0.00017;
    double alpha0 = 0.001;

    /**
     * 3. OptCLIK CTor
     * ================
     */
    clock_t t;
    t = clock();
    std::unique_ptr<IClosedLoopInverseKinematics> optCLIK = std::make_unique<OptCLIK>(
        qInitial, cycleTime, configuration, diagW, objFunVector, tolerance, K, kinManip0, alpha0);
    t = clock() - t;
    double dT = static_cast<double>(t)/CLOCKS_PER_SEC;
    std::cout << "OptCLIK ctor took " << dT << " seconds." << std::endl;

    /**
     * 4. Parameters of OptCLIK functions
     * ===================================
     */
    TaskPose z;
    TaskVelocity zd;
    if (sample == 1) {
        z = TaskPose({-1.1223464, -0.38955782, -0.090344234},
            Eigen::AngleAxisd(1.6231879, Eigen::Vector3d({-0.16927254, 0.16830511, 0.97109227})));
        zd = TaskVelocity({0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    } else {  // Sample = 2
        z = TaskPose({0.30459577, -0.14780326, 1.4725429},
            Eigen::AngleAxisd(0.3900936, Eigen::Vector3d({0.01944744, 0.44541964, 0.8951107})));
        zd = TaskVelocity({0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    }
    JointPositions qAttr({0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    std::cout << std::endl << std::endl;

    /**
     * 5. In the following code 3 functions are called: 1st getExtendedResults(), 2nd getResults()
     * and 3rd getExtendedResults() again. By default, the third function is commented out.
     * Launching the default code, shows that the first function getExtendedResults() takes more
     * time to be computed than the 2nd function getResults().
     * If you uncomment the 3rd function and comment the 1st function, you will see that now the
     * function getResults() takes more time.
     * Thus, this demonstrates that always the first function being called takes more time to be
     * computed.
     */

    /**
     * 1st FUNCTION getExtendedResults()
     * ==================================
     */
    clock_t timeGetExtendedResults = clock();
    crf::control::inversekinematics::ResultsIK resultWithDetails(
        optCLIK->getExtendedResults(qAttr, z, zd));
    timeGetExtendedResults = clock() - timeGetExtendedResults;
    double dTimeGetExtendedResults = static_cast<double>(timeGetExtendedResults)/CLOCKS_PER_SEC;
    std::cout << "getExtendedResults() function tooks " << dTimeGetExtendedResults << " seconds."
        << std::endl;
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
        << "qd = " << resultWithDetails.qdResult() << std::endl << std::endl << std::endl;

    /**
     * 2nd FUNCTION getResults()
     * ==========================
     */
    clock_t timeGetResults = clock();
    std::tuple<JointPositions, JointVelocities, JointAccelerations,
        crf::control::inversekinematics::ResultFlags> result(optCLIK->getResults(qAttr, z, zd));
    timeGetResults = clock() - timeGetResults;
    double dTimeGetResults = static_cast<double>(timeGetResults)/CLOCKS_PER_SEC;
    std::cout << "getResults() function tooks " << dTimeGetResults << " seconds." << std::endl;
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
     * 3rd FUNCTION getExtendedResults()
     * ==================================
     */
    // clock_t timeGetExtendedResults = clock();
    // crf::control::inversekinematics::ResultsIK resultWithDetails(
    //     optCLIK->getExtendedResults(qAttr, z, zd));
    // timeGetExtendedResults = clock() - timeGetExtendedResults;
    // double dTimeGetExtendedResults = static_cast<double>(timeGetExtendedResults)/CLOCKS_PER_SEC;
    // std::cout << "getExtendedResults() function tooks " << dTimeGetExtendedResults << " seconds."
    //     << std::endl;
    // if (resultWithDetails.flag() ==
    //     crf::control::inversekinematics::ResultFlags::workspaceViolation) {
    //     std::cout << "ERROR: The desired end-effector position is outside the workspace of the "
    //         "robot." << std::endl;
    // } else if (resultWithDetails.flag() ==
    //     crf::control::inversekinematics::ResultFlags::endEffectorToleranceViolation) {
    //     std::cout << "ERROR: The desired end-effector position has NOT been achieved within the "
    //         "desired position tolerance during the cycleTime established." << std::endl;
    // } else if (resultWithDetails.flag() ==
    //     crf::control::inversekinematics::ResultFlags::lowManipulability) {
    //     std::cout << "WARN: The robot is close to a singularity." << std::endl;
    // } else if (resultWithDetails.flag() ==
    //     crf::control::inversekinematics::ResultFlags::success) {
    //     std::cout << "SUCCESS: The desired end-effector position has been achieved within the "
    //         "desired position tolerance during the cycleTime established." << std::endl;
    // } else {
    //     std::cout << "Flag unexpected." << std::endl;
    // }
    // std::cout << "q = " << resultWithDetails.qResult() << std::endl
    //     << "qd = " << resultWithDetails.qdResult() << std::endl << std::endl << std::endl;

    return 0;
}
