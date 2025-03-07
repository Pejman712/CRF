/* © Copyright CERN 2022. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Laura Rodrigo Pérez CERN BE/CEM/MRO 2022
 *
 *  ================================================================================================================
*/

#include <math.h>

#include "InverseKinematics/CollisionAvoidanceSphere/CollisionAvoidanceSphere.hpp"
#include "InverseKinematics/JointLimits/JointLimits.hpp"
#include "InverseKinematics/DesiredJointPositions/DesiredJointPositions.hpp"
#include "InverseKinematics/OptCLIK/OptCLIK.hpp"
#include "Robot/RobotConfiguration.hpp"
#include "GeometricMethods/Sinusoid/Sinusoid.hpp"

using crf::control::inversekinematics::OptCLIK;

int main(int argc, char* argv[]) {
    crf::utility::logger::EventLogger logger(
        "BasicOptCLIKTrajectoryReduced2DTaskSpace3DOFRobotSample");
    /**
     * @brief
     * 
     * Robot with 3 DOF (q1, q2, q3) in a task space of 2 DOF (x, y). Task spaces with less than
     * 6 DOF are handled as reduced dimension task spaces. Thus, the TaskPose variables have
     * to be constructed in custom mode.
     * This combination of robot and task space leads to a redundant system, thus objective
     * functions will be applied.
     * 
     * The trajectory of the end-effector will be a linear movement in y-direction from the starting
     * position (defined by the initial joints configuration) and the desired final position.
     * 
     * @param q1Init, q2Init, q3Init
     *        Initial configuration of the robot in radians.
     * 
     * @param yDes
     *        Desired final postion of the end-effector in y-direction of the task space in metres.
     * 
     * @param maxVelEE
     *        Maximum velocity of the end-effector during the trajectory.
     */

    /**
     * 1. Set robot initial joints position and desired task position
     * ===============================================================
     */
    double q1Init, q2Init, q3Init, yDes, maxVelEE;
    if (argc == 6) {
        q1Init = std::stof(argv[1]);
        q2Init = std::stof(argv[2]);
        q3Init = std::stof(argv[3]);
        yDes = std::stof(argv[4]);
        maxVelEE = std::stof(argv[5]);
    } else {
        q1Init = M_PI/4.0;
        q2Init = -M_PI/8.0;
        q3Init = M_PI/8.0;
        yDes = -1.7969;
        maxVelEE = 0.05;
    }
    std::cout << "Inputs:" << std::endl << "q1Init = " << q1Init << " rad\tq2Init = " << q2Init
        << " rad\tq3Init = " << q3Init << " rad" << std::endl << "yDes = " << yDes << " m"
        << std::endl << "maxVelEE = " << maxVelEE << " m/s" << std::endl << std::endl;

    /**
     * 2. Configuration file reading
     * ==============================
     */
    // Path to the config file
    std::string dirName = __FILE__;
    dirName = dirName.substr(0, dirName.find("Control"));
    dirName += "Actuators/Robot/config/SampleRobots/";
    std::ifstream robotData(dirName + "3DOFRobotConfig.json");
    nlohmann::json robotJSON = nlohmann::json::parse(robotData);

    /**
     * 3. Objective functions
     * ======================
     */
    double cycleTimeSeconds = 0.01;
    // Joint limits
    double rangeSinusoidL = 20.0;
    double cL = 3.0;
    double pL = 1.0;
    crf::utility::types::JointPositions minLimit({-M_PI/3.0, -M_PI/2.5, 0.0});
    crf::utility::types::JointPositions maxLimit({M_PI/3.0, 0.0, M_PI/2.5});
    std::shared_ptr<crf::control::inversekinematics::IKinematicObjectiveFunction> jointLimits =
        std::make_unique<crf::control::inversekinematics::JointLimits>(
        rangeSinusoidL, cycleTimeSeconds, cL, pL, minLimit, maxLimit);
    // Collision avoidance
    double rangeSinusoidC = 20.0;
    int curveTypeC = 1;  // Exponential
    double cC = 10.0;
    double pC = 1.0;
    Eigen::Vector3d center(1.0, -1.5, 0.0);
    double radius = 0.25;
    int robot = 1;  // 3 DOF robot
    std::shared_ptr<crf::control::inversekinematics::CollisionAvoidanceSphere> collisionAvoidanceSphere =  // NOLINT
        std::make_shared<crf::control::inversekinematics::CollisionAvoidanceSphere>(
        rangeSinusoidC, cycleTimeSeconds, curveTypeC, cC, pC, center, radius, robot);
    // Vector of objective functions
    std::vector<std::shared_ptr<crf::control::inversekinematics::IKinematicObjectiveFunction>> objFunVector(2);  // NOLINT
    objFunVector[0] = jointLimits;
    objFunVector[1] = collisionAvoidanceSphere;

    /**
     * 4. Parameters of OptCLIK ctor
     * ==============================
     */
    JointPositions qInit({q1Init, q2Init, q3Init});
    std::cout << "qInit = [" << qInit << "]" << std::endl;
    std::chrono::microseconds cycleTime(static_cast<int>(cycleTimeSeconds * 1000000));
    std::shared_ptr<crf::actuators::robot::RobotConfiguration> configuration =
        std::make_shared<crf::actuators::robot::RobotConfiguration>(robotJSON);
    std::vector<double> diagW({1.0, 1.0, 1.0});
    TaskPose tolerance(
        Eigen::Vector3d({0.01, 0.01, 0.0}), crf::math::rotation::CardanXYZ({0.0, 0.0, 0.0}));
    double K = 150;
    double kinManip0 = 0.002;
    double alpha0 = 0.001;
    // OptCLIK CTor
    std::unique_ptr<OptCLIK> optCLIK = std::make_unique<OptCLIK>(
        qInit, cycleTime, configuration, diagW, objFunVector, tolerance, K, kinManip0, alpha0);

    /**
     * 5. Declaration of the q attractor variable needed for the getResults OptCLIK function
     * ======================================================================================
     */
    JointPositions qAttr({std::nan(""), std::nan(""), std::nan("")});

    /**
     * 6. Forward Kinematics from qInitial to obtain the zStart (initial End-Effector Position in
     *    Task Space)
     * ===========================================================================================
     */
    std::shared_ptr<crf::control::forwardkinematics::IForwardKinematics> forwardKinematics =
        configuration->getForwardKinematics();
    std::optional<TaskPose> zInit = forwardKinematics->getPose(qInit);
    if (!zInit) {
        std::cout << "Error obtaining position with forward kinematics" << std::endl;
        return -1;
    }
    std::cout << "zInit (axis-angle) = ["
        << zInit.value().getPosition()(0) << ", "
        << zInit.value().getPosition()(1) << "]"
        << std::endl << std::endl;

    /**
     * 7. Vertical trajectory of the EE based on the current qInit position
     * =====================================================================
     */
    double xInit = zInit.value().getPosition()(0);
    double yInit = zInit.value().getPosition()(1);
    double xDes = xInit;
    std::shared_ptr<crf::math::geometricmethods::Sinusoid> trajectoryBuilder =
        std::make_shared<crf::math::geometricmethods::Sinusoid>(yInit, yDes, maxVelEE);
    std::optional<double> duration = trajectoryBuilder->getRange();
    if (!duration) {
        std::cout << "Duration: Failed to calculate the parameters" << std::endl;
        return -1;
    }

    /**
     * 8. Computation of the number of cycles needed based on the trajectory duration and the
     *    cycleTimeSeconds (time expended in one cycle)
     * =======================================================================================
     */
    int cycles = (duration.value() / cycleTimeSeconds) + 1;
    std::cout << "The number of cycles is " << cycles << " ("
        << (duration.value() / cycleTimeSeconds) + 1 << ")" << std::endl;

    /**
     * 9. Declaration of the storage variables used in the loop
     * =========================================================
     */
    std::vector<double> time(cycles, 0.0);
    std::vector<crf::utility::types::TaskPose> zDes(cycles,
        crf::utility::types::TaskPose());
    std::vector<crf::utility::types::TaskVelocity> zdDes(cycles,
        crf::utility::types::TaskVelocity());
    std::vector<crf::utility::types::TaskAcceleration> zddDes(cycles,
        crf::utility::types::TaskAcceleration());
    std::vector<crf::utility::types::JointPositions> q(cycles,
        crf::utility::types::JointPositions(3));
    std::vector<crf::utility::types::JointVelocities> qd(cycles,
        crf::utility::types::JointVelocities(3));
    std::vector<unsigned int> resFlag(cycles, 0);

    /**
     * 10. Creation of the storage file
     * =================================
     */
    std::string filename = __FILE__;
    filename = filename.substr(0, filename.find(
        "BasicOptCLIKTrajectoryReduced2DTaskSpace3DOFRobotSample.cpp"));
    filename += "BasicOptCLIKTrajectoryReduced2DTaskSpace3DOFRobot.csv";
    std::ofstream fout(filename);
    fout  << "time" << ','
        << "zDesX" << ',' << "zDesY" << ',' << "zdDesX" << ',' << "zdDesY" << ','
        << "zddDesX" << ',' << "zddDesY" << ','
        << "q1" << ',' << "q2" << ',' << "q3" << ',' << "qd1" << ',' << "qd2" << ',' << "qd3" << ','
        << "flag" << '\n';

    /**
     * 11. Loop start
     * ===============
     */
    for (int i = 0; i < cycles; i++) {
        time[i] = i * cycleTimeSeconds;

        /**
         * 12. Enabling / Disabling of the objective functions
         * ====================================================
         */
        if (time[i] == 0.0) {
            if (!jointLimits->enable(true)) logger->error("Error during enabling");
        }
        if (time[i] == 80.0) {
            if (!jointLimits->enable(false)) logger->error("Error during disabling");
        }
        if (time[i] == 40.0) {
            if (!collisionAvoidanceSphere->enable(true)) logger->error("Error during enabling");
        }

        /**
         * 13. Creation of the trajectory
         * ===============================
         */
        zDes[i].getPosition()(0) = xDes;
        zDes[i].getPosition()(1) = trajectoryBuilder->evaluate(time[i], 0).value();
        zdDes[i][0] = 0.0;
        zdDes[i][1] = trajectoryBuilder->evaluate(time[i], 1).value();
        zddDes[i][0] = 0.0;
        zddDes[i][1] = trajectoryBuilder->evaluate(time[i], 2).value();

        /**
         * 14. Results of inverse kinematics
         * ==================================
         */
        std::tuple<JointPositions, JointVelocities, JointAccelerations,
            crf::control::inversekinematics::ResultFlags> optCLIKResult(
            optCLIK->getResults(qAttr, zDes[i], zdDes[i]));
        if (std::get<3>(optCLIKResult) ==
            crf::control::inversekinematics::ResultFlags::workspaceViolation) {
            resFlag[i] = 5;
        } else if (std::get<3>(optCLIKResult) ==
            crf::control::inversekinematics::ResultFlags::endEffectorToleranceViolation) {
            resFlag[i] = 3;
        } else if (std::get<3>(optCLIKResult) ==
            crf::control::inversekinematics::ResultFlags::lowManipulability) {
            resFlag[i] = 2;
        } else if (std::get<3>(optCLIKResult) ==
            crf::control::inversekinematics::ResultFlags::success) {
            resFlag[i] = 1;
        } else {
            resFlag[i] = 0;
        }
        q[i] = std::get<0>(optCLIKResult);
        qd[i] = std::get<1>(optCLIKResult);

        /**
         * 19. Store the computed information
         * ===================================
         */
        // fout << std::setprecision(16);
        fout << time[i] << ','
            << zDes[i].getPosition()(0) << ','
            << zDes[i].getPosition()(1) << ','
            << zdDes[i][0] << ',' << zdDes[i][1] << ','
            << zddDes[i][0] << ',' << zddDes[i][1] << ','
            << q[i][0] << ',' << q[i][1] << ',' << q[i][2] << ','
            << qd[i][0] << ',' << qd[i][1] << ',' << qd[i][2] << ','
            << resFlag[i] << '\n';
    }
    fout.close();
    std::cout << "qEnd = [" << q[cycles-1] << "]" << std::endl << std::endl;
    std::cout << "Info of the sample stored in:" << std::endl << filename << std::endl;

    return 0;
}
