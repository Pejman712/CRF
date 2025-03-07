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
        "ExpertOptCLIKTrajectoryReduced3DTaskSpace6DOFUR10eRedundantRobotSample");
    /**
     * @brief
     * 
     * UR10e robot with 6 DOF (q1, ..., q6) in a task space with 3 DOF (x, y, z). Task spaces with
     * less than 6 DOF are handled as reduced dimension task spaces. Thus, the TaskPose
     * variables have to be constructed in reduced task space mode.
     * This combination of robot and task space leads to a redundant system, thus objective
     * functions will be applied.
     * 
     * The trajectory of the end-effector will be a movement from the starting position (defined by
     * the initial joints configuration) and the desired final position.
     * 
     * @param q1Init, q2Init, q3Init, q4Init, q5Init, q6Init
     *        Initial configuration of the robot in radians.
     * 
     * @param zXDes, zYDes, zZDes
     *        Desired final postion of the end-effector in the task space in metres.
     * 
     * @param maxVelEE
     *        Maximum velocity of the end-effector during the trajectory.
     */
    int robotDOF = 6, taskSpaceDim = 3;

    /**
     * 1. Set robot initial joints position and desired task position
     * ===============================================================
     */
    double q1Init, q2Init, q3Init, q4Init, q5Init, q6Init, zXDes, zYDes, zZDes, maxVelEE;
    if (argc == 5) {
        q1Init = 0.0; q2Init = 0.0; q3Init = 0.0; q4Init = 0.0; q5Init = 0.0; q6Init = 0.0;
        zXDes = std::stof(argv[1]); zYDes = std::stof(argv[2]); zZDes = std::stof(argv[3]);
        maxVelEE = std::stof(argv[4]);
    } else if (argc == 7) {
        q1Init = std::stof(argv[1]); q2Init = std::stof(argv[2]); q3Init = std::stof(argv[3]);
        q4Init = std::stof(argv[4]); q5Init = std::stof(argv[5]); q6Init = std::stof(argv[6]);
        zXDes = 0.0; zYDes = 0.0; zZDes = 0.0;
        maxVelEE = 0.01;
    } else if (argc == 11) {
        q1Init = std::stof(argv[1]); q2Init = std::stof(argv[2]); q3Init = std::stof(argv[3]);
        q4Init = std::stof(argv[4]); q5Init = std::stof(argv[5]); q6Init = std::stof(argv[6]);
        zXDes = std::stof(argv[7]); zYDes = std::stof(argv[8]); zZDes = std::stof(argv[9]);
        maxVelEE = std::stof(argv[10]);
    } else if ((argc > 1 && argc < 5) || argc == 6 || (argc > 7 && argc < 11) || argc > 11) {
        std::cout << "Wrong number of arguments" << std::endl;
        return -1;
    } else {
        q1Init = 1.0; q2Init = 1.0; q3Init = 1.0; q4Init = 1.0; q5Init = 1.0; q6Init = 1.0;
        zXDes = 0.5; zYDes = 0.5; zZDes = 0.5;
        maxVelEE = 0.01;
    }
    std::cout << "Inputs:" << std::endl
        << "q1Init = " << q1Init << " rad\tq2Init = " << q2Init << " rad\tq3Init = " << q3Init
        << " rad\tq4Init = " << q4Init << " rad\tq5Init = " << q5Init
        << " rad\tq6Init = " << q6Init << " rad" << std::endl
        << "zXDes = " << zXDes << " m\tzYDes = " << zYDes << " m\tzZDes = " << zZDes << " m"
        << std::endl << "maxVelEE = " << maxVelEE << " m/s" << std::endl << std::endl;

    /**
     * 2. Configuration file reading
     * ==============================
     */
    // Path to the config file
    std::string dirName = __FILE__;
    dirName = dirName.substr(0, dirName.find("Control"));
    dirName += "Actuators/Robot/config/UniversalRobot/";
    std::ifstream robotData(dirName + "UR10eIn3DTaskSpaceSimulation.json");
    nlohmann::json robotJSON = nlohmann::json::parse(robotData);

    /**
     * 3. Objective functions
     * ======================
     */
    double cycleTimeSeconds = 0.002;
    // Joint limits
    double rangeSinusoidL = 2.0;
    double cL = 10.0;
    double pL = 10.2;
    crf::utility::types::JointPositions minLimit({0.0, -6.28, -6.28, -6.28, -6.28, std::nan("")});
    crf::utility::types::JointPositions maxLimit({6.28, 6.28, 6.28, 6.28, 6.28, std::nan("")});
    std::shared_ptr<crf::control::inversekinematics::IKinematicObjectiveFunction> jointLimits =
        std::make_unique<crf::control::inversekinematics::JointLimits>(
        rangeSinusoidL, cycleTimeSeconds, cL, pL, minLimit, maxLimit);
    // Desired joints position
    double cP = 40.0;
    double rangeSinusoidP = 5.0;
    std::shared_ptr<crf::control::inversekinematics::IKinematicObjectiveFunction> desiredJointPositions =  // NOLINT
        std::make_shared<crf::control::inversekinematics::DesiredJointPositions>(
        rangeSinusoidP, cycleTimeSeconds, cP);
    // Collision avoidance
    double rangeSinusoidC = 5.0;
    int curveTypeC = 1;  // Exponential
    double cC = 10.0;
    double pC = 1.0;
    Eigen::Vector3d center(0.0, 0.0, 0.3);
    double radius = 0.2;
    int robot = 2;  // UR10e - 6DOF robot
    std::shared_ptr<crf::control::inversekinematics::CollisionAvoidanceSphere> collisionAvoidanceSphere =  // NOLINT
        std::make_shared<crf::control::inversekinematics::CollisionAvoidanceSphere>(
        rangeSinusoidC, cycleTimeSeconds, curveTypeC, cC, pC, center, radius, robot);
    // Vector of objective functions
    std::vector<std::shared_ptr<crf::control::inversekinematics::IKinematicObjectiveFunction>> objFunVector(3);  // NOLINT
    objFunVector[0] = jointLimits;
    objFunVector[1] = desiredJointPositions;
    objFunVector[2] = collisionAvoidanceSphere;

    /**
     * 4. Parameters of OptCLIK ctor
     * ==============================
     */
    JointPositions qInit({q1Init, q2Init, q3Init, q4Init, q5Init, q6Init});
    std::chrono::microseconds cycleTime(static_cast<int>(cycleTimeSeconds * 1000000));
    std::shared_ptr<crf::actuators::robot::RobotConfiguration> configuration =
        std::make_shared<crf::actuators::robot::RobotConfiguration>(robotJSON);
    std::vector<double> diagW({1.0, 1.0, 1.0, 1.0, 1.0, 1.0});
    TaskPose tolerance(Eigen::Vector3d({0.01, 0.01, 0.01}),
        crf::math::rotation::CardanXYZ({0.0, 0.0, 0.0}));
    double K = 500;
    double kinManip0 = 0.001;
    double alpha0 = 0.001;
    // OptCLIK CTor
    clock_t tOptCLIKCtor;
    tOptCLIKCtor = clock();
    std::unique_ptr<OptCLIK> optCLIK = std::make_unique<OptCLIK>(
        qInit, cycleTime, configuration, diagW, objFunVector, tolerance, K, kinManip0, alpha0);
    tOptCLIKCtor = clock() - tOptCLIKCtor;
    std::cout << "OptCLIK ctor took " << static_cast<double>(tOptCLIKCtor)/CLOCKS_PER_SEC
        << " seconds." << std::endl << std::endl;
    std::cout << "qInit = [" << qInit << "]" << std::endl;

    /**
     * 5. Declaration of the q attractor variable needed for the getResults OptCLIK function
     * ======================================================================================
     */
    JointPositions qAttr({std::nan(""), std::nan(""), std::nan(""),
        1.1, std::nan(""), std::nan("")});

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
    std::cout << "zInit = ["
        << zInit.value().getPosition()(0) << ", "
        << zInit.value().getPosition()(1) << ", "
        << zInit.value().getPosition()(2) << "]"
        << std::endl << std::endl;

    /**
     * 7. Trajectory of the EE based on the current qInit position
     * ============================================================
     */
    double zXInit = zInit.value().getPosition()(0);
    double zYInit = zInit.value().getPosition()(1);
    double zZInit = zInit.value().getPosition()(2);
    std::shared_ptr<crf::math::geometricmethods::Sinusoid> trajectoryBuilder =
        std::make_shared<crf::math::geometricmethods::Sinusoid>(
        std::hypot(zXInit, zYInit, zZInit), std::hypot(zXDes, zYDes, zZDes), maxVelEE,
        crf::math::geometricmethods::ComputationMethod::Limit1stDerivative);
    std::optional<double> duration = trajectoryBuilder->getRange();
    if (!duration) {
        std::cout << "Duration: Failed to calculate the parameters" << std::endl;
        return -1;
    }
    std::shared_ptr<crf::math::geometricmethods::Sinusoid> xTrajectoryBuilder =
        std::make_shared<crf::math::geometricmethods::Sinusoid>(zXInit, zXDes, duration.value(),
        crf::math::geometricmethods::ComputationMethod::SetRange);
    std::shared_ptr<crf::math::geometricmethods::Sinusoid> yTrajectoryBuilder =
        std::make_shared<crf::math::geometricmethods::Sinusoid>(zYInit, zYDes, duration.value(),
        crf::math::geometricmethods::ComputationMethod::SetRange);
    std::shared_ptr<crf::math::geometricmethods::Sinusoid> zTrajectoryBuilder =
        std::make_shared<crf::math::geometricmethods::Sinusoid>(zZInit, zZDes, duration.value(),
        crf::math::geometricmethods::ComputationMethod::SetRange);

    /**
     * 8. Computation of the number of cycles needed based on the trajectory duration and the
     *    cycleTimeSeconds (time expended in one cycle)
     * =======================================================================================
     */
    int cycles = (duration.value() / cycleTimeSeconds) + 1;
    std::cout << "The duration is " << duration.value() << " s, and the number of cycles is "
        << cycles << " (" << (duration.value() / cycleTimeSeconds) + 1 << ")"
        << std::endl << std::endl;

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
        crf::utility::types::JointPositions(robotDOF));
    std::vector<crf::utility::types::JointVelocities> qd(cycles,
        crf::utility::types::JointVelocities(robotDOF));
    std::vector<unsigned int> resFlag(cycles, 0);
    std::vector<std::vector<double>> zError(cycles);
    std::vector<Eigen::MatrixXd> penalty(cycles, Eigen::MatrixXd());
    std::vector<std::vector<double>> transitionFactor(cycles);
    std::vector<crf::utility::types::TaskPose> z(cycles,
        crf::utility::types::TaskPose());
    std::vector<crf::utility::types::TaskVelocity> zd(cycles,
        crf::utility::types::TaskVelocity());
    std::vector<std::vector<double>> zDifference(cycles), zdDifference(cycles);
    std::vector<double> measuredWallTimeIK(cycles, 0.0), measuredCPUTimeIK(cycles, 0.0);

    /**
     * 10. Creation of the storage file
     * =================================
     */
    std::string filename = __FILE__;
    filename = filename.substr(0, filename.find("OptCLIKAndObjectiveFunctions/"));
    filename += "OptCLIKAndObjectiveFunctions/";
    filename += "ExpertOptCLIKTrajectoryReduced3DTaskSpace6DOFUR10eRedundantRobot.csv";
    std::ofstream fout(filename);
    fout  << "time" << ','
        << "zDesX" << ',' << "zDesY" << ',' << "zDesZ" << ','
        << "zdDesX" << ',' << "zdDesY" << ',' << "zdDesZ" << ','
        << "zddDesX" << ',' << "zddDesY" << ',' << "zddDesZ" << ','
        << "q1" << ',' << "q2" << ',' << "q3" << ',' << "q4" << ',' << "q5" << ',' << "q6" << ','
        << "qd1" << ',' << "qd2" << ',' << "qd3" << ','
        << "qd4" << ',' << "qd5" << ',' << "qd6" << ','
        << "flag" << ',' << "zErrorX" << ',' << "zErrorY" << ',' << "zErrorZ" << ','
        << "penalty1" << ',' << "penalty2" << ',' << "penalty3" << ','
        << "penalty4" << ',' << "penalty5" << ',' << "penalty6" << ','
        << "transitionFactorL" << ',' << "transitionFactorP" << ',' << "transitionFactorC" << ','
        << "obstacleCentre1" << ',' << "obstacleCentre2" << ',' << "obstacleCentre3" << ','
        << "radius" << ','
        << "zX" << ',' << "zY" << ',' << "zZ" << ','
        << "zdX" << ',' << "zdY" << ',' << "zdZ" << ','
        << "zDifferenceX" << ',' << "zDifferenceY" << ',' << "zDifferenceZ" << ','
        << "zdDifferenceX" << ',' << "zdDifferenceY" << ',' << "zdDifferenceZ" << ','
        << "measuredWallTimeIK" << ',' << "measuredCPUTimeIK" << '\n';

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
        if (time[i] == 1.0) {
            if (!jointLimits->enable(true)) logger->error("Error during enabling");
        }
        if (time[i] == 5.0) {
            if (!jointLimits->enable(false)) logger->error("Error during disabling");
        }
        if (time[i] == 3.0) {
            if (!desiredJointPositions->enable(true)) logger->error("Error during enabling");
        }
        if (time[i] == 12.0) {
            if (!desiredJointPositions->enable(false)) logger->error("Error during disabling");
        }
        if (time[i] == 0.0) {
            if (!collisionAvoidanceSphere->enable(true)) logger->error("Error during enabling");
        }

        /**
         * 13. Creation of the trajectory
         * ===============================
         */
        zDes[i].getPosition()(0) = xTrajectoryBuilder->evaluate(time[i], 0).value();
        zDes[i].getPosition()(1) = yTrajectoryBuilder->evaluate(time[i], 0).value();
        zDes[i].getPosition()(2) = zTrajectoryBuilder->evaluate(time[i], 0).value();
        zdDes[i][0] = xTrajectoryBuilder->evaluate(time[i], 1).value();
        zdDes[i][1] = yTrajectoryBuilder->evaluate(time[i], 1).value();
        zdDes[i][2] = zTrajectoryBuilder->evaluate(time[i], 1).value();
        zddDes[i][0] = xTrajectoryBuilder->evaluate(time[i], 2).value();
        zddDes[i][1] = yTrajectoryBuilder->evaluate(time[i], 2).value();
        zddDes[i][2] = zTrajectoryBuilder->evaluate(time[i], 2).value();

        /**
         * 14. Results of inverse kinematics
         * ==================================
         */
        // Start measuring time
        struct timespec beginWallTime, endWallTime, beginCPUTime, endCPUTime;
        clock_gettime(CLOCK_REALTIME, &beginWallTime);
        clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &beginCPUTime);
        // Function
        crf::control::inversekinematics::ResultsIK optCLIKExtendedResult(
            optCLIK->getExtendedResults(qAttr, zDes[i], zdDes[i]));
        // Stop measuring time and calculate the elapsed time
        clock_gettime(CLOCK_REALTIME, &endWallTime);
        clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &endCPUTime);
        double elapsedWallTime = (endWallTime.tv_sec - beginWallTime.tv_sec) + (endWallTime.tv_nsec - beginWallTime.tv_nsec)*1e-9;  // NOLINT
        double elapsedCPUTime = (endCPUTime.tv_sec - beginCPUTime.tv_sec) + (endCPUTime.tv_nsec - beginCPUTime.tv_nsec)*1e-9;  // NOLINT
        measuredWallTimeIK[i] = elapsedWallTime;
        measuredCPUTimeIK[i] = elapsedCPUTime;
        // Results of OptCLIK
        if (optCLIKExtendedResult.flag() ==
            crf::control::inversekinematics::ResultFlags::workspaceViolation) {
            resFlag[i] = 5;
        } else if (optCLIKExtendedResult.flag() ==
            crf::control::inversekinematics::ResultFlags::endEffectorToleranceViolation) {
            resFlag[i] = 3;
        } else if (optCLIKExtendedResult.flag() ==
            crf::control::inversekinematics::ResultFlags::lowManipulability) {
            resFlag[i] = 2;
        } else if (optCLIKExtendedResult.flag() ==
            crf::control::inversekinematics::ResultFlags::success) {
            resFlag[i] = 1;
        } else {
            resFlag[i] = 0;
        }
        q[i] = optCLIKExtendedResult.qResult();
        qd[i] = optCLIKExtendedResult.qdResult();
        zError[i] = optCLIKExtendedResult.zError();
        penalty[i] = optCLIKExtendedResult.penaltyGradients();

        /**
         * 15. Check velocity limits
         * ==========================
         */
        JointVelocities jointsMaxVelocity = configuration->getJointLimits().maxVelocity;
        for (unsigned int j = 0; j < jointsMaxVelocity.size(); j++) {
            if (qd[i][j] > jointsMaxVelocity[j] || qd[i][j] < -jointsMaxVelocity[j]) {
                resFlag[i] = 4;
                logger->error("The computed joint velocity [{}] is higher than the "
                    "corresponding maximum joint velocity of the robot", j);
            }
        }

        /**
         * 16. Transform the obtained joints position and velocities to task position and velocity
         * ========================================================================================
         */
        z[i] = forwardKinematics->getPose(q[i]).value();
        if (i > 0) {
            zd[i][0] = (z[i].getPosition()(0) - z[i-1].getPosition()(0)) /
                cycleTimeSeconds;
            zd[i][1] = (z[i].getPosition()(1) - z[i-1].getPosition()(1)) /
                cycleTimeSeconds;
            zd[i][2] = (z[i].getPosition()(2) - z[i-1].getPosition()(2)) /
                cycleTimeSeconds;
        } else {
            zd[i][0] = 0.0;
            zd[i][1] = 0.0;
            zd[i][2] = 0.0;
        }

        /**
         * 17. Compute the difference between the desired task position and velocity, and the
         *     obtained ones with the computed joints position and velocities
         * ===================================================================================
         */
        zDifference[i].resize(taskSpaceDim);
        zdDifference[i].resize(taskSpaceDim);
        for (int j = 0; j < taskSpaceDim; j++) {
            zDifference[i][j] = zDes[i].getPosition()(j) - z[i].getPosition()(j);
            zdDifference[i][j] = zdDes[i][j] - zd[i][j];
        }

        /**
         * 18. Information from objective functions
         * =========================================
         *
         * In all the objective functions there is an internal counter to mesure the transition
         * time.
         * Before calling the functions getGradient and getTransitionFactor outside the OptCLIK
         * and OptOLIK classes, it is necessary setting to false the function goToNextIteration,
         * and after, setting it to true to activate again the counter for the transition.
         */
        transitionFactor[i].resize(3);
        // Joint limits
        jointLimits->goToNextIteration(false);
        transitionFactor[i][0] = jointLimits->getTransitionFactor();
        jointLimits->goToNextIteration(true);
        // Desired joint position
        desiredJointPositions->goToNextIteration(false);
        transitionFactor[i][1] = desiredJointPositions->getTransitionFactor();
        desiredJointPositions->goToNextIteration(true);
        // Collision avoidance
        collisionAvoidanceSphere->goToNextIteration(false);
        transitionFactor[i][2] = collisionAvoidanceSphere->getTransitionFactor();
        collisionAvoidanceSphere->goToNextIteration(true);

        /**
         * 19. Store the computed information
         * ===================================
         */
        fout << std::setprecision(16);
        fout << time[i] << ','
            << zDes[i].getPosition()(0) << ','
            << zDes[i].getPosition()(1) << ','
            << zDes[i].getPosition()(2) << ','
            << zdDes[i][0] << ',' << zdDes[i][1] << ',' << zdDes[i][2] << ','
            << zddDes[i][0] << ',' << zddDes[i][1] << ',' << zddDes[i][2] << ','
            << q[i][0] << ',' << q[i][1] << ',' << q[i][2] << ','
            << q[i][3] << ',' << q[i][4] << ',' << q[i][5] << ','
            << qd[i][0] << ',' << qd[i][1] << ',' << qd[i][2] << ','
            << qd[i][3] << ',' << qd[i][4] << ',' << qd[i][5] << ','
            << resFlag[i] << ','
            << zError[i][0] << ',' << zError[i][1] << ',' << zError[i][2] << ','
            << penalty[i](0, 0) << ',' << penalty[i](1, 0) << ',' << penalty[i](2, 0) << ','
            << penalty[i](3, 0) << ',' << penalty[i](4, 0) << ',' << penalty[i](5, 0) << ','
            << transitionFactor[i][0] << ',' << transitionFactor[i][1] << ','
            << transitionFactor[i][2] << ','
            << center[0] << ',' << center[1] << ',' << center[2] << ',' << radius << ','
            << z[i].getPosition()(0) << ','
            << z[i].getPosition()(1) << ','
            << z[i].getPosition()(2) << ','
            << zd[i][0] << ',' << zd[i][1] << ',' << zd[i][2] << ','
            << zDifference[i][0] << ',' << zDifference[i][1] << ',' << zDifference[i][2] << ','
            << zdDifference[i][0] << ',' << zdDifference[i][1] << ',' << zdDifference[i][2] << ','
            << measuredWallTimeIK[i] << ',' << measuredCPUTimeIK[i] << '\n';
    }
    fout.close();
    std::cout << "qEnd = [" << q[cycles-1] << "]" << std::endl;
    std::cout << "zEnd (axis-angle) = ["
        << z[cycles-1].getPosition()(0) << ", "
        << z[cycles-1].getPosition()(1) << ", "
        << z[cycles-1].getPosition()(2) << "]"
        << std::endl << std::endl;
    std::cout << "Info of the sample stored in:" << std::endl << filename << std::endl;

    return 0;
}
