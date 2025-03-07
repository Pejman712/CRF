/* © Copyright CERN 2022. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Laura Rodrigo Pérez CERN BE/CEM/MRO 2022
 *
 *  ================================================================================================================
*/

#include <math.h>

#include "GeometricMethods/DeBoor/DeBoor.hpp"
#include "InverseKinematics/OptCLIK/OptCLIK.hpp"
#include "InverseKinematics/JointLimits/JointLimits.hpp"
#include "Robot/RobotConfiguration.hpp"

using crf::control::inversekinematics::IClosedLoopInverseKinematics;
using crf::control::inversekinematics::OptCLIK;

int main(int argc, char* argv[]) {
    crf::utility::logger::EventLogger logger(
        "ExpertOptCLIKTrajectory6DTaskSpace6DOFUR10eRobotSample");
    /**
     * @brief
     * 
     * UR10e robot with 6 DOF (q1, ..., q6) in a task space with 6 DOF (element of SE3).
     * This sample demonstrates the usage of the OptCLIK algorithm to follow a full trajectory
     * defined in task space.
     * The trajectory of the end-effector will be a linear or/and angular movement depending on the
     * different trajectory parameters. The movement is based on NURB spline curves.
     * 
     * Since the combination of robot and task space leads to a non-redundant system, objective
     * functions shouldn't be applied. Neverdeless, in this sample you can turn on the joint limits
     * objective function and check that the desired trajectory always has priority. In the moment
     * the objective function interacts, the systems starts becoming unestable.
     * 
     * 
     * @param sample
     *        Sample 1: Linear round-trip trayectory diagonal in axis Y and Z.
     *        Sample 2: Diagonal in axis Y and Z linear and orientation round-trip trayectory.
     * 
     * @param withObjFun
     *        "true" : The objective function joint limits is used in the sample.
     *        "false" : The objective function joint limits is NOT used in the sample.
     * 
     * @param reactionLevel
     *        Level 1: No interaction.
     *        Level 2: No interaction in sample 1. In sample 2, it causes some oscilations.
     *        Level 3: In sample 1, it causes some oscilations. Sample 1 becomes unstable.
     */

    int sample = 1;
    std::string withObjFun = "false";
    int reactionLevel = 1;
    if (argc >= 2) {
        sample = std::stoi(argv[1]);
    }
    if (argc >= 3) {
        withObjFun = argv[2];
    }
    if (argc == 4) {
        reactionLevel = std::stoi(argv[3]);
    }
    if (argc > 4) {
        std::cout << "Too many arguments." << std::endl;
        return -1;
    }

    /**
     * 1. Set the desired displacement in the end-effector from the starting position.
     *    Trajectory parameters: In meters for linear movement (X, Y and Z) and in radians for
     *    orientation movements.
     * ========================================================================================
     */
    double movementX, movementY, movementZ, movementAngle;
    if (sample == 1) {
        std::cout << "Sample 1: Linear round-trip trayectory diagonal in axis Y and Z."
            << std::endl;
        movementX = 0.0;
        movementY = 0.1;
        movementZ = 0.1;
        movementAngle = 0.0;
    } else if (sample == 2) {
        std::cout << "Sample 2: Diagonal in axis Y and Z linear and orientation round-trip "
            "trayectory." << std::endl;
        movementX = 0.0;
        movementY = 0.1;
        movementZ = 0.1;
        movementAngle = M_PI*0.5;
    } else {
        std::cout << "Sample number " << sample << " doesn't exist." << std::endl;
        return -1;
    }

    /**
     * 2. Configuration file reading
     * ==============================
     */
    // Path to the config file
    std::string dirName = __FILE__;
    dirName = dirName.substr(0, dirName.find("InverseKinematics"));
    dirName += "ForwardKinematics/samples/MathExprForwardKinematics/";
    std::ifstream robotData(dirName + "UR10eSimFKLengthsJacobian.json");
    nlohmann::json robotJSON = nlohmann::json::parse(robotData);

    /**
     * 3. Objective functions
     * ======================
     */
    double rangeSinusoid = 2.0;
    double cycleTimeSeconds = 0.002;
    double c = 0.1, p = 10.2;
    if (withObjFun == "false") {
        std::cout << "\t  The objective function joint limits will be disabled during all the "
            "trajectory." << std::endl << std::endl;
    } else if (withObjFun == "true") {
        std::cout << "\t  The objective function joint limits will be enabled in some moment of "
            "the trajectory with reaction level: " << reactionLevel << std::endl << std::endl;
        if (reactionLevel == 1) {
            c = 0.1;
            p = 10.2;
        } else if (reactionLevel == 2) {
            c = 5.0;
            p = 0.6;
        } else if (reactionLevel == 3) {
            c = 50.0;
            p = 60;
        } else {
            std::cout << "Reaction level " << reactionLevel << " doesn't exist." << std::endl;
            return -1;
        }
    } else {
        std::cout << std::endl << "Wrong input for withObjFun input argument of the sample"
            << std::endl;
        return -1;
    }
    crf::utility::types::JointPositions minLimit({-2.0, -2.0, -2.0, -2.0, -2.0, -2.0});
    crf::utility::types::JointPositions maxLimit({2.0, 2.0, 2.0, 2.0, 2.0, 2.0});
    std::shared_ptr<crf::control::inversekinematics::IKinematicObjectiveFunction> jointLimits =
        std::make_unique<crf::control::inversekinematics::JointLimits>(
        rangeSinusoid, cycleTimeSeconds, c, p, minLimit, maxLimit);
    std::vector<std::shared_ptr<crf::control::inversekinematics::IKinematicObjectiveFunction>> objFunVector(1);  // NOLINT
    objFunVector[0] = jointLimits;

    /**
     * 4. Parameters of OptCLIK ctor
     * ==============================
     */
    TaskPose tolerance(Eigen::Vector3d({0.01, 0.01, 0.01}),
    crf::math::rotation::CardanXYZ({2*M_PI/180, 2*M_PI/180, 2*M_PI/180}));
    JointPositions qInitial;
    if (sample == 1) {
        qInitial = JointPositions({5.0*M_PI/180, -113.48*M_PI/180, 67.47*M_PI/180,
            -118.99*M_PI/180, 5.0*M_PI/180, 5.0*M_PI/180});
    } else if (sample == 2) {
        qInitial = JointPositions({25.0*M_PI/180, -152.0*M_PI/180, 118.0*M_PI/180,
            250.0*M_PI/180, -55.0*M_PI/180, 67.0*M_PI/180});
    }
    std::cout << "qInitial = [" << qInitial << "]" << std::endl;
    std::chrono::microseconds cycleTime(static_cast<int>(cycleTimeSeconds * 1000000));
    std::shared_ptr<crf::actuators::robot::RobotConfiguration> configuration =
        std::make_shared<crf::actuators::robot::RobotConfiguration>(robotJSON);
    std::vector<double> diagW({500, 500, 500, 0.05, 0.05, 0.05});
    double K = 500;
    double kinManip0 = 0.00017;
    double alpha0 = 0.001;
    // OptCLIK ctor
    clock_t t;
    t = clock();
    std::unique_ptr<IClosedLoopInverseKinematics> optCLIK = std::make_unique<OptCLIK>(
        qInitial, cycleTime, configuration, diagW, objFunVector, tolerance, K, kinManip0, alpha0);
    t = clock() - t;
    double dT = static_cast<double>(t)/CLOCKS_PER_SEC;
    std::cout << "OptCLIK ctor took " << dT << " seconds." << std::endl;

    /**
     * 5. Declaration of the q attractor variable needed for the getResults OptCLIK function
     * ======================================================================================
     */
    JointPositions qAttr({std::nan(""), std::nan(""), std::nan(""),
        std::nan(""), std::nan(""), std::nan("")});

    /**
     * 6. Forward Kinematics from qInitial to obtain the zStart (initial End-Effector Position in
     *    Task Space)
     * ===========================================================================================
     */
    std::shared_ptr<crf::control::forwardkinematics::IForwardKinematics> forwardKinematics =
        configuration->getForwardKinematics();
    const TaskPose zStart = forwardKinematics->getPose(qInitial).value();
    std::cout << "zStart (axis-angle) = ["
        << zStart.getPosition()(0) << ", "
        << zStart.getPosition()(1) << ", "
        << zStart.getPosition()(2) << ", "
        << zStart.getAngleAxis().angle() << ", "
        << zStart.getAngleAxis().axis()(0) << ", "
        << zStart.getAngleAxis().axis()(1) << ", "
        << zStart.getAngleAxis().axis()(2) << "]"
        << std::endl << std::endl;

    /**
     * 7. Trajectory of the EE based on the current qInit position
     * ============================================================
     */
    double rangeTrajectory = 8.0;
    // Initialize the NURBS trajectory
    int degree(4);
    std::vector<double> knots({0.0, 0.0, 0.0, 0.0, 0.0, 0.0625, 0.125, 0.1875, 0.25, 0.3125, 0.375,
        0.4375, 0.5, 0.5625, 0.625, 0.6875, 0.75, 0.8125, 0.875, 0.9375, 1.0, 1.0, 1.0, 1.0, 1.0});
    std::vector<double> controlPoints({0.0, 0.0, 0.0, 0.0, 0.05650608, 0.236911941, 0.560672572,
        0.972236973, 1.357214323, 1.590212873, 1.590212873, 1.357214323, 0.972236973, 0.560672572,
        0.236911941, 0.05650608, 0.0, 0.0, 0.0, 0.0});
    // Evaluate NURBS Trajectory
    for (size_t i = 0; i < knots.size(); i++) knots[i] *= (rangeTrajectory);
    std::unique_ptr<crf::math::geometricmethods::DeBoor> spline =
        std::make_unique<crf::math::geometricmethods::DeBoor>(degree, knots, controlPoints);

    /**
     * 8. Computation of the number of cycles needed based on the trajectory duration and the
     *    cycleTimeSeconds (time expended in one cycle)
     * =======================================================================================
     */
    int cycles = (rangeTrajectory / cycleTimeSeconds) + 1;
    std::cout << "The number of cycles is " << cycles
        << " (" << (rangeTrajectory / cycleTimeSeconds) + 1 << ")" << std::endl << std::endl;

    /**
     * 9. Declaration of the storage variables used in the loop
     * =========================================================
     */
    std::vector<double> time(cycles, 0.0);
    std::vector<double> trajecX(cycles, 0.0), trajecY(cycles, 0.0),
        trajecZ(cycles, 0.0), trajecAngle(cycles, 0.0);
    std::vector<crf::utility::types::TaskPose> zDes(cycles);
    std::vector<crf::utility::types::TaskVelocity> zdDes(cycles);
    std::vector<crf::utility::types::JointPositions> q(cycles,
        crf::utility::types::JointPositions(6));
    std::vector<crf::utility::types::JointVelocities> qd(cycles,
        crf::utility::types::JointVelocities(6));
    std::vector<crf::utility::types::TaskPose> z(cycles);
    std::vector<std::array<double, 6>> zCardan(cycles);
    std::vector<std::array<double, 7>> zAxisAngle(cycles);
    std::vector<crf::utility::types::TaskVelocity> zd(cycles);
    std::vector<unsigned int> resFlag(cycles, 0);
    std::vector<double> measuredWallTime(cycles, 0.0);
    std::vector<double> measuredCPUTime(cycles, 0.0);
    std::vector<Eigen::MatrixXd> penalty(cycles, Eigen::MatrixXd());
    std::vector<double> transitionFactor(cycles, 0.0);
    std::vector<std::vector<double>> zDifference(cycles);
    std::vector<std::vector<double>> zdDifference(cycles);

    /**
     * 10. Creation of the storage file
     * =================================
     */
    std::string filename = __FILE__;
    filename = filename.substr(0, filename.find("OptCLIKAndObjectiveFunctions/"));
    filename += "OptCLIKAndObjectiveFunctions/ExpertOptCLIKTrajectory6DTaskSpace6DOFUR10eRobot.csv";
    std::ofstream fout(filename);
    fout  << "time" << ',' << "zDesX" << ',' << "zDesY" << ',' << "zDesZ" << ','
        << "zDesAxisX" << ',' << "zDesAxisY" << ',' << "zDesAxisZ" << ',' << "zDesAngle" << ','
        << "zdDesX" << ',' << "zdDesY" << ',' << "zdDesZ" << ','
        << "zdDesAlpha" << ',' << "zdDesBeta" << ',' << "zdDesGamma" << ','
        << "q1" << ',' << "q2" << ',' << "q3" << ','
        << "q4" << ',' << "q5" << ',' << "q6" << ','
        << "qd1" << ',' << "qd2" << ',' << "qd3" << ','
        << "qd4" << ',' << "qd5" << ',' << "qd6" << ','
        << "zX" << ',' << "zY" << ',' << "zZ" << ','
        << "zAxisX" << ',' << "zAxisY" << ',' << "zAxisZ" << ',' << "zAngle" << ','
        << "zdX" << ',' << "zdY" << ',' << "zdZ" << ','
        << "zdAlpha" << ',' << "zdBeta" << ',' << "zdGamma" << ','
        << "zDifferenceX" << ',' << "zDifferenceY" << ',' << "zDifferenceZ" << ','
        << "zDifferenceAxisX" << ',' << "zDifferenceAxisY" << ',' << "zDifferenceAxisZ" << ','
        << "zDifferenceAngle" << ','
        << "zdDifferenceX" << ',' << "zdDifferenceY" << ',' << "zdDifferenceZ" << ','
        << "zdDifferenceAlpha" << ',' << "zdDifferenceBeta" << ',' << "zdDifferenceGamma" << ','
        << "flag" << ',' << "measuredWallTime" << ',' << "measuredCPUTime" << ','
        << "penalty1" << ',' << "penalty2" << ',' << "penalty3" << ','
        << "penalty4" << ',' << "penalty5" << ',' << "penalty6" << ','
        << "transitionFactor" << '\n';

    /**
     * 11. Loop start
     * ===============
     */
    for (int i = 0; i < cycles; i++) {
        time[i] = i*cycleTimeSeconds;

        /**
         * 12. Enabling / Disabling of the objective functions
         * ====================================================
         */
        if (withObjFun == "true") {
            if (time[i] == 1.0) {
                if (!jointLimits->enable(true)) logger->error("Error during enabling");
            }
            if (time[i] == 5.0) {
                if (!jointLimits->enable(false)) logger->error("Error during disabling");
            }
        }

        /**
         * 13. Creation of the trajectory
         * ===============================
         */
        std::optional<double> splineTrajec = spline->evaluate(i*cycleTimeSeconds, 0);
        trajecX[i] = zStart.getPosition()(0) + splineTrajec.value() * (movementX/1.5708);
        trajecY[i] = zStart.getPosition()(1) + splineTrajec.value() * (movementY/1.5708);
        trajecZ[i] = zStart.getPosition()(2) + splineTrajec.value() * (movementZ/1.5708);
        trajecAngle[i] = zStart.getAngleAxis().angle() +
            splineTrajec.value() * (movementAngle/1.5708);
        zDes[i] = TaskPose({trajecX[i], trajecY[i], trajecZ[i]},
            Eigen::AngleAxisd(trajecAngle[i], Eigen::Vector3d({
                zStart.getAngleAxis().axis()(0),
                zStart.getAngleAxis().axis()(1),
                zStart.getAngleAxis().axis()(2)})));
        if (i > 0) {
            zdDes[i] = TaskVelocity(crf::math::distancemeasures::byCardanXYZ(
                zDes[i-1], zDes[i]) / cycleTimeSeconds);
        }
        if (i == 1) {
            zdDes[0] = zdDes[1];
        }

        /**
         * 14. Results of inverse kinematics
         * ==================================
         */
        // Start measuring time
        struct timespec beginWallTime, endWallTime, beginCPUTime, endCPUTime;
        clock_gettime(CLOCK_REALTIME, &beginWallTime);
        clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &beginCPUTime);
        // Function
        std::tuple<JointPositions, JointVelocities, JointAccelerations,
            crf::control::inversekinematics::ResultFlags> optCLIKResult(
            optCLIK->getResults(qAttr, zDes[i], zdDes[i]));
        // Stop measuring time and calculate the elapsed time
        clock_gettime(CLOCK_REALTIME, &endWallTime);
        clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &endCPUTime);
        double elapsedWallTime = (endWallTime.tv_sec - beginWallTime.tv_sec) + (endWallTime.tv_nsec - beginWallTime.tv_nsec)*1e-9;  // NOLINT
        double elapsedCPUTime = (endCPUTime.tv_sec - beginCPUTime.tv_sec) + (endCPUTime.tv_nsec - beginCPUTime.tv_nsec)*1e-9;  // NOLINT
        measuredWallTime[i] = elapsedWallTime;
        measuredCPUTime[i] = elapsedCPUTime;
        // Results of OptCLIK
        if (std::get<3>(optCLIKResult) ==
            crf::control::inversekinematics::ResultFlags::workspaceViolation) {
            resFlag[i] = 3;
        } else if (std::get<3>(optCLIKResult) ==
            crf::control::inversekinematics::ResultFlags::endEffectorToleranceViolation) {
            resFlag[i] = 2;
        } else if (std::get<3>(optCLIKResult) ==
            crf::control::inversekinematics::ResultFlags::lowManipulability) {
            resFlag[i] = 1;
        } else if (std::get<3>(optCLIKResult) ==
            crf::control::inversekinematics::ResultFlags::success) {
            resFlag[i] = 0;
        } else {
            resFlag[i] = 5;
        }
        q[i] = std::get<0>(optCLIKResult);
        qd[i] = std::get<1>(optCLIKResult);

        /**
         * 15. Check velocity limits
         * ==========================
         */
        JointVelocities jointsMaxVelocity = configuration->getJointLimits().maxVelocity;
        for (unsigned int j = 0; j < jointsMaxVelocity.size(); j++) {
            if (qd[i][j] > jointsMaxVelocity[j] || qd[i][j] < -jointsMaxVelocity[j]) {
                resFlag[i] = 4;
                logger->error("At least, one of the computed joint velocity is higher than the "
                    "corresponding maximum joint velocity of the robot");
            }
        }

        /**
         * 16. Transform the obtained joints position and velocities to task position and velocity
         * ========================================================================================
         */
        z[i] = forwardKinematics->getPose(q[i]).value();
        if (i > 0) {
            zd[i] = TaskVelocity(crf::math::distancemeasures::byCardanXYZ(
                z[i-1], z[i]) / cycleTimeSeconds);
        } else {
            zd[i] = TaskVelocity();
        }

        /**
         * 17. Compute the difference between the desired task position and velocity, and the
         *     obtained ones with the computed joints position and velocities
         * ===================================================================================
         */
        zDifference[i].resize(7);
        zdDifference[i].resize(6);
        for (size_t j = 0; j < 3; j++) {
            zAxisAngle[i][j] = z[i].getPosition()(j);
        }
        for (size_t j = 0; j < 3; j++) {
            zAxisAngle[i][j+3] = z[i].getAngleAxis().axis()(j);
        }
        zAxisAngle[i][6] = z[i].getAngleAxis().angle();
        for (size_t j = 0; j < 3; j++) {
            zDifference[i][j] =
                zDes[i].getPosition()(j) -
                zAxisAngle[i][j];
            zdDifference[i][j] = zdDes[i][j] - zd[i][j];
        }
        for (size_t j = 0; j < 3; j++) {
            zDifference[i][j + 3] =
                zDes[i].getAngleAxis().axis()(j) -
                zAxisAngle[i][j + 3];
            zdDifference[i][j + 3] = zdDes[i][j + 3] - zd[i][j + 3];
        }
        zDifference[i][6] =
            zDes[i].getAngleAxis().angle() -
            zAxisAngle[i][6];

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
        // Joint limits
        jointLimits->goToNextIteration(false);
        penalty[i] = jointLimits->getGradient(q[i]);
        transitionFactor[i] = jointLimits->getTransitionFactor();
        jointLimits->goToNextIteration(true);

        /**
         * 19. Store the computed information
         * ===================================
         */
        // fout << std::setprecision(16);
        fout << time[i] << ','
            << zDes[i].getPosition()(0) << ','
            << zDes[i].getPosition()(1) << ','
            << zDes[i].getPosition()(2) << ','
            << zDes[i].getAngleAxis().axis()(0) << ','
            << zDes[i].getAngleAxis().axis()(1) << ','
            << zDes[i].getAngleAxis().axis()(2) << ','
            << zDes[i].getAngleAxis().angle() << ','
            << zdDes[i][0] << ',' << zdDes[i][1] << ',' << zdDes[i][2] << ','
            << zdDes[i][3] << ',' << zdDes[i][4] << ',' << zdDes[i][5] << ','
            << q[i][0] << ',' << q[i][1] << ',' << q[i][2] << ','
            << q[i][3] << ',' << q[i][4] << ',' << q[i][5] << ','
            << qd[i][0] << ',' << qd[i][1] << ',' << qd[i][2] << ','
            << qd[i][3] << ',' << qd[i][4] << ',' << qd[i][5] << ','
            << zAxisAngle[i][0] << ',' << zAxisAngle[i][1] << ',' << zAxisAngle[i][2] << ','
            << zAxisAngle[i][3] << ',' << zAxisAngle[i][4] << ',' << zAxisAngle[i][5] << ','
            << zAxisAngle[i][6] << ','
            << zd[i][0] << ',' << zd[i][1] << ',' << zd[i][2] << ','
            << zd[i][3] << ',' << zd[i][4] << ',' << zd[i][5] << ','
            << zDifference[i][0] << ',' << zDifference[i][1] << ',' << zDifference[i][2] << ','
            << zDifference[i][3] << ',' << zDifference[i][4] << ',' << zDifference[i][5] << ','
            << zDifference[i][6] << ','
            << zdDifference[i][0] << ',' << zdDifference[i][1] << ',' << zdDifference[i][2] << ','
            << zdDifference[i][3] << ',' << zdDifference[i][4] << ',' << zdDifference[i][5] << ','
            << resFlag[i] << ',' <<  measuredWallTime[i] << ',' << measuredCPUTime[i] << ','
            << penalty[i](0, 0) << ',' << penalty[i](1, 0) << ',' << penalty[i](2, 0) << ','
            << penalty[i](3, 0) << ',' << penalty[i](4, 0) << ',' << penalty[i](5, 0) << ','
            << transitionFactor[i] << '\n';
    }
    fout.close();
    std::cout << "qEnd = [" << q[cycles-1] << "]" << std::endl;
    std::cout << "zEnd (axis-angle) = [" << zAxisAngle[cycles-1][0] << ", "
        << zAxisAngle[cycles-1][1] << ", " << zAxisAngle[cycles-1][2] << ", "
        << zAxisAngle[cycles-1][3] << ", " << zAxisAngle[cycles-1][4] << ", "
        << zAxisAngle[cycles-1][5] << ", " << zAxisAngle[cycles-1][6] << "]"
        << std::endl << std::endl;
    std::cout << "Info of the sample stored in:" << std::endl << filename << std::endl;

    return 0;
}
