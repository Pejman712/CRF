/* © Copyright CERN 2022. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Laura Rodrigo Pérez CERN BE/CEM/MRO 2022
 *
 *  ================================================================================================================
*/

#include <math.h>

#include "InverseKinematics/OptCLIK/OptCLIK.hpp"
#include "Robot/RobotConfiguration.hpp"
#include "GeometricMethods/Sinusoid/Sinusoid.hpp"

using crf::control::inversekinematics::OptCLIK;
using crf::utility::types::TaskSpaceTangentDimension;

int main(int argc, char* argv[]) {
    crf::utility::logger::EventLogger logger(
        "BasicOptCLIKTrajectoryReduced2DTaskSpace2DOFARISArmRobotSample");
    /**
     * @brief
     * 
     * Robot with 2 DOF (q1, q2) (corresponding to the two first joints of ARIS robot) in a
     * 2 dimensional task space (y, z). Task spaces with less than 6 DOF are handled as reduced
     * dimension task spaces. Thus, the TaskPose variables have to be constructed in custom
     * mode.
     * The combination of robot and task space leads to a non-redundant system, thus no objective
     * functions will be applied in this example.
     * The trajectory of the end-effector will be a linear movement in y-direction.
     * The units of the task space variable are mm and rad for linear and angular movement,
     * respectively.
     */

    /**
     * 1. Set robot initial joints position and desired task position in the Y axis
     * =============================================================================
     */
    double q1Init, q2Init, yDes;
    if (argc == 5) {
        q1Init = std::stod(argv[1]);
        q2Init = std::stod(argv[2]);
        yDes = std::stod(argv[3]);
    } else {
        q1Init = 500.0;
        q2Init = 20.0;
        yDes = 0.0;
    }
    std::cout << "Inputs:" << std::endl << "q1Init = " << q1Init << "\t q2Init = " << q2Init
        << "\t yDes = " << yDes << std::endl << std::endl;

    /**
     * 2. Configuration file reading
     * ==============================
     */
    // Path to the config file
    std::string dirName = __FILE__;
    dirName = dirName.substr(0, dirName.find("Control"));
    dirName += "Actuators/Robot/config/ARISArm/";
    std::ifstream robotData(dirName + "1_3_GHz_ARIS_2Joints.json");
    nlohmann::json robotJSON = nlohmann::json::parse(robotData);

    /**
     * 3. Parameters of OptCLIK ctor
     * ==============================
     */
    JointPositions qInit({q1Init, q2Init});
    std::cout << "qInit = [" << qInit << "]" << std::endl;
    double cycleTimeSeconds = 0.01;
    std::chrono::microseconds cycleTime(static_cast<int>(cycleTimeSeconds * 1000000));
    std::shared_ptr<crf::actuators::robot::RobotConfiguration> configuration =
        std::make_shared<crf::actuators::robot::RobotConfiguration>(robotJSON);
    std::vector<double> diagW({1.0, 100.0});
    std::vector<std::shared_ptr<crf::control::inversekinematics::IKinematicObjectiveFunction>> objFunVector(0);  // NOLINT
    TaskPose tolerance(
        Eigen::Vector3d({0.0, 0.01, 0.01}), crf::math::rotation::CardanXYZ({0.0, 0.0, 0.0}));
    double K = 150;  // maximum K with cycle time 0.01 seconds: 200
    double kinManip0 = 0.001;
    double alpha0 = 0.001;
    // OptCLIK CTor
    std::unique_ptr<OptCLIK> optCLIK = std::make_unique<OptCLIK>(
        qInit, cycleTime, configuration, diagW, objFunVector, tolerance, K, kinManip0, alpha0);

    /**
     * 4. Declaration of the q attractor variable needed for the getResults OptCLIK function
     * ======================================================================================
     */
    JointPositions qAttr({0.0, 0.0});

    /**
     * 5. Forward Kinematics from qInit to obtain zInit (initial End-Effector Position in
     *    Task Space)
     * ===================================================================================
     */
    std::shared_ptr<crf::control::forwardkinematics::IForwardKinematics> forwardKinematics =
        configuration->getForwardKinematics();
    std::optional<TaskPose> zInit = forwardKinematics->getPose(qInit);
    if (!zInit) {
        std::cout << "Error obtaining position with forward kinematics" << std::endl;
        return -1;
    }
    std::cout << "zInit (axis-angle) = ["
        << zInit.value().getPosition()(1) << ", "
        << zInit.value().getPosition()(2) << "]"
        << std::endl << std::endl;

    /**
     * 6. Vertical trajectory of the EE based on the current qInit position
     * =====================================================================
     */
    double xInit = zInit.value().getPosition()(1);
    double yInit = zInit.value().getPosition()(2);
    double maxVelEE = 5;
    double xDes = xInit;
    std::shared_ptr<crf::math::geometricmethods::Sinusoid> trajectoryBuilder =
        std::make_shared<crf::math::geometricmethods::Sinusoid>(yInit, yDes, maxVelEE);
    std::optional<double> duration = trajectoryBuilder->getRange();
    if (!duration) {
        std::cout << "Duration: Failed to calculate the parameters" << std::endl;
        return -1;
    }

    /**
     * 7. Computation of the number of cycles needed based on the trajectory duration and the
     *    cycleTimeSeconds (time expended in one cycle)
     * =======================================================================================
     */
    int cycles = (duration.value() / cycleTimeSeconds) + 1;
    std::cout << "The number of cycles is " << cycles << " ("
        << (duration.value() / cycleTimeSeconds) + 1 << ")" << std::endl << std::endl;

    /**
     * 8. Declaration of the storage variables used in the loop
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
        crf::utility::types::JointPositions(2));
    std::vector<crf::utility::types::JointVelocities> qd(cycles,
        crf::utility::types::JointVelocities(2));
    std::vector<crf::utility::types::TaskPose> z(cycles,
        crf::utility::types::TaskPose());
    std::vector<crf::utility::types::TaskVelocity> zd(cycles,
        crf::utility::types::TaskVelocity());
    std::vector<unsigned int> resFlag(cycles, 0);
    std::vector<std::vector<double>> zDifference(cycles);
    std::vector<std::vector<double>> zdDifference(cycles);

    /**
     * 9. Creation of the storage file
     * ================================
     */
    std::string filename = __FILE__;
    filename = filename.substr(0, filename.find("OptCLIK/"));
    filename += "OptCLIK/BasicOptCLIKTrajectoryReduced2DTaskSpace2DOFARISArmRobot.csv";
    std::ofstream fout(filename);
    fout  << "time" << ','
        << "zDesX" << ',' << "zDesY" << ',' << "zdDesX" << ',' << "zdDesY" << ','
        << "zddDesX" << ',' << "zddDesY" << ','
        << "q1" << ',' << "q2" << ',' << "qd1" << ',' << "qd2" << ','
        << "flag"
        << "zX" << ',' << "zY" << ',' << "zdX" << ',' << "zdY" << ','
        << "zDifferenceX" << ',' << "zDifferenceY" << ','
        << "zdDifferenceX" << ',' << "zdDifferenceY" << ',' << '\n';

    /**
     * 10. Loop start
     * ===============
     */
    for (int i = 0; i < cycles; i++) {
        time[i] = i * cycleTimeSeconds;

        /**
         * 11. Creation of the trajectory with custom Task Space
         * ======================================================
         */
        zDes[i].getPosition()(1) = xDes;
        zDes[i].getPosition()(2) = trajectoryBuilder->evaluate(time[i], 0).value();
        zdDes[i][1] = 0.0;
        zdDes[i][2] = trajectoryBuilder->evaluate(time[i], 1).value();
        zddDes[i][1] = 0.0;
        zddDes[i][2] = trajectoryBuilder->evaluate(time[i], 2).value();

        /**
         * 12. Results of inverse kinematics
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
         * 13. Transform the obtained joints position and velocities to task position and velocity
         * ========================================================================================
         */
        z[i] = forwardKinematics->getPose(q[i]).value();
        if (i > 0) {
            zd[i][1] = (z[i].getPosition()(1) -
                z[i-1].getPosition()(1)) / cycleTimeSeconds;
            zd[i][2] = (z[i].getPosition()(2) -
                z[i-1].getPosition()(2)) / cycleTimeSeconds;
        } else {
            zd[i][1] = 0.0;
            zd[i][2] = 0.0;
        }

        /**
         * 14. Compute the difference between the desired task position and velocity, and the
         *     obtained ones with the computed joints position and velocities
         * ===================================================================================
         */
        zDifference[i].resize(2);
        zdDifference[i].resize(2);
        for (int j = 1; j < 3; j++) {
            zDifference[i][j] = zDes[i].getPosition()(j) -
                z[i].getPosition()(j);
            zdDifference[i][j] = zdDes[i][j] - zd[i][j];
        }

        /**
         * 15. Store the computed information
         * ===================================
         */
        // fout << std::setprecision(16);
        fout << time[i] << ','
            << zDes[i].getPosition()(1) << ','
            << zDes[i].getPosition()(2) << ','
            << zdDes[i][1] << ',' << zdDes[i][2] << ','
            << zddDes[i][1] << ',' << zddDes[i][2] << ','
            << q[i][1] << ',' << q[i][2] << ',' << qd[i][1] << ',' << qd[i][2] << ','
            << resFlag[i] << ','
            << z[i].getPosition()(1) << ','
            << z[i].getPosition()(2) << ','
            << zd[i][1] << ',' << zd[i][2] << ','
            << zDifference[i][1] << ',' << zDifference[i][2] << ','
            << zdDifference[i][1] << ',' << zdDifference[i][2] << '\n';
    }
    fout.close();
    std::cout << "qEnd = [" << q[cycles-1] << "]" << std::endl;
    std::cout << "zEnd (axis-angle) = ["
        << z[cycles-1].getPosition()(1) << ", "
        << z[cycles-1].getPosition()(2) << "]"
        << std::endl << std::endl;
    std::cout << "Info of the sample stored in:" << std::endl << filename << std::endl;

    return 0;
}
