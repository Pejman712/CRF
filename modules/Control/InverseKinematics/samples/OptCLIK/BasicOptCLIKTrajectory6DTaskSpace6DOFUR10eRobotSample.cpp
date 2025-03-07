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
#include "Robot/RobotConfiguration.hpp"

using crf::control::inversekinematics::IClosedLoopInverseKinematics;
using crf::control::inversekinematics::OptCLIK;

int main(int argc, char* argv[]) {
    crf::utility::logger::EventLogger logger(
        "BasicOptCLIKTrajectory6DTaskSpace6DOFUR10eRobotSample");
    /**
     * @brief
     * 
     * UR10e robot with 6 DOF (q1, ..., q6) in a task space with 6 DOF (element of SE3).
     * This sample demonstrates the usage of the OptCLIK algorithm to follow a full trajectory
     * defined in task space.
     * The default trajectory of the end-effector will be a linear movement based on NURB spline
     * curves.
     * Since the combination of robot and task space leads to a non-redundant system, objective
     * functions will not be applied.
     */

    /**
     * 1. Set the desired displacement in the end-effector from the starting position.
     *    Trajectory parameters: In meters for linear movement (X, Y and Z) and in radians for
     *    orientation movements.
     * ========================================================================================
     */
    double movementX = 0.0;
    double movementY = 0.1;
    double movementZ = 0.1;
    double movementAngle = 0.0;

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
     * 3. Parameters of OptCLIK ctor
     * ==============================
     */
    JointPositions qInitial({5.0*M_PI/180, -113.48*M_PI/180, 67.47*M_PI/180,
        -118.99*M_PI/180, 5.0*M_PI/180, 5.0*M_PI/180});
    std::cout << "qInitial = [" << qInitial << "]" << std::endl;
    double cycleTimeSeconds = 0.002;
    std::chrono::microseconds cycleTime(static_cast<int>(cycleTimeSeconds * 1000000));
    std::shared_ptr<crf::actuators::robot::RobotConfiguration> configuration =
        std::make_shared<crf::actuators::robot::RobotConfiguration>(robotJSON);
    std::vector<double> diagW({500, 500, 500, 0.05, 0.05, 0.05});
    std::vector<std::shared_ptr<crf::control::inversekinematics::IKinematicObjectiveFunction>> objFunVector(0);  // NOLINT
    TaskPose tolerance(
        Eigen::Vector3d({0.01, 0.01, 0.01}),
        crf::math::rotation::CardanXYZ({2*M_PI/180, 2*M_PI/180, 2*M_PI/180}));
    double K = 500;
    double kinManip0 = 0.00017;
    double alpha0 = 0.001;
    // OptCLIK ctor
    std::unique_ptr<IClosedLoopInverseKinematics> optCLIK = std::make_unique<OptCLIK>(
        qInitial, cycleTime, configuration, diagW, objFunVector, tolerance, K, kinManip0, alpha0);

    /**
     * 4. Declaration of the q attractor variable needed for the getResults OptCLIK function
     * ======================================================================================
     */
    JointPositions qAttr({std::nan(""), std::nan(""), std::nan(""),
        std::nan(""), std::nan(""), std::nan("")});

    /**
     * 5. Forward Kinematics from qInitial to obtain the zStart (initial End-Effector Position in
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
     * 6. Trajectory of the EE based on the current qInit position
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
     * 7. Computation of the number of cycles needed based on the trajectory duration and the
     *    cycleTimeSeconds (time expended in one cycle)
     * =======================================================================================
     */
    int cycles = (rangeTrajectory / cycleTimeSeconds) + 1;
    std::cout << "The number of cycles is " << cycles
        << " (" << (rangeTrajectory / cycleTimeSeconds) + 1 << ")" << std::endl << std::endl;

    /**
     * 8. Declaration of the storage variables used in the loop
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
    std::vector<unsigned int> resFlag(cycles, 0);

    /**
     * 9. Creation of the storage file
     * ================================
     */
    std::string filename = __FILE__;
    filename = filename.substr(0, filename.find("OptCLIK/"));
    filename += "OptCLIK/BasicOptCLIKTrajectory6DTaskSpace6DOFUR10eRobot.csv";
    std::ofstream fout(filename);
    fout  << "time" << ',' << "zDesX" << ',' << "zDesY" << ',' << "zDesZ" << ','
        << "zDesAxisX" << ',' << "zDesAxisY" << ',' << "zDesAxisZ" << ',' << "zDesAngle" << ','
        << "zdDesX" << ',' << "zdDesY" << ',' << "zdDesZ" << ','
        << "zdDesAlpha" << ',' << "zdDesBeta" << ',' << "zdDesGamma" << ','
        << "q1" << ',' << "q2" << ',' << "q3" << ','
        << "q4" << ',' << "q5" << ',' << "q6" << ','
        << "qd1" << ',' << "qd2" << ',' << "qd3" << ','
        << "qd4" << ',' << "qd5" << ',' << "qd6" << ','
        << "flag" << '\n';

    /**
     * 10. Loop start
     * ===============
     */
    for (int i = 0; i < cycles; i++) {
        time[i] = i*cycleTimeSeconds;

        /**
         * 11. Creation of the trajectory
         * ===============================
         */
        std::optional<double> splineTrajec = spline->evaluate(i*cycleTimeSeconds, 0);
        Eigen::Vector3d zStartPosition = zStart.getPosition();
        trajecX[i] = zStartPosition(0) + splineTrajec.value() * (movementX/1.5708);
        trajecY[i] = zStartPosition(1) + splineTrajec.value() * (movementY/1.5708);
        trajecZ[i] = zStartPosition(2) + splineTrajec.value() * (movementZ/1.5708);
        Eigen::AngleAxisd zStartAngleAxis = zStart.getAngleAxis();
        trajecAngle[i] =
            zStartAngleAxis.angle() +
            splineTrajec.value() * (movementAngle/1.5708);
        Eigen::AngleAxisd zDesAngleAxis;
        zDesAngleAxis.angle() = trajecAngle[i];
        zDesAngleAxis.axis() = zStartAngleAxis.axis();
        zDes[i] = TaskPose({trajecX[i], trajecY[i], trajecZ[i]}, zDesAngleAxis);
        if (i > 0) {
            zdDes[i] = TaskVelocity(crf::math::distancemeasures::byCardanXYZ(
                zDes[i-1], zDes[i]) / cycleTimeSeconds);
        }
        if (i == 1) {
            zdDes[0] = zdDes[1];
        }

        /**
         * 12. Results of inverse kinematics
         * ==================================
         */
        // Constructor
        std::tuple<JointPositions, JointVelocities, JointAccelerations,
            crf::control::inversekinematics::ResultFlags> optCLIKResult(
            optCLIK->getResults(qAttr, zDes[i], zdDes[i]));
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
         * 13. Store the computed information
         * ===================================
         */
        // fout << std::setprecision(16);
        fout << time[i] << ','
            << zDes[i].getPosition()(0) << ','
            << zDes[i].getPosition()(1) << ','
            << zDes[i].getPosition()(2) << ','
            << zDes[i].getAngleAxis().angle() << ','
            << zDes[i].getAngleAxis().axis()(0) << ','
            << zDes[i].getAngleAxis().axis()(1) << ','
            << zDes[i].getAngleAxis().axis()(2) << ','
            << zdDes[i][0] << ',' << zdDes[i][1] << ',' << zdDes[i][2] << ','
            << zdDes[i][3] << ',' << zdDes[i][4] << ',' << zdDes[i][5] << ','
            << q[i][0] << ',' << q[i][1] << ',' << q[i][2] << ','
            << q[i][3] << ',' << q[i][4] << ',' << q[i][5] << ','
            << qd[i][0] << ',' << qd[i][1] << ',' << qd[i][2] << ','
            << qd[i][3] << ',' << qd[i][4] << ',' << qd[i][5] << ','
            << resFlag[i] << '\n';
    }
    fout.close();
    std::cout << "qEnd = [" << q[cycles-1] << "]" << std::endl << std::endl;
    std::cout << "Info of the sample stored in:" << std::endl << filename << std::endl;

    return 0;
}
