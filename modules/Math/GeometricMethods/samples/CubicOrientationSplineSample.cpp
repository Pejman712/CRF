/* © Copyright CERN 2024. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Chelsea Davidson CERN BE/CEM/MRO 2024
 *
 *  ================================================================================================================
 */

#include "GeometricMethods/CubicOrientationSpline/CubicOrientationSpline.hpp"

using crf::math::geometricmethods::CubicOrientationSpline;

int main(int argc, char** argv) {
    // --------------  Example 1 - creating an interpolation between unit quaternions --------------
    std::vector<double> timeInstancesQuaternion = {0.0, 90.0, 180.0, 270.0, 360.0};

    std::vector<Orientation> orientationsQuaternion = {
        Orientation(
            Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0)),
        Orientation(
            Eigen::Quaterniond(0.707106781187, 0.000000000000, -0.707106781186, 0.000000000000)),
        Orientation(
            Eigen::Quaterniond(0.500000000000, -0.500000000000, -0.500000000000, 0.500000000000)),
        Orientation(
            Eigen::Quaterniond(0.707106781187, 0.000000000000, -0.000000000000, 0.707106781187)),
        Orientation(
            Eigen::Quaterniond(1.000000000000, 0.000000000000, -0.000000000000, 0.000000000000))};

    // Define initial and final angular velocities
    Eigen::Vector3d initialAngularVelocityQuaternion(0, 0.0174533, 0);
    Eigen::Vector3d finalAngularVelocityQuaternion(0, 0.0174533, 0);

    // Create the interpolation object
    CubicOrientationSpline quatInterp(orientationsQuaternion, initialAngularVelocityQuaternion,
        finalAngularVelocityQuaternion, timeInstancesQuaternion);

    // Find the quaternion, angular velocity, and angular acceleration at a specific time instance
    double t = 4.4;
    Eigen::Quaterniond quatEval = quatInterp.evaluateOrientation(t).value().getQuaternion();
    Eigen::Vector3d angVelEval = quatInterp.evaluate(t, 1).value();
    Eigen::Vector3d accEval = quatInterp.evaluate(t, 2).value();

    // Output evaluation results
    std::cout << "\nExample 1 - evaluation of a quaternion interpolation at time t = "
              << std::setprecision(3) << std::fixed << t << "s:" << std::endl;
    std::cout << "   Quaternion: " << std::setprecision(15) << std::fixed << quatEval.w() << "  "
              << quatEval.x() << "  " << quatEval.y() << "  " << quatEval.z() << std::endl;
    std::cout << "   Angular velocity: " << std::setprecision(15) << std::fixed << angVelEval.x()
              << "  " << angVelEval.y() << "  " << angVelEval.z() << std::endl;
    std::cout << "   Angular acceleration: " << std::setprecision(15) << std::fixed
              << accEval.x() << "  " << accEval.y() << "  " << accEval.z() << std::endl;

    // -------------- Example 2 - creating an interpolation between Axis
    //                Angle representations of orientations              --------------
    std::vector<double> timeInstancesAxAng = {0.0, 90.0, 180.0, 270.0};

    std::vector<Orientation> orientationsAxAng = {
        Orientation(Eigen::AngleAxisd(0.174533, Eigen::Vector3d(0.5, 0, 1).normalized())),
        Orientation(Eigen::AngleAxisd(0.523599, Eigen::Vector3d(0.25, 1, 0).normalized())),
        Orientation(Eigen::AngleAxisd(1.0472,   Eigen::Vector3d(0.3, 0, 1).normalized())),
        Orientation(Eigen::AngleAxisd(0.785398, Eigen::Vector3d(0.7, 0.2, 1).normalized()))};

    // Define initial and final angular velocities
    Eigen::Vector3d initialAngularVelocityAxAng(0, 0, -0.174533);
    Eigen::Vector3d finalAngularVelocityAxAng(0, 0.174533, 0);

    // Create the interpolation object
    CubicOrientationSpline axAngInterp(orientationsAxAng, initialAngularVelocityAxAng,
        finalAngularVelocityAxAng, timeInstancesAxAng);

    // Find Axis Angle at a specific time instance
    t = 198.3;
    Eigen::AngleAxisd axAngEval = axAngInterp.evaluateOrientation(t).value().getAngleAxis();
    Eigen::Vector3d ax = axAngEval.axis();
    double ang = axAngEval.angle();
    std::cout << "\nExample 2 - evaluation of an Axis Angle interpolation at time t = "
              << std::setprecision(3) << std::fixed << t << "s:" << std::endl;
    std::cout << "   Axis: " << std::setprecision(15) << std::fixed << ax.x() << "  " << ax.y()
              << "  " << ax.z() << std::endl;
    std::cout << "   Angle: " << std::setprecision(15) << std::fixed << ang << std::endl;

    // ---------- Example 3 - creating an interpolation between Cardan XYZ Angle representations of
    //            orientations and specifying convergence tolerance and max iterations    ----------
    std::vector<double> timeInstancesCardan = {0.0, 10.0, 20.0, 30.0};

    std::vector<Orientation> orientationsCardan = {
        Orientation(CardanXYZ({0, 0, 0.2})),
        Orientation(CardanXYZ({0.3, 0, 0.5})),
        Orientation(CardanXYZ({0, 0.3, 0})),
        Orientation(CardanXYZ({1.0, 0, 0.8}))};

    // Define initial and final angular velocities
    Eigen::Vector3d initialAngularVelocityCardan(0, 0, -0.7);
    Eigen::Vector3d finalAngularVelocityCardan(0, 0, 0);

    // Create interpolation object, specifying convergence tolerance and max number of iterations
    double tol = 1e-12;
    std::size_t maxIter = 50;
    CubicOrientationSpline cardanInterp(orientationsCardan, initialAngularVelocityCardan,
        finalAngularVelocityCardan, timeInstancesCardan, tol, maxIter);

    // Find Cardan angle at a specific time instance
    t = 15.5;
    CardanXYZ cardanEval = cardanInterp.evaluateOrientation(t).value().getCardanXYZ();
    std::cout << "\nExample 3 - evaluation of a Cardan XYZ angle interpolation at time t = "
              << std::setprecision(3) << std::fixed << t << "s:" << std::endl;
    std::cout << "   Cardan XYZ angle: " << std::setprecision(15) << std::fixed
              << cardanEval.rawArray()[0] << "  " << cardanEval.rawArray()[1] << "  "
              << cardanEval.rawArray()[2] << std::endl;

    // ----------   Example 4 - creating an interpolation between
    //              Euler ZXZ Angle representations of orientations     --------------
    std::vector<double> timeInstancesEuler = {0.0, 45.0, 90.0, 135.0};

    std::vector<Orientation> orientationsEuler = {
        Orientation(EulerZXZ({0, 0.03, 0})),
        Orientation(EulerZXZ({0.03, 0.28, 0.02})),
        Orientation(EulerZXZ({0.14, 0.14, 0.06})),
        Orientation(EulerZXZ({0.06, 0.28, 0.03}))};

    // Define initial and final angular velocities
    Eigen::Vector3d initialAngularVelocityEuler(0, 0, 0);
    Eigen::Vector3d finalAngularVelocityEuler(0, 0, 0);

    // Create the interpolation object
    CubicOrientationSpline eulerInterp(orientationsEuler, initialAngularVelocityEuler,
        finalAngularVelocityEuler, timeInstancesEuler);

    // Find Euler angle at a specific time instance
    t = 0.5;
    EulerZXZ eulerEval = eulerInterp.evaluateOrientation(t).value().getEulerZXZ();
    std::cout << "\nExample 4 - evaluation of a Euler ZXZ angle interpolation at time t = "
              << std::setprecision(3) << std::fixed << t << "s:" << std::endl;
    std::cout << "   Euler ZXZ angle: " << std::setprecision(15) << std::fixed
              << eulerEval.rawArray()[0] << "  " << eulerEval.rawArray()[1] << "  "
              << eulerEval.rawArray()[2] << std::endl;

    // Find the range
    double range = eulerInterp.getRange().value();
    std::cout << "   Range: " << range << std::endl;

    return 0;
}
