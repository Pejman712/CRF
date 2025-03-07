/* © Copyright CERN 2022. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Laura Rodrigo Pérez CERN BE/CEM/MRO 2022
 *
 *  ================================================================================================================
*/

#include <fstream>

#include "InverseKinematics/CollisionAvoidanceSphere/CollisionAvoidanceSphere.hpp"
#include "GeometricMethods/DeBoor/DeBoor.hpp"

using crf::control::inversekinematics::IKinematicObjectiveFunction;
using crf::control::inversekinematics::CollisionAvoidanceSphere;

int main(int argc, char* argv[]) {
    crf::utility::logger::EventLogger logger("CollisionAvoidanceSphereSample");

    std::string filename = __FILE__;
    filename = filename.substr(0, filename.find("CollisionAvoidanceSphereSample.cpp"));
    std::ofstream fout(filename + "collisionAvoidanceSphere.csv");
    fout  << "time" << ',' << "q(0)" << ',' << "q(1)" << ',' << "penalty(0)" << ',' << "penalty(1)"
        << ',' << "transitionFactor" << '\n';


    double rangeSinusoid = 2.0;
    double cycleTime = 0.002;
    int curveType = 1;  // Exponential
    double c = 0.1;
    double p = 1.0;
    int robot = 0;  // 2 DOF robot


    // set the obstacle
    Eigen::Vector3d center(0.2, 0.0, -1.0);
    double radius = 0.4;

    std::unique_ptr<IKinematicObjectiveFunction> collisionAvoidanceSphere =
        std::make_unique<CollisionAvoidanceSphere>(rangeSinusoid, cycleTime, curveType, c, p,
        center, radius, robot);

    double rangeTrajectory = 8.0;

    int timeNumber = (rangeTrajectory / cycleTime) + 1;
    std::cout << "The timeNumber is " << timeNumber
        << " (" << (rangeTrajectory / cycleTime) + 1 << ")" << std::endl;

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


    std::cout << " Time | q | Penalty" << std::endl;
    std::vector<double> timeVector(timeNumber, 0.0);
    std::vector<crf::utility::types::JointPositions> qVector(timeNumber,
        crf::utility::types::JointPositions(2));
    std::vector<Eigen::MatrixXd> penaltyVector(timeNumber, Eigen::MatrixXd());
    std::vector<double> transitionFactorVector(timeNumber, 0.0);


    for (int i = 0; i < timeNumber; i++) {
        timeVector[i] = i*cycleTime;

        if (timeVector[i] == 1.0) {
            if (!collisionAvoidanceSphere->enable(true)) logger->error("Error during enabling");
        }
        if (timeVector[i] == 5.0) {
            if (!collisionAvoidanceSphere->enable(false)) logger->error("Error during disabling");
        }

        qVector[i] = crf::utility::types::JointPositions({spline->evaluate(i*cycleTime, 0).value(),
            -1 * spline->evaluate(i*cycleTime, 0).value()});
        penaltyVector[i] = collisionAvoidanceSphere->getGradient(qVector[i]);
        collisionAvoidanceSphere->goToNextIteration(false);
        transitionFactorVector[i] = collisionAvoidanceSphere->getTransitionFactor();
        collisionAvoidanceSphere->goToNextIteration(true);

        std::cout << timeVector[i] << " | " << qVector[i] << " | "
            << penaltyVector[i](0, 0) << "  " << penaltyVector[i](1, 0) << std::endl;

        fout << timeVector[i] << ',' << qVector[i][0] << ',' << qVector[i][1] << ','
            << penaltyVector[i](0, 0) << ',' << penaltyVector[i](1, 0) << ','
            << transitionFactorVector[i] << '\n';
    }
    fout.close();



    crf::utility::types::JointPositions q({0.0, 0.0});
    if (!collisionAvoidanceSphere->enable(true)) logger->error("Error during enabling");
    Eigen::MatrixXd penalty = collisionAvoidanceSphere->getGradient(q);
    std::cout << std::endl << std::endl
        << "penalty = {" << penalty(0, 0) << ", " << penalty(1, 0) << "}" << std::endl << std::endl;

    std::unique_ptr<IKinematicObjectiveFunction> collisionAvoidanceSphere2 =
        std::make_unique<CollisionAvoidanceSphere>(rangeSinusoid, cycleTime, curveType, c, p,
        center, radius, robot);

    if (!collisionAvoidanceSphere2->setParam("c", "0.5")) {
        logger->error("Error setting c parameter");
    }

    if (!collisionAvoidanceSphere2->enable(true)) {
        logger->error("Error during enabling");
    }
    penalty = collisionAvoidanceSphere2->getGradient(q);
    std::cout << "penalty = {" << penalty(0, 0) << ", " << penalty(1, 0) << "}" << std::endl;

    return 0;
}
