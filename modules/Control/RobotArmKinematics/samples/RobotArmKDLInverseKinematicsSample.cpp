/* © Copyright CERN 2018. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Zsolt Pasztori CERN EN/SMM/MRO 2018
 *
 *  ==================================================================================================
 */

// This file was created to test the inverse kinematics solver on different robot configurations
// You need to provide the path to robot configuration file, otherwise it will run on the default

#include "RobotArm/RobotArmConfiguration.hpp"

#include <TRACInverseKinematics/trac_ik.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>

#include <chrono>
#include <memory>
#include <string>
#include <vector>

using crf::actuators::robotarm::RobotArmConfiguration;
using crf::actuators::robotarm::JointLimits;
using crf::actuators::robotarm::DHParameter;

double fRand(double min, double max) {
    double f = (double)rand() / RAND_MAX;  // NOLINT
    return min + f * (max - min);
}

int main(int argc, char* argv[]) {
    // Get the config file
    std::string robotConfigFileName = "../modules/Robots/KinovaArm/config/KinovaJaco2CW.json";
    std::shared_ptr<RobotArmConfiguration> config = std::make_shared<RobotArmConfiguration>();
    if (argc > 1) {
        robotConfigFileName = argv[1];
    }
    std::ifstream robotData(robotConfigFileName);
    nlohmann::json robotJSON = nlohmann::json::parse(robotData);
    if (!config->parse(robotJSON)) {
        return -1;
    }

    // Set up the kinematic chain
    KDL::Chain chain;
    KDL::JntArray ll(6);
    KDL::JntArray ul(6);
    std::vector<DHParameter> chainVector = config->getKinematicChain();
    std::vector<JointLimits> jointLimitsVector = config->getJointsConfiguration();
    int numberOfJoints = config->getNumberOfJoints();
    for (int i=0; i< numberOfJoints; i++) {
        chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), KDL::Frame::DH(
            chainVector[i].a, chainVector[i].alpha, chainVector[i].d, chainVector[i].theta)));
        ll(i) = jointLimitsVector[i].minimumPosition;
        ul(i) = jointLimitsVector[i].maximumPosition;
    }

    std::cout << "The test will generate 10000 joint configuration"
        " and try to solve their inverse kinematics" << std::endl;

    // Create list of random joint configurations
    unsigned int num_samples = 10000;
    std::vector<KDL::JntArray> JointList;
    KDL::JntArray q(chain.getNrOfJoints());
    for (unsigned int i = 0; i < num_samples; i++) {
        for (unsigned int j = 0; j < ll.data.size(); j++) {
            q(j) = fRand(ll(j), ul(j));
        }
        JointList.push_back(q);
    }

    // Track IK test
    std::cout << std::endl;
    std::cout << "Starting Trac_ik solver" << std::endl;
    KDL::ChainFkSolverPos_recursive fksolver(chain);
    TRAC_IK::TRAC_IK solver(chain, ll, ul, 0.005, 1e-5, TRAC_IK::SolveType::Speed);
    solver.getKDLChain(chain);
    KDL::JntArray q_out(chain.getNrOfJoints());
    float succes = 0;

    auto timeNow = std::chrono::system_clock::now();
    for (unsigned int k = 1; k < num_samples; ++k) {
        KDL::Frame currentTaskPose{};
        fksolver.JntToCart(JointList[k], currentTaskPose);
        int result = solver.CartToJnt(JointList[k-1], currentTaskPose, q_out);
        bool insideLimits = true;
        for (int i = 0; i < 6; ++i) {
            if ((q_out(i) > ul(i)) || (q_out(i) < ll(i))) {
                insideLimits = false;
                break;
            }
        }
        if (result >= 0 && insideLimits) {
            succes++;
        }
        // For tracking performance
        if (k % (num_samples /10) == 0) {
            std::cout << " Percentage : " << k / (num_samples /100) << std::endl;
        }
    }
    std::chrono::duration<double, std::micro> duration = std::chrono::system_clock::now() - timeNow;
    std::cout << "Average solve time : " << duration.count() / num_samples
                << " [microsec]" << std::endl;
    std::cout << "Success rate       : " << succes / num_samples * 100 << " [%] " << std::endl;

    // KDL Newton Rhapson joint limits performance
    std::cout << std::endl;
    std::cout << "Starting KDL Newton Rhapson with joint limits solver" << std::endl;
    KDL::ChainFkSolverPos_recursive fk_solver(chain);  // Forward kin. solver
    KDL::ChainIkSolverVel_pinv vik_solver(chain);  // PseudoInverse vel solver
    KDL::ChainIkSolverPos_NR_JL kdl_solver(chain, ll, ul, fk_solver, vik_solver, 150, 1e-4);
    succes = 0;
    timeNow = std::chrono::system_clock::now();
    for (unsigned int k = 1; k < num_samples; ++k) {
        KDL::Frame currentTaskPose{};
        fksolver.JntToCart(JointList[k], currentTaskPose);
        int result = kdl_solver.CartToJnt(JointList[k-1], currentTaskPose, q_out);
        if (result >= 0) {
            succes++;
        }
        // For tracking performance
        if (k % (num_samples /10) == 0) {
            std::cout << " Percentage : " << k / (num_samples /100) << std::endl;
        }
    }
    std::chrono::duration<double, std::micro> duration3 =
            std::chrono::system_clock::now() - timeNow;
    std::cout << "Average solve time : "
        << duration3.count() / num_samples << " [microsec]"<< std::endl;
    std::cout << "Success rate       : "
        << succes / num_samples * 100 << " [%] " << std::endl;

    // KDL Newton Rhapson performance
    std::cout << std::endl;
    std::cout << "Starting KDL Levenberg-Marquardt solver" << std::endl;
    KDL::ChainIkSolverPos_LMA solver2(chain, 1E-5, 1000);
    succes = 0;
    float falsePositives = 0;
    timeNow = std::chrono::system_clock::now();
    for (unsigned int k = 1; k < num_samples; ++k) {
        KDL::Frame currentTaskPose{};
        fksolver.JntToCart(JointList[k], currentTaskPose);
        int result = solver2.CartToJnt(JointList[k-1], currentTaskPose, q_out);
        bool insideLimits = true;
        for (int i = 0; i < 6; ++i) {
            if ((q_out(i) > ul(i)) || (q_out(i) < ll(i))) {
                insideLimits = false;
                break;
            }
        }
        if (result >= 0 && insideLimits) {
            succes++;
        }
        if (result >= 0 && !insideLimits) {
            falsePositives++;
        }
        // For tracking performance
        if (k % (num_samples /10) == 0) {
            std::cout << " Percentage : " << k / (num_samples /100) << std::endl;
        }
    }
    std::chrono::duration<double, std::micro> duration2 =
            std::chrono::system_clock::now() - timeNow;
    std::cout << "Average solve time : "
        << duration2.count() / num_samples << " [microsec]"<< std::endl;
    std::cout << "Levenberg-Marquardt gives false positives,"
                 " sometimes it returns a solution which is not valid " << std::endl;
    std::cout << "False positive rate "
            << falsePositives / num_samples * 100 << " [%] " << std::endl;
    std::cout << "Actual success rate : "
              << succes / num_samples * 100 << " [%] " << std::endl;

    return 0;
}
