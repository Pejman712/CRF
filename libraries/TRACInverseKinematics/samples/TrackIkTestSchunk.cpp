/* © Copyright CERN 2018. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Zsolt Pasztori CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#include <TRACInverseKinematics/trac_ik.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>

#include <chrono>

double fRand(double min, double max) {
    double f = static_cast<double>(rand()) / RAND_MAX;  // NOLINT
    return min + f * (max - min);
}

int main(int argc, char* argv[]) {
    std::cout << "The test will generate 100000 joint configuration and try to solve their "
        "inverse kinematics" << std::endl;

    // SchunkArm DH parameters
    KDL::Chain chain;
    chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),
                                  KDL::Frame::DH(0.0,  -M_PI/2, 0.205, 0.0)));
    chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),
                                  KDL::Frame::DH(0.350, M_PI,   0.0, -M_PI/2)));
    chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),
                                  KDL::Frame::DH(0.0,  -M_PI/2, 0.0, -M_PI/2)));
    chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),
                                  KDL::Frame::DH(0.0,   M_PI/2, 0.305, 0.0)));
    chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),
                                  KDL::Frame::DH(0.0,  -M_PI/2, 0.0, 0.0)));
    chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),
                                  KDL::Frame::DH(0.0,   0.0,  0.075, 0.0)));

    KDL::JntArray ll(6);
    KDL::JntArray ul(6);
    for (int i = 0; i < 6; ++i) {
        ll(i) = - (170.0 / 180) * M_PI;
        ul(i) = (170.0 / 180) * M_PI;
    }

    // Create list of random joint configurations
    int num_samples = 100000;
    std::vector<KDL::JntArray> JointList;
    KDL::JntArray q(chain.getNrOfJoints());
    for (uint i = 0; i < num_samples; i++) {
        for (uint j = 0; j < ll.data.size(); j++) {
            q(j) = fRand(ll(j), ul(j));
        }
        JointList.push_back(q);
    }

    // Track IK test
    std::cout << std::endl;
    std::cout << "Starting Trac_ik solver" << std::endl;
    KDL::ChainFkSolverPos_recursive fksolver(chain);
    TRAC_IK::TRAC_IK solver(chain, ll, ul, 0.005, 1e-5, TRAC_IK::SolveType::Speed);
    bool valid = solver.getKDLChain(chain);
    KDL::JntArray q_out(chain.getNrOfJoints());
    float succes = 0;

    auto timeNow = std::chrono::system_clock::now();
    for (int k = 1; k < num_samples; ++k) {
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
    std::cout << "Average solve time : " << duration.count() / num_samples << " [microsec]"
        << std::endl;
    std::cout << "Success rate       : " << succes / num_samples * 100 << " [%] " << std::endl;

    // KDL Newton Rhapson performance
    std::cout << std::endl;
    std::cout << "Starting KDL Levenberg-Marquardt solver" << std::endl;
    KDL::ChainIkSolverPos_LMA solver2(chain, 1E-5, 1000);
    succes = 0;
    float falsePositives = 0;
    timeNow = std::chrono::system_clock::now();
    for (int k = 1; k < num_samples; ++k) {
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
    std::cout << "Average solve time : " << duration2.count() / num_samples << " [microsec]"
        << std::endl;
    std::cout << "False positive rate "
        << falsePositives / num_samples * 100 << " [%] " << std::endl;
    std::cout << "Actual success rate : "
        << succes / num_samples * 100 << " [%] " << std::endl;

    // KDL Newton Rhapson joint limits performance
    std::cout << std::endl;
    std::cout << "Starting KDL Newton Rhapson with joint limits solver" << std::endl;
    KDL::ChainFkSolverPos_recursive fk_solver(chain);  // Forward kin. solver
    KDL::ChainIkSolverVel_pinv vik_solver(chain);  // PseudoInverse vel solver
    KDL::ChainIkSolverPos_NR_JL kdl_solver(chain, ll, ul, fk_solver, vik_solver, 150, 1e-4);
    succes = 0;
    timeNow = std::chrono::system_clock::now();
    for (int k = 1; k < num_samples; ++k) {
        KDL::Frame currentTaskPose{};
        fksolver.JntToCart(JointList[k], currentTaskPose);
        int result = kdl_solver.CartToJnt(JointList[k-1], currentTaskPose, q_out);
        bool insideLimits = true;
//        for (int i = 0; i < 6; ++i) {
//            if ((q_out(i) > ul(i)) || (q_out(i) < ll(i))) {
//                insideLimits = false;
//                break;
//            }
//        }
        if (result >= 0) {  // && insideLimits) {
            succes++;
        }
        // For tracking performance
        if (k % (num_samples /10) == 0) {
            std::cout << " Percentage : " << k / (num_samples /100) << std::endl;
        }
    }
    std::chrono::duration<double, std::micro> duration3 =
        std::chrono::system_clock::now() - timeNow;
    std::cout << "Average solve time : " << duration3.count() / num_samples << " [microsec]"
        << std::endl;
    std::cout << "Success rate       : " << succes / num_samples * 100 << " [%] " << std::endl;

    return 0;
}
