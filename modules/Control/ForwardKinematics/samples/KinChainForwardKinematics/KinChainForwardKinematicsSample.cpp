/* © Copyright CERN 2023. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Ante Marić CERN BE/CEM/MRO 2023
 *
 *  ================================================================================================================
*/

#include <Eigen/Dense>
#include <vector>

#include "ForwardKinematics/KinChainForwardKinematics/KinChainForwardKinematics.hpp"

using crf::math::kinematicchain::URDFKinematicChain;
using crf::control::forwardkinematics::KinChainForwardKinematics;
using crf::utility::types::JointPositions;
using crf::utility::types::TaskPose;

int main() {
    crf::utility::logger::EventLogger logger("KinChainForwardKinematicsSample");

    std::string dirName_ = __FILE__;
    std::string pathToURDFs_ = dirName_.substr(0, dirName_.find("Control"));
    pathToURDFs_ += "Actuators/Robot/urdf";

    std::string pathToURDF = pathToURDFs_ + "/KinematicChainCombinedTestingBasedOnSPS.urdf";
    std::string UR10eTool = "kinova_end_effector";

    auto UR10e = std::make_shared<URDFKinematicChain>(pathToURDF, UR10eTool, "");
    auto forwardKin = std::make_unique<KinChainForwardKinematics>(UR10e);

    JointPositions q = JointPositions({0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    std::cout << std::endl << "q: {" << q[0] << ", " << q[1] << ", " << q[2] << ", "
        << q[3] << ", " << q[4] << ", " << q[5] << "}" << std::endl;
    TaskPose z = forwardKin->getPose(q).value();
    std::cout << std::endl << "z: " << z << std::endl;
    crf::math::rotation::CardanXYZ cardanXYZ = z.getCardanXYZ();
    std::cout << "zCardanXYZ: " << cardanXYZ << std::endl;
    Eigen::Matrix4d zMatrix = z.getHomogeneousTransformationMatrix();
    std::cout << "zMatrix: {" <<
    zMatrix(0, 0) << ", " << zMatrix(0, 1) << ", " << zMatrix(0, 2)
        << ", " << zMatrix(0, 3) << ",\n\t  " << zMatrix(1, 0) << ", " << zMatrix(1, 1) << ", "
        << zMatrix(1, 2) << ", " << zMatrix(1, 3) << ",\n\t  " << zMatrix(2, 0) << ", "
        << zMatrix(2, 1) << ", " << zMatrix(2, 2) << ", " << zMatrix(2, 3) << ",\n\t  "
        << zMatrix(3, 0) << ", " << zMatrix(3, 1) << ", " << zMatrix(3, 2) << ", "
        << zMatrix(3, 3) << "}"
        << std::endl;

    return 0;
}
