/* © Copyright CERN 2022. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Authors: Bartosz Sójka CERN BE/CEM/MRO 2023
 *
 *  ===============================================================================================================
 */

#include "Jacobian/KinChainJacobian/KinChainJacobian.hpp"
#include "KinematicChain/URDFKinematicChain/URDFKinematicChain.hpp"

using crf::math::jacobian::KinChainJacobian;

int main(int argc, char **argv) {
    std::string robotPathToURDF;
    std::string EndEffectorName;
    std::string toolPathToURDF;

    std::string pathToURDFs = __FILE__;
    pathToURDFs = pathToURDFs.substr(0, pathToURDFs.find("Jacobian"));
    pathToURDFs += "Jacobian/samples/config";

    robotPathToURDF = pathToURDFs + "/GEN3-6DOF_NO-VISION_URDF_ARM_V01.urdf";

    toolPathToURDF = pathToURDFs + "/toolExample.urdf";

    EndEffectorName = "end_effector_link";

    auto kinova3ToolKinChain = std::make_shared<crf::math::kinematicchain::URDFKinematicChain>(
        robotPathToURDF, EndEffectorName, toolPathToURDF);

    int chainSize = kinova3ToolKinChain->getChainSize();

    crf::utility::types::JointPositions jointPositions2(
        {-3.029177341404146,
         -0.457014640871583,
         1.242448406390738,
         -94.155944487570736,
         0.933728162671238,
         0.350321001356112});

    std::shared_ptr<KinChainJacobian> kinova3ToolJacobian =
        std::make_shared<KinChainJacobian>(kinova3ToolKinChain);

    Eigen::MatrixXd kinova3ToolJacobianMatrix = kinova3ToolJacobian->evaluate(jointPositions2);

    std::cout.precision(15);

    std::cout << kinova3ToolJacobianMatrix << std::endl << std::endl << std::endl;

    return 0;
}
