/* © Copyright CERN 2022. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Authors: Bartosz Sójka CERN BE/CEM/MRO 2023
 *
 *  ===============================================================================================================
 */

#include "KinematicChain/URDFKinematicChain/URDFKinematicChain.hpp"

int main(int argc, char** argv) {
    std::string testDirName = __FILE__;
    std::string pathToURDFs(testDirName.substr(0, testDirName.find("KinematicChain")));
    pathToURDFs += "KinematicChain/tests/config";
    std::string pathToURDF = pathToURDFs + "/KinematicChainCombinedTestingBasedOnSPS.urdf";

    std::string SPSRobot_toolName("kinova_end_effector");

    crf::math::kinematicchain::URDFKinematicChain URDFSampleRobot(
        pathToURDF, SPSRobot_toolName, "");

    pathToURDFs = __FILE__;
    pathToURDFs = pathToURDFs.substr(0, pathToURDFs.find("Math"));
    pathToURDFs += "Math/KinematicChain/samples/config";
    std::string pathToURDFCERNBot = pathToURDFs + "/URDFCombinedSample.urdf";
    std::string CERNBot_toolName("kinova_end_effector");

    pathToURDFs = __FILE__;
    pathToURDFs = pathToURDFs.substr(0, pathToURDFs.find("Math"));
    pathToURDFs += "Math/KinematicChain/samples/config";
    std::string pathToURDFToolExample = pathToURDFs + "/toolExample.urdf";
    auto CERNBot = std::make_shared<crf::math::kinematicchain::URDFKinematicChain>(
        pathToURDFCERNBot, CERNBot_toolName, pathToURDFToolExample);

    crf::utility::types::JointPositions jointPositions3(
        {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 3.14, 0.0});

    Eigen::Vector3d Ir_IE1(0.0, 0.0, 0.0);
    Eigen::Vector3d Ir_IE2(0.0, 0.0, 0.0);

    int chainSize_ = CERNBot->getChainSize();

    CERNBot->setJointPositions(jointPositions3);

    for (int i = 0; i < chainSize_; i++) {
        Ir_IE1 += CERNBot->getTranslation(crf::math::kinematicchain::Translations::IPJ, i);

        if (i == 0) {
            Ir_IE2 += CERNBot->getTranslation(crf::math::kinematicchain::Translations::PPJ, 0);
        } else {
            Ir_IE2 += CERNBot->getRotation(crf::math::kinematicchain::Rotations::IIJ, i - 1) *
                CERNBot->getTranslation(crf::math::kinematicchain::Translations::PPJ, i);
        }
    }
    std::cout << Ir_IE1 << std::endl << std::endl;

    std::cout << Ir_IE2 << std::endl << std::endl;

    std::cout << CERNBot->getTranslation(crf::math::kinematicchain::Translations::IIE, 0)
              << std::endl
              << std::endl;

    std::cout << Ir_IE1 - CERNBot->getTranslation(crf::math::kinematicchain::Translations::IIE, 0)
              << std::endl
              << std::endl;

    std::cout << Ir_IE2 - CERNBot->getTranslation(crf::math::kinematicchain::Translations::IIE, 0)
              << std::endl
              << std::endl;
    return 0;
}
