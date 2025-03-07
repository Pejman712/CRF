/* © Copyright CERN 2024. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Bartosz Sójka CERN BE/CEM/MRO 2024
 *
 *  ================================================================================================================
 */

#include "Types/JointTypes/JointTypes.hpp"

using crf::utility::types::VectorXd;
using crf::utility::types::JointPositions;
using crf::utility::types::JointVelocities;
using crf::utility::types::JointAccelerations;
using crf::utility::types::JointForceTorques;

int main() {
    double input = 1.7;
    double output = 0;
    /**
     * Sample std::vector<double> and Eigen::VectorXd from which joint types can be constructed.
    */
    Eigen::VectorXd eigenVector(7);
    eigenVector << 3.14, 1.7, 0.5, 2.15, 43.44, -0.15, 8.0;
    const std::vector<double> stdVector({3.14, 1.7, 0.5, 2.15, 43.44, -0.15, 8.0});

    /**
     * Default constructors.
     * Creates an empty object corresponding to no joints.
    */
    JointPositions jointPositions0;
    JointVelocities jointVelocities0;
    JointAccelerations jointAccelerations0;
    JointForceTorques jointForceTorques0;

    /**
     * Constructors from  Eigen::VectorXd, std::vector<double> and initialiser_list
    */
    JointPositions jointPositions1(eigenVector);
    JointVelocities jointVelocities1(eigenVector);
    JointAccelerations jointAccelerations1(eigenVector);
    JointForceTorques jointForceTorques1(eigenVector);
    JointPositions jointPositions2(stdVector);
    JointVelocities jointVelocities2(stdVector);
    JointAccelerations jointAccelerations2(stdVector);
    JointForceTorques jointForceTorques2(stdVector);
    JointPositions jointPositions3({2.17, 3.15, 0.7, 1.7, 0.5, 2.15, -16.0});
    JointVelocities jointVelocities3({2.17, 3.15, 0.7, 1.7, 0.5, 2.15, -16.0});
    JointAccelerations jointAccelerations3({2.17, 3.15, 0.7, 1.7, 0.5, 2.15, -16.0});
    JointForceTorques jointForceTorques3({2.17, 3.15, 0.7, 1.7, 0.5, 2.15, -16.0});

    /**
     * Assignment operators from Eigen::VectorXd, std::vector<double> and initialiser_list
    */
    jointPositions1 = eigenVector;
    jointVelocities1 = eigenVector;
    jointAccelerations1 = eigenVector;
    jointForceTorques1 = eigenVector;
    jointPositions1 = stdVector;
    jointVelocities1 = stdVector;
    jointAccelerations1 = stdVector;
    jointForceTorques1 = stdVector;
    jointPositions1 = {2.17, 3.15, 0.7, 1.7, 0.5, 2.1, -16.0};
    jointVelocities1 = {2.17, 3.15, 0.7, 1.7, 0.5, 2.1, -16.0};
    jointAccelerations1 = {2.17, 3.15, 0.7, 1.7, 0.5, 2.1, -16.0};
    jointForceTorques1 = {2.17, 3.15, 0.7, 1.7, 0.5, 2.1, -16.0};

    /**
     * Index operator.
    */
    jointPositions1[4] = input;
    jointVelocities1[4] = input;
    jointAccelerations1[4] = input;
    jointForceTorques1[4] = input;
    output = jointPositions1[4];
    output = jointVelocities1[4];
    output = jointAccelerations1[4];
    output = jointForceTorques1[4];

    /**
     * Size function. Returns the number of the joints.
    */
    jointPositions1.size();
    jointVelocities1.size();
    jointAccelerations1.size();
    jointForceTorques1.size();

    /**
     * Raw function, returns Eigen::VectorXd with the coordinates.
    */
    jointPositions1.raw();
    jointVelocities1.raw();
    jointAccelerations1.raw();
    jointForceTorques1.raw();

    /**
     * Print to stream operator.
    */
    std::cout << jointPositions1 << std::endl;
    std::cout << jointVelocities1 << std::endl;
    std::cout << jointAccelerations1 << std::endl;
    std::cout << jointForceTorques1 << std::endl;

    return 0;
}
