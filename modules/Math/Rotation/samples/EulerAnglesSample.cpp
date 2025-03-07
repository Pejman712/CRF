/* © Copyright CERN 2024. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Bartosz Sójka CERN BE/CEM/MRO 2024
 *
 *  ================================================================================================================
 */

#include "Rotation/EulerAngles.hpp"

using crf::math::rotation::CardanXYZ;
using crf::math::rotation::EulerZXZ;

int main() {
    double input = 1.7;
    double output = 0;
    /**
     * Sample std array from which euler angles can be constructed
    */
    const std::array<double, 3> array({3.14, 1.7, 0.5});

    /**
     * Default constructors.
     * Creates [0.0, 0.0, 0.0] euler angles.
    */
    CardanXYZ cardanXYZ0;
    EulerZXZ eulerZXZ0;

    /**
     * Constructors from std::array<double, 3> and initialiser_list
    */
    CardanXYZ cardanXYZ1(array);
    EulerZXZ eulerZXZ1(array);
    CardanXYZ cardanXYZ2({2.17, 3.15, 0.7});
    EulerZXZ eulerZXZ2({2.17, 3.15, 0.7});

    /**
     * Assignment operators from std::array<double, 3> and initialiser_list
    */
    cardanXYZ1 = array;
    eulerZXZ1 = array;
    cardanXYZ1 = {2.17, 3.15, 0.7};
    eulerZXZ1 = {2.17, 3.15, 0.7};

    /**
     * Index operator.
    */
    cardanXYZ1[1] = input;
    eulerZXZ1[1] = input;
    output = cardanXYZ1[1];
    output = eulerZXZ1[1];

    /**
     * Size operator. Always returns 3.
    */
    cardanXYZ1.size();
    eulerZXZ1.size();

    /**
     * RawArray operator, returns std::array<double, 3> with the euler angles
    */
    cardanXYZ1.rawArray();
    eulerZXZ1.rawArray();

    /**
     * Print to stream operator.
    */
    std::cout << cardanXYZ1 << std::endl;
    std::cout << eulerZXZ1 << std::endl;

    return 0;
}
