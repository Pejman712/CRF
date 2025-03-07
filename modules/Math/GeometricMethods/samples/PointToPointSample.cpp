/* © Copyright CERN 2022. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Jorge Playán Garai CERN BE/CEM/MRO 2022
 *
 *  ================================================================================================================
 */

#include <optional>

#include "GeometricMethods/PointToPoint/PointToPoint.hpp"

using crf::math::geometricmethods::PointToPoint;

int main(int argc, char** argv) {
    std::vector<double> points0thderivative = {0, 1};
    double maxVel = 1;
    double maxAcc = 3;

    PointToPoint poliVia(points0thderivative, maxVel, maxAcc);

    std::cout << "0th derivative in t = 3 is " << poliVia.evaluate(3, 0).value() << std::endl;
    std::cout << "1st derivative in t = 3 is " << poliVia.evaluate(3, 1).value() << std::endl;
    std::cout << "2nd derivative in t = 3 is " << poliVia.evaluate(3, 2).value() << std::endl;
    std::cout << "3rd derivative in t = 3 is " << poliVia.evaluate(3, 3).value() << std::endl;

    return 0;
}
