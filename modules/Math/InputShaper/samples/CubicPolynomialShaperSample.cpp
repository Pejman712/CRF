/* © Copyright CERN 2022. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Jorge Playán Garai CERN BE/CEM/MRO 2022
 *
 *  ================================================================================================================
 */

#include <optional>

#include "InputShaper/CubicPolynomialShaper/CubicPolynomialShaper.hpp"

using crf::math::inputshaper::CubicPolynomialShaper;

int main(int argc, char** argv) {
    CubicPolynomialShaper shaper(0, 1);
    shaper.setResponsivenessFactor(2);
    shaper.setReference(1.0);
    bool flag = true;
    bool flag2 = true;
    for (double t = 0.0; t < 9.0; t+=0.01) {  // 10 ms
        std::cout << "Input in time " << t << " is: " << shaper.getInputPoint(t) << std::endl;
        if (t > 2.0 && flag) {
            shaper.setReference(0.0);
            flag = false;
        }
        if (t > 4.0 && flag2) {
            shaper.setReference(1.0);
            flag2 = false;
        }
    }
    return 0;
}
