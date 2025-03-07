/* © Copyright CERN 2022. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Jorge Playán Garai CERN BE/CEM/MRO 2022
 *
 *  ================================================================================================================
 */

#include <optional>

#include "GeometricMethods/CubicPolynomial/CubicPolynomial.hpp"

using crf::math::geometricmethods::CubicPolynomial;

int main(int argc, char** argv) {
    double start0thderivative = 0;
    double end0thderivative = 1;
    double start1stderivative = 0;
    double end1stderivative = 0;
    double range = 3;

    CubicPolynomial poli(
        start0thderivative, end0thderivative, start1stderivative, end1stderivative, range);

    std::cout << "0th derivative in t = 3 is " << poli.evaluate(3, 0).value() << std::endl;
    std::cout << "1st derivative in t = 3 is " << poli.evaluate(3, 1).value() << std::endl;
    std::cout << "2nd derivative in t = 3 is " << poli.evaluate(3, 2).value() << std::endl;
    std::cout << "3rd derivative in t = 3 is " << poli.evaluate(3, 3).value() << std::endl;

    std::vector<double> points0thderivative = {0, 1};
    start1stderivative = 0;
    end1stderivative = 0;
    std::vector<double> ranges = {0, 3};

    // With a tolerance of 1e-3 the speed is around 3x as fast but the solution is less precise
    CubicPolynomial poliNlopt(0, 10, 0, 0, 1, 1, 1e-5, std::chrono::milliseconds(1000));

    std::optional<double> rangeOpt = poliNlopt.getRange();

    if (!rangeOpt) {
        std::cout << "Range could not be calculated" << std::endl;
        return -1;
    }

    range = rangeOpt.value();

    std::cout << "First pos: 0 vs " << poliNlopt.evaluate(0, 0).value() << std::endl;
    std::cout << "Last pos: 10 vs " << poliNlopt.evaluate(range, 0).value() << std::endl;
    std::cout << "First vel: 0 vs " << poliNlopt.evaluate(0, 1).value() << std::endl;
    std::cout << "Last vel: 0 vs" << poliNlopt.evaluate(range, 1).value() << std::endl;
    std::cout << "First acc: 1 > " << poliNlopt.evaluate(0, 2).value() << std::endl;
    std::cout << "Last acc: 1 > " << poliNlopt.evaluate(range, 2).value() << std::endl;
    std::cout << "Max velocity: 1 > " << poliNlopt.evaluate(range/2, 1).value() << std::endl;
}
