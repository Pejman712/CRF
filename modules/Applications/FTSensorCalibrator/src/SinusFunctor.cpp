/* © Copyright CERN 2018. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Zsolt Pasztori EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#include "Eigen/Eigen"
#include "FTSensorCalibrator/SinusFunctor.hpp"

namespace crf {
namespace applications {
namespace ftsensorcalibrator {


int sinusFunctor::operator()(const Eigen::VectorXf &x, Eigen::VectorXf &fvec) const {
    float amplitude = x(0);
    float phase = x(1);
    float offset = x(2);

    for (int i = 0; i < values(); i++) {
        float xValue = measuredValues(i, 0);
        float yValue = measuredValues(i, col);
        fvec(i) = yValue - (amplitude * sin(w * xValue + phase) + offset);
    }
    return 0;
}

int sinusFunctor::df(const Eigen::VectorXf &x, Eigen::MatrixXf &fjac) const {
    float amplitude = x(0);
    float phase = x(1);
    // float offset = x(2);
    for (int i = 0; i < values(); i++) {
        float xValue = measuredValues(i, 0);
        fjac(i, 0) = -sin(w * xValue + phase);
        fjac(i, 1) = -amplitude*cos(w * xValue + phase);
        fjac(i, 2) = -1;
    }
    return 0;
}

int sinusFunctor::values() const {
    return m;
}

int sinusFunctor::inputs() const {
    return n;
}

}  // namespace ftsensorcalibrator
}  // namespace applications
}  // namespace crf
