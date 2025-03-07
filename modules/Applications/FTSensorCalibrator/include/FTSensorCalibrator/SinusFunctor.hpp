#pragma once
/* © Copyright CERN 2018. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Zsolt Pasztori EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#include <fstream>

#include "Eigen/Eigen"

namespace crf {
namespace applications {
namespace ftsensorcalibrator {

struct sinusFunctor {
    Eigen::MatrixXf measuredValues;
    float w;
    int col = 1;
    int operator()(const Eigen::VectorXf &x, Eigen::VectorXf &fvec) const;  // NOLINT

    int df(const Eigen::VectorXf &x, Eigen::MatrixXf &fjac) const;  // NOLINT

    // Number of data points, i.e. values.
    int m;

    // Returns 'm', the number of values.
    int values() const;

    // The number of parameters, i.e. inputs.
    int n;

    // Returns 'n', the number of inputs.
    int inputs() const;
};

}  // namespace ftsensorcalibrator
}  // namespace applications
}  // namespace crf
