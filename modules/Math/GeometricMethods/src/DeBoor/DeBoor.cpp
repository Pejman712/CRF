/* © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Hannes Gamper CERN BE/CEM/MRO 2021
 *         Laura Rodrigo Pérez CERN BE/CEM/MRO 2021
 *
 *  ================================================================================================================
 */

#include <optional>

#include "EventLogger/EventLogger.hpp"
#include "GeometricMethods/DeBoor/DeBoor.hpp"

namespace crf {
namespace math {
namespace geometricmethods {

DeBoor::DeBoor(unsigned int degree, std::vector<double> knots, std::vector<double> controlPoints):
    logger_("DeBoor"),
    degree_(degree),
    knots_(knots),
    controlPoints_(controlPoints),
    nKnots_(knots_.size()),
    nControlPoints_(controlPoints_.size()) {
    logger_->debug("Ctor");
    if (nControlPoints_ == 0 || nKnots_ == 0 || degree_ == 0) {
        throw std::runtime_error("Input parameters not valid");
    }
    // TODO(@hagamper): Properly defined this variable avoiding things like 2+1.
    int n_m = controlPoints_.size()+degree_+2+1;
    a_.resize(n_m);
    Pk_.resize(n_m);
    for (int i = 0 ; i < n_m ; i++) {
        a_[i].resize(n_m);
        Pk_[i].resize(n_m);
    }
}

DeBoor::~DeBoor() {
    logger_->debug("DTor");
}

std::optional<double> DeBoor::getRange() const {
    logger_->debug("getRange");
    return knots_[nKnots_-1];
}

std::optional<double> DeBoor::evaluate(double evaluationPoint, unsigned int derivative) {
    logger_->debug("evaluate");

    if (derivative != 0) {
        logger_->error("This function only can do 0 Derivative - Returning empty vector");
        return std::nullopt;
    }

    // Saturation for input evaluationPoint to the range on which spline is defined (given by knots)
    if (evaluationPoint < 0.0) {
        logger_->error("The evaluation point must be positive - Returning empty vector");
        return std::nullopt;
    } else if (evaluationPoint < knots_[0]) {
        evaluationPoint = knots_[0];
    } else if (evaluationPoint > knots_[nKnots_-1]) {
        evaluationPoint = knots_[nKnots_-1];
        logger_->info("The evaluation point introduced has exceeded the range: {}. It has been "
            "asigned as evaluation point.", evaluationPoint);
    }

    // Find evaluationPoint in knots.
    int s = 0;  // Amount of values evaluationPoint in knots.
    int ix = degree_;  // Indicates in which interval of knots evaluationPoint lies.
    for (int i = 0; i < nKnots_; i++) {
        if (evaluationPoint == knots_[i]) {
            s = s+1;
        }
        if (evaluationPoint >= knots_[i]) {
            ix = i;
        }
    }

    // Use <= because an offset (like starting arrays from 0) doesnt effect differentials
    for (int i=ix-degree_; i <= ix-s; i++) {
        Pk_[i][0] = controlPoints_[i];
    }

    // Identify degree+1 relevant control points.
    int h = degree_-s;
    int q = ix-1;
    double output = 0.0f;
    if (h > 0) {
        for (int r = 0; r < h; r++) {
            for (int i = (q-degree_+r+1); i <= (q-s); i++) {
                a_[i+1][r+1] = (evaluationPoint-knots_[i+1])/(knots_[i+degree_-r+1+1-1]- \
                    knots_[i+1]);
                Pk_[i+1][r+1] = (1.0-a_[i+1][r+1])*Pk_[i][r]+a_[i+1][r+1]*Pk_[i+1][r];
            }
        }
        output = Pk_[ix-s][degree_-s];
    } else if (ix == nKnots_-1) {  // Last control point is a special case
        output = controlPoints_[nControlPoints_-1];
    } else {
        output = controlPoints_[ix-degree_];
    }
    return output;
}

}  // namespace geometricmethods
}  // namespace math
}  // namespace crf
