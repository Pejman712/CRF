/* © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Hannes Gamper CERN BE/CEM/MRO 2021
 *         Laura Rodrigo Pérez CERN BE/CEM/MRO 2021
 *
 *  ================================================================================================================
 */

#include <math.h>
#include <optional>

#include "GeometricMethods/Sinusoid/Sinusoid.hpp"

namespace crf {
namespace math {
namespace geometricmethods {

Sinusoid::Sinusoid(double start0thDerivative,
    double end0thDerivative,
    double definitionValue,
    ComputationMethod method):
    start0thDerivative_(start0thDerivative),
    logger_("Sinusoid") {
    logger_->debug("Ctor");
    curveScalingFactor_ = end0thDerivative - start0thDerivative_;
    if (curveScalingFactor_ == 0.0 || definitionValue == 0.0) {
        throw std::runtime_error("Input parameters not valid");
    }
    switch (method) {
        case crf::math::geometricmethods::ComputationMethod::Limit1stDerivative: {
            double maxDerivative1Value = definitionValue;
            double maxDerivative1Scaled = abs(maxDerivative1Value / curveScalingFactor_);
            w_ = M_PI*maxDerivative1Scaled;
            max2ndDerivative_ = 2*pow(maxDerivative1Scaled, 2);
            break;
        }
        case crf::math::geometricmethods::ComputationMethod::SetRange: {
            double endEvalPointValue = definitionValue;
            w_ = 2*M_PI/endEvalPointValue;
            max2ndDerivative_ = 2*pow(w_/M_PI, 2);
            break;
        }
    }
}

Sinusoid::~Sinusoid() {
    logger_->debug("DTor");
}

std::optional<double> Sinusoid::getRange() const {
    logger_->debug("getRange");
    return (2*M_PI)/w_;
}

std::optional<double> Sinusoid::evaluate(double evaluationPoint, unsigned int derivative) {
    logger_->debug("evaluate");

    if (evaluationPoint < 0.0) {
        evaluationPoint = 0.0;
        logger_->warn("The evaluation point introduced has exceeded the range: [{},{}]. {} has "
            "been asigned as evaluation point.", evaluationPoint, static_cast<double>((2*M_PI)/w_),
            evaluationPoint);
    } else if (evaluationPoint > (2*M_PI)/w_) {
        evaluationPoint = (2*M_PI)/w_;
        logger_->warn("The evaluation point introduced has exceeded the range: [{},{}]. {} has "
            "been asigned as evaluation point.", 0.0, evaluationPoint, evaluationPoint);
    }

    if (derivative > 2) {
        logger_->error("This function only can do from 0 to 2 Derivative");
        return std::nullopt;
    }

    double result = 0.0;
    switch (derivative) {
        case 0 :
            if (evaluationPoint < 0) {
                result = start0thDerivative_;
            } else if (evaluationPoint < M_PI/w_) {
                result = start0thDerivative_ + curveScalingFactor_ * ((max2ndDerivative_/ \
                    (4*pow(w_, 2))) * (pow(cos(w_*evaluationPoint), 2) + (pow(w_, 2)* \
                    pow(evaluationPoint, 2))-1));
            } else if (evaluationPoint < (2*M_PI)/w_) {
                result = start0thDerivative_ + curveScalingFactor_ * (-(max2ndDerivative_/ \
                    (4*pow(w_, 2))) * (pow(cos(w_*evaluationPoint), 2) + (pow(w_, 2)* \
                    pow(evaluationPoint, 2))-1) + (sqrt(2*max2ndDerivative_)*evaluationPoint)- \
                    ((max2ndDerivative_*pow(M_PI, 2))/(2*pow(w_, 2))));
            } else {
                result = start0thDerivative_ + curveScalingFactor_;
            }
            break;
        case 1 :
            if (evaluationPoint < 0) {
                result = 0;
            } else if (evaluationPoint < M_PI/w_) {
                result = curveScalingFactor_ * (-(max2ndDerivative_/(2*w_))* \
                    (cos(w_*evaluationPoint) * sin(w_*evaluationPoint)-(w_*evaluationPoint)));
            } else if (evaluationPoint < (2*M_PI)/w_) {
                result = curveScalingFactor_ * ((max2ndDerivative_/(2*w_))* \
                    (cos(w_*evaluationPoint) * sin(w_*evaluationPoint)-(w_*evaluationPoint))+ \
                    sqrt(2*max2ndDerivative_));
            } else {
                result = 0;
            }
            break;
        case 2 :
            if (evaluationPoint < 0) {
                result = 0;
            } else if (evaluationPoint < M_PI/w_) {
                result = curveScalingFactor_ * (max2ndDerivative_*pow(sin(w_*evaluationPoint), 2));
            } else if (evaluationPoint < (2*M_PI)/w_) {
                result = curveScalingFactor_ * (-max2ndDerivative_*pow(sin(w_*evaluationPoint), 2));
            } else {
                result = 0;
            }
            break;
    }
    return result;
}

}  // namespace geometricmethods
}  // namespace math
}  // namespace crf
