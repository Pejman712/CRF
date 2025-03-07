#pragma once
/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license. 
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 * 
 * Author: Julia Kabalar CERN EN/SMM/MRO
 * 
 *  ==================================================================================================
 */

#include <memory>
#include <vector>

#include "EventLogger/EventLogger.hpp"
#include "StateEstimator/DefaultMeasurementModel.hpp"
#include "StateEstimator/DefaultSystemModel.hpp"
#include "StateEstimator/kalman/LinearizedSystemModel.hpp"
#include "StateEstimator/kalman/LinearizedMeasurementModel.hpp"
#include "StateEstimator/kalman/ExtendedKalmanFilter.hpp"
#include "StateEstimator/kalman/UnscentedKalmanFilter.hpp"

#include "StateEstimator/IStateEstimator.hpp"

namespace crf {
namespace algorithms {
namespace stateestimator {

/**
* @brief Template StateEstimator
* @param INPUTVECTORSIZE defines the vector size for the measurement values
* @param STATEVECTORSIZE defines the vector size for the state vector
*/
template<int STATEVECTORSIZE, int INPUTVECTORSIZE>
class StateEstimator : public IStateEstimator {
    static_assert(STATEVECTORSIZE > 0, "State vector size is too small!");
    static_assert(INPUTVECTORSIZE > 0, "Input vector size is too small!");
    static_assert(STATEVECTORSIZE >= INPUTVECTORSIZE, "Defined state vector is too small!");

 public:
    /**
    * Constructor
    * 
    * @param kalmanFilterType enum to choose available implementations of StandardFilterBase
    * @param initialState initial state of the system provided as an array with STATEVECTORSIZE
    * @param measurementModel linearized measurement model linking the measured values to the actual state type
    * @param systemModel linearized measurement model defining how the system state evolves over time
    */
    StateEstimator(std::array<float, STATEVECTORSIZE> initialState,
        StateEstimatorFilterType kalmanFilterType = StateEstimatorFilterType::UNSCENTED_KF,
        std::shared_ptr<Kalman::LinearizedMeasurementModel<
            Kalman::Vector<float, STATEVECTORSIZE>,
            Kalman::Vector<float, INPUTVECTORSIZE>, Kalman::StandardBase>> measurementModel =
            std::make_shared<DefaultMeasurementModel<STATEVECTORSIZE, INPUTVECTORSIZE>>(),
        std::shared_ptr<Kalman::LinearizedSystemModel<
            Kalman::Vector<float, STATEVECTORSIZE>,
            Kalman::Vector<float, 0>, Kalman::StandardBase>> systemModel =
            std::make_shared<DefaultSystemModel<STATEVECTORSIZE>>()):
        logger_("StateEstimator"),
        kalmanFilterType_(kalmanFilterType),
        measurementModel_(measurementModel),
        systemModel_(systemModel),
        ekf_(),
        ukf_() {
        logger_->debug("CTor");
        systemState_.setZero();
        for (int dimID = 0; dimID < STATEVECTORSIZE; dimID++) {
            systemState_(dimID) = initialState[dimID];
        }
        switch (kalmanFilterType_) {
            case StateEstimatorFilterType::UNSCENTED_KF:
                ukf_.init(systemState_);
                break;
            case StateEstimatorFilterType::EXTENDED_KF:
                ekf_.init(systemState_);
                break;
            default:
                throw std::runtime_error("The state estimator type is not implemented");
        }
    }
    ~StateEstimator() {
        logger_->debug("DTor");
    }
    std::vector<float> getEstimate() override {
        logger_->debug("getEstimate");
        Kalman::Vector<float, 0> control;
        systemState_ = systemModel_->f(systemState_, control);
        std::vector<float> systemStateVector;
        for (int dimID = 0; dimID < STATEVECTORSIZE; dimID++) {
            systemStateVector.push_back(systemState_(dimID));
        }
        return systemStateVector;
    }

    bool addMeasurement(const std::vector<float>& measuredValues) override {
        logger_->debug("addMeasurement");
        if (measuredValues.size() != INPUTVECTORSIZE) {
            logger_->debug("measuredValues are of invalid size");
            return false;
        }
        Kalman::Vector<float, INPUTVECTORSIZE> measuredState;
        for (int dimID = 0; dimID < INPUTVECTORSIZE; dimID++) {
            measuredState(dimID) = measuredValues[dimID];
        }
        Kalman::Vector<float, STATEVECTORSIZE> currentSystemState;
        if (kalmanFilterType_ == StateEstimatorFilterType::UNSCENTED_KF) {
                logger_->info("updating UKF");
                ukf_.predict(*systemModel_);
                currentSystemState = ukf_.update(*measurementModel_, measuredState);
        } else if (kalmanFilterType_ == StateEstimatorFilterType::EXTENDED_KF) {
                logger_->info("updating EKF");
                ekf_.predict(*systemModel_);
                currentSystemState = ekf_.update(*measurementModel_, measuredState);
        } else {
            return false;
        }
        systemState_ = currentSystemState;
        return true;
    }
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW_IF(NeedsToAlign)

 protected:
    utility::logger::EventLogger logger_;
    StateEstimatorFilterType kalmanFilterType_;
    std::shared_ptr<Kalman::LinearizedMeasurementModel<
        Kalman::Vector<float, STATEVECTORSIZE>,
        Kalman::Vector<float, INPUTVECTORSIZE>,
        Kalman::StandardBase>> measurementModel_;
    std::shared_ptr<Kalman::LinearizedSystemModel<
        Kalman::Vector<float, STATEVECTORSIZE>,
        Kalman::Vector<float, 0>,
        Kalman::StandardBase>> systemModel_;
    Kalman::UnscentedKalmanFilter<Kalman::Vector<float, STATEVECTORSIZE>> ukf_;
    Kalman::ExtendedKalmanFilter<Kalman::Vector<float, STATEVECTORSIZE>> ekf_;
    Kalman::Vector<float, STATEVECTORSIZE> systemState_;
    // Using macro conditionnally (depending on template parameters)
    // see more in https://eigen.tuxfamily.org/dox/group__TopicStructHavingEigenMembers.html
    enum {
        NeedsToAlign = (sizeof(systemState_)%16) == 0
    };
};

}  // namespace stateestimator
}  // namespace algorithms
}  // namespace crf


