/* © Copyright CERN 2019.  All rights reserved. This software is released under a CERN proprietary software licence.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Jorge Playán Garai CERN EN/SMM/MRO 2019
 *
 *  ==================================================================================================
 */

#include <string>
#include <memory>
#include <vector>
#include <random>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <Eigen/Dense>

#include "KalmanFilter/ExtendedKalmanFilter.hpp"
#include "EventLogger/EventLogger.hpp"

using testing::_;
using testing::Invoke;
using testing::NiceMock;

using crf::math::kalmanfilter::ExtendedKalmanFilter;
using crf::math::kalmanfilter::StateSpace;
using crf::math::kalmanfilter::SystemModelLinearized;
using crf::math::kalmanfilter::ObservationModelLinearized;

class ExtendedKalmanFilterShould: public ::testing::Test {
 protected:
    ExtendedKalmanFilterShould(): logger_("ExtendedKalmanFilterShould") {
        logger_->info("{} BEGIN", testing::UnitTest::GetInstance()->current_test_info()->name());
    }
    ~ExtendedKalmanFilterShould() {
        logger_->info("{} END with {}",
            testing::UnitTest::GetInstance()->current_test_info()->name(),
            testing::UnitTest::GetInstance()->current_test_info()->result()->Passed());
    }

    void SetUp() {
        sut_ = std::make_unique<ExtendedKalmanFilter>();
    }

    crf::utility::logger::EventLogger logger_;

    std::unique_ptr<ExtendedKalmanFilter> sut_;

    const Eigen::MatrixXd systemNoise_ {
        {0.01, 0, 0},
        {0, 0.01, 0},
        {0, 0, 0.01}
    };
    const Eigen::MatrixXd measurementNoise_ {
        {1e-4, 0, 0},
        {0, 1e-4, 0},
        {0, 0, 1e-4}
    };
};

TEST_F(ExtendedKalmanFilterShould, removeGaussianNoise) {
    std::random_device rd;
    std::mt19937 gen(rd());

    Eigen::VectorXd realValue {
        {0.5},
        {1.0},
        {1.5}
    };

    Eigen::VectorXd initialEstimation {
        {0},
        {0},
        {0}
    };

    Eigen::MatrixXd initialCovariance {
        {0, 0, 0},
        {0, 0, 0},
        {0, 0, 0}
    };

    std::shared_ptr<StateSpace> stateSpace =
        std::make_shared<StateSpace>(initialEstimation, initialCovariance);
    std::shared_ptr<SystemModelLinearized> systemModel =
        std::make_shared<SystemModelLinearized>(systemNoise_);
    std::shared_ptr<ObservationModelLinearized> observationModel =
        std::make_shared<ObservationModelLinearized>(measurementNoise_);

    for (uint32_t i = 0; i < 100; i++) {
        // Prediction
        Eigen::VectorXd control;
        systemModel->evaluate(stateSpace->getMean(), control);
        sut_->prediction(systemModel, stateSpace);

        // Simualte measure with normal noise
        Eigen::VectorXd measure;
        measure.resize(realValue.rows(), realValue.cols());
        for (uint32_t j = 0; j < measurementNoise_.rows(); j++) {
            std::normal_distribution<double> noise(0, measurementNoise_(j, j));
            measure[j] = (realValue[j] + noise(gen));
        }

        observationModel->evaluate(stateSpace->getMean());
        observationModel->setMeasurement(measure);

        // Correction
        sut_->correction(observationModel, stateSpace);

        logger_->debug("Measure: \n{}", measure);
        logger_->debug("New mean \n{}", stateSpace->getMean());
        logger_->debug("New cova \n{}", stateSpace->getCovariance());
    }

    for (uint32_t i = 0; i < realValue.size(); i++) {
        ASSERT_NEAR(stateSpace->getMean()(i), realValue[i], 0.01);
    }
}
