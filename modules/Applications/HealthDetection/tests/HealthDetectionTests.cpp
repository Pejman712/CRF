/* © Copyright CERN 2020. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Arturs Ivanovs CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <algorithm>
#include <fstream>
#include <iostream>
#include <complex>
#include <cstdlib>
#include <vector>
#include <memory>
#include <string>

#include "HealthDetection/HealthDetection.hpp"
#include "PeakDetection/CFAR.hpp"
#include "PeakDetection/GradientPeakDetection.hpp"
#include "StateEstimator/IStateEstimator.hpp"
#include "StateEstimator/DefaultSystemModel.hpp"
#include "StateEstimator/DefaultMeasurementModel.hpp"
#include "StateEstimator/StateEstimator.hpp"

#include "Mocks/Sensors/FraunhoferRadarMock.hpp"

#define STATEVECTORSIZE 1
#define MEASUREMENTVECTORSIZE 1

using testing::_;
using testing::SetArgReferee;
using testing::NiceMock;
using ::testing::Invoke;

using crf::sensors::fraunhoferradar::RadarMock;
using crf::applications::healthdetection::HealthDetection;
using crf::applications::healthdetection::VitalSignalPacket;
using crf::applications::healthdetection::RadarPacket;
using crf::algorithms::peakdetection::CFAR;
using crf::algorithms::peakdetection::GradientPeakDetection;
using crf::algorithms::stateestimator::StateEstimator;

class HealthDetectorShould: public ::testing::Test {
 protected:
    HealthDetectorShould(): logger_("HealthDetectorShould"),
    path_(__FILE__) {
    logger_->info("{0} BEGIN", testing::UnitTest::GetInstance()->current_test_info()->name());
    path_ = path_.substr(0, path_.find("cpproboticframework/"));
    radarMock_.reset(new NiceMock<RadarMock>);
    cfar_.reset(new CFAR(10, 40, 0.001));
    heartPeakDetector_.reset(new GradientPeakDetection(0.1));
    respirationPeakDetector_.reset(new GradientPeakDetection(0.5));
    auto measurementModel = std::make_shared<crf::algorithms::stateestimator::
        DefaultMeasurementModel<STATEVECTORSIZE, MEASUREMENTVECTORSIZE>>();
    auto systemModel = std::make_shared<crf::algorithms::stateestimator::
        DefaultSystemModel<STATEVECTORSIZE>>();
    auto type = crf::algorithms::stateestimator::StateEstimatorFilterType::UNSCENTED_KF;
    std::array<float, STATEVECTORSIZE> initialState{0};
    stateEstimator_.reset(new StateEstimator<STATEVECTORSIZE, MEASUREMENTVECTORSIZE>(
        initialState, type, measurementModel, systemModel));

    ON_CALL(*radarMock_, initialize()).WillByDefault(::testing::Return(true));
    ON_CALL(*radarMock_, deinitialize()).WillByDefault(::testing::Return(true));
    ON_CALL(*radarMock_, getMaxObservationFrequency()).WillByDefault(::testing::Return(9.25926));
    }

    ~HealthDetectorShould() {
        logger_->info("{0} END with {1}",
        testing::UnitTest::GetInstance()->current_test_info()->name(),
        testing::UnitTest::GetInstance()->current_test_info()->result()->Passed());
    }

    std::vector<std::vector<float>> importMockData(const std::string& fileName) {
        std::vector<std::vector<float>> mockData;
        std::ifstream mockDataFile(path_ + "cpproboticframework/bin/TestData/" + fileName);
        if (!mockDataFile.is_open()) {
            logger_->error("Unable to open file: {}", path_ + "testFiles/" + fileName);
            throw std::runtime_error("failed to open file");
        }
        std::string line;
        while (getline(mockDataFile, line, '\n')) {
            std::stringstream ss(line);
            std::vector<float> numbers;
            std::string in_line;
            while (getline(ss, in_line, ' ')) {
                numbers.push_back(std::stof(in_line, 0));
            }
            mockData.push_back(numbers);
        }
        mockDataFile.close();
        return mockData;
    }

    crf::utility::logger::EventLogger logger_;
    std::shared_ptr<NiceMock<RadarMock>> radarMock_;
    std::shared_ptr<CFAR> cfar_;
    std::shared_ptr<GradientPeakDetection> heartPeakDetector_, respirationPeakDetector_;
    std::unique_ptr<HealthDetection> sut_;
    std::shared_ptr<crf::algorithms::stateestimator::StateEstimator<
      STATEVECTORSIZE, MEASUREMENTVECTORSIZE>> stateEstimator_;
    std::string path_;
};

TEST_F(HealthDetectorShould, returnFalseIfCouldNotInitializeRadar) {
    EXPECT_CALL(*radarMock_, initialize()).WillOnce(::testing::Return(false));
    sut_.reset(new HealthDetection(radarMock_, cfar_, heartPeakDetector_, respirationPeakDetector_,
        stateEstimator_));
    ASSERT_FALSE(sut_->initialize());
}

TEST_F(HealthDetectorShould, ReturnFalseIfAlreadyRunningAndShouldNotStartTwice) {
    sut_.reset(new HealthDetection(radarMock_, cfar_, heartPeakDetector_, respirationPeakDetector_,
        stateEstimator_));
    ASSERT_TRUE(sut_->initialize());
    ASSERT_FALSE(sut_->initialize());
    ASSERT_TRUE(sut_->deinitialize());
    ASSERT_FALSE(sut_->deinitialize());

    ASSERT_TRUE(sut_->initialize());
    ASSERT_FALSE(sut_->initialize());
    ASSERT_TRUE(sut_->deinitialize());
    ASSERT_FALSE(sut_->deinitialize());
}

TEST_F(HealthDetectorShould, returnFalseIfNoPeaksWereDetected) {
    boost::optional<VitalSignalPacket> receivedVitalSignalPacket;
    std::vector<std::vector<float>> fakeRadarData;
    for (int i = 0; i < 100; i++) {
        std::vector<float> fakeDataVec;
        for (int j = 0; j < 100; j++) {
            fakeDataVec.push_back(0);
        }
        fakeRadarData.push_back(fakeDataVec);
    }
    EXPECT_CALL(*radarMock_, getFrame()).WillOnce(::testing::Return(fakeRadarData));
    sut_.reset(new HealthDetection(radarMock_, cfar_, heartPeakDetector_, respirationPeakDetector_,
        stateEstimator_));
    ASSERT_TRUE(sut_->initialize());
    receivedVitalSignalPacket = sut_->getVitalSigns();
    ASSERT_FALSE(receivedVitalSignalPacket);
}

TEST_F(HealthDetectorShould, returnEmptyVarianceIfPeakVarianceTooHigh) {
    boost::optional<VitalSignalPacket> receivedVitalSignalPacket;
    std::vector<std::vector<float>> fakeRadarData;
    try {
        fakeRadarData = importMockData("Sensors/Radar/RadarHighVariance.dat");
    } catch (std::exception &ex) {
        logger_->error(ex.what());
        FAIL();
    }
    EXPECT_CALL(*radarMock_, getFrame()).WillOnce(::testing::Return(fakeRadarData));
    sut_.reset(new HealthDetection(radarMock_, cfar_, heartPeakDetector_, respirationPeakDetector_,
        stateEstimator_));
    ASSERT_TRUE(sut_->initialize());
    receivedVitalSignalPacket = sut_->getVitalSigns();
    ASSERT_FALSE(receivedVitalSignalPacket);
}

TEST_F(HealthDetectorShould, returnNoMovementIfPhaseChangeTooSmall) {
    boost::optional<VitalSignalPacket> receivedVitalSignalPacket;
    std::vector<std::vector<float>> fakeRadarData;
    try {
        fakeRadarData = importMockData("Sensors/Radar/RadarNoPersonInFrame.dat");
    } catch (std::exception &ex) {
        logger_->error(ex.what());
        FAIL();
    }
    EXPECT_CALL(*radarMock_, getFrame()).WillOnce(::testing::Return(fakeRadarData));
    sut_.reset(new HealthDetection(radarMock_, cfar_, heartPeakDetector_, respirationPeakDetector_,
        stateEstimator_));
    ASSERT_TRUE(sut_->initialize());
    receivedVitalSignalPacket = sut_->getVitalSigns();
    ASSERT_FALSE(receivedVitalSignalPacket);
}

TEST_F(HealthDetectorShould, returnReturnCorrectAmountOfRespirationAndHeartBeat) {
    boost::optional<VitalSignalPacket> receivedVitalSignalPacket;
    std::vector<std::vector<float>> fakeRadarData;
    try {
        fakeRadarData = importMockData("Sensors/Radar/RadarBreathingPersonInFrame.dat");
    } catch (std::exception &ex) {
        logger_->error(ex.what());
        FAIL();
    }
    EXPECT_CALL(*radarMock_, getFrame()).WillOnce(::testing::Return(fakeRadarData));
    sut_.reset(new HealthDetection(radarMock_, cfar_, heartPeakDetector_, respirationPeakDetector_,
        stateEstimator_));
    ASSERT_TRUE(sut_->initialize());
    receivedVitalSignalPacket = sut_->getVitalSigns();
    ASSERT_EQ(receivedVitalSignalPacket.get().respirationRate, 8);
    ASSERT_EQ(receivedVitalSignalPacket.get().heartRate, 92);
}
