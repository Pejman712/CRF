/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Carlos Veiga Almagro & Jorge Camarero Vera CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <fstream>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "EventLogger/EventLogger.hpp"
#include "PanTilt/DynamixelPanTilt.hpp"
#include "PanTilt/IPanTilt.hpp"

#include "Mocks/Devices/DynamixelMock.hpp"

using testing::_;
using testing::Invoke;
using testing::NiceMock;
using ::testing::Return;

class PanTiltShould : public ::testing::Test {
 protected:
    PanTiltShould() : logger_("PanTiltShould") {
        logger_->info("{} BEGIN",
            testing::UnitTest::GetInstance()->current_test_info()->name());
        dynamixelMock_.reset(new NiceMock<crf::devices::dynamixelstepper::DynamixelMock>);
        std::string testAddress = __FILE__;
        testAddress = testAddress.substr(0, testAddress.find("PanTiltTests.cpp"));
        std::ifstream dxlData(testAddress +
            "./../../../modules/Devices/Dynamixel/configuration/DXL-AX-12A.json");
        dxlData >> dxlJSON_;
        configuration_.parse(dxlJSON_);
        for (int i = 0; i < configuration_.getNumberOfMotors(); i++) {
            zeroPosition_.push_back(dxlJSON_.at("Limits").at("StartValue").get<int>());;
        }
    }

    void SetUp() override {
        ON_CALL(*dynamixelMock_, initialize()).WillByDefault(Return(true));
        ON_CALL(*dynamixelMock_, deinitialize()).WillByDefault(Return(true));
        ON_CALL(*dynamixelMock_, writeDynamixel(_, _)).WillByDefault(Return(true));
        ON_CALL(*dynamixelMock_, readCurrentPosition(_)).WillByDefault(Return(0));
        ON_CALL(*dynamixelMock_, getZeroPosition()).WillByDefault(Return(zeroPosition_));
        ON_CALL(*dynamixelMock_, getConfiguration()).WillByDefault(Return(configuration_));
    }

    ~PanTiltShould() {
        logger_->info("{} END with {}",
            testing::UnitTest::GetInstance()->current_test_info()->name(),
            testing::UnitTest::GetInstance()->current_test_info()->result()->Passed());
    }

    crf::utility::logger::EventLogger logger_;

    std::unique_ptr<crf::devices::pantilt::IPanTilt> panTilt_;
    std::shared_ptr<crf::devices::dynamixelstepper::DynamixelMock> dynamixelMock_;
    crf::devices::dynamixelstepper::DynamixelConfiguration configuration_;
    std::vector<uint16_t> zeroPosition_;
    nlohmann::json dxlJSON_;
};

TEST_F(PanTiltShould, initializeAndDeinitializeTwice) {
    panTilt_.reset(new crf::devices::pantilt::DynamixelPanTilt(dynamixelMock_));

    ASSERT_TRUE(panTilt_->initialize());
    ASSERT_FALSE(panTilt_->initialize());
    ASSERT_TRUE(panTilt_->deinitialize());
    ASSERT_FALSE(panTilt_->deinitialize());

    ASSERT_TRUE(panTilt_->initialize());
    ASSERT_FALSE(panTilt_->initialize());
    ASSERT_TRUE(panTilt_->deinitialize());
    ASSERT_FALSE(panTilt_->deinitialize());
}

TEST_F(PanTiltShould, badDynamixelInitialization) {
    EXPECT_CALL(*dynamixelMock_, initialize()).WillOnce(Return(false));

    panTilt_.reset(new crf::devices::pantilt::DynamixelPanTilt(dynamixelMock_));
    ASSERT_FALSE(panTilt_->initialize());
}

TEST_F(PanTiltShould, wrongNumberOfMotorsInConfiguration) {
    std::string testAddress = __FILE__;
    testAddress = testAddress.substr(0, testAddress.find("PanTiltTests.cpp"));
    std::ifstream dxlData(testAddress + "./data/OneMotor.json");
    nlohmann::json json;
    dxlData >> json;
    crf::devices::dynamixelstepper::DynamixelConfiguration configuration;
    configuration.parse(json);
    EXPECT_CALL(*dynamixelMock_, getConfiguration()).WillOnce(Return(configuration));

    panTilt_.reset(new crf::devices::pantilt::DynamixelPanTilt(dynamixelMock_));
    ASSERT_FALSE(panTilt_->initialize());
}

TEST_F(PanTiltShould, failToSetTheInitialPosition) {
    EXPECT_CALL(*dynamixelMock_, writeDynamixel(_, _)).WillOnce(Return(false));

    panTilt_.reset(new crf::devices::pantilt::DynamixelPanTilt(dynamixelMock_));
    ASSERT_FALSE(panTilt_->initialize());
}

TEST_F(PanTiltShould, badDynamixelDeinitialization) {
    EXPECT_CALL(*dynamixelMock_, deinitialize()).WillOnce(Return(false));

    panTilt_.reset(new crf::devices::pantilt::DynamixelPanTilt(dynamixelMock_));
    ASSERT_TRUE(panTilt_->initialize());
    ASSERT_FALSE(panTilt_->deinitialize());
}

TEST_F(PanTiltShould, setPosition) {
    std::pair<double, double> position = {0.5, 0.3};
    std::pair<double, double> givenPosition;
    auto degreeToRadian = [this](uint16_t position) {
        return (position * M_PI * dxlJSON_.at("Unit_degree").get<float>()) / 180;
    };
    EXPECT_CALL(*dynamixelMock_, writeDynamixel(_, _)).WillOnce(Return(true)).
        WillOnce(Return(true)).WillOnce(Invoke(
        [&givenPosition, &degreeToRadian](const uint8_t& index, const uint16_t& goalPosition) {
            givenPosition.first = degreeToRadian(goalPosition);
            return true;
        })).WillOnce(Invoke(
        [&givenPosition, &degreeToRadian](const uint8_t & index, const uint16_t & goalPosition) {
            givenPosition.second = degreeToRadian(goalPosition);
            return true;
        }));

    panTilt_.reset(new crf::devices::pantilt::DynamixelPanTilt(dynamixelMock_));
    ASSERT_TRUE(panTilt_->initialize());
    ASSERT_TRUE(panTilt_->setPosition(position));
    ASSERT_TRUE(panTilt_->deinitialize());
    ASSERT_EQ(position.first, std::round(givenPosition.first * 10) / 10.0f);
    ASSERT_EQ(position.second, std::round(givenPosition.second * 10) / 10.0f);
}

TEST_F(PanTiltShould, setPositionWithoutInitialization) {
    std::pair<double, double> position = {0.5, 0.3};
    panTilt_.reset(new crf::devices::pantilt::DynamixelPanTilt(dynamixelMock_));
    ASSERT_FALSE(panTilt_->setPosition(position));
}

TEST_F(PanTiltShould, getStartingPosition) {
    panTilt_.reset(new crf::devices::pantilt::DynamixelPanTilt(dynamixelMock_));
    ASSERT_TRUE(panTilt_->initialize());
    std::pair<double, double> position = panTilt_->getPosition();
    ASSERT_TRUE(panTilt_->deinitialize());
    auto radianToDegree = [this](double radians) {
        return (radians * 180) / (M_PI * dxlJSON_.at("Unit_degree").get<float>());
    };
    logger_->debug("Position: {0} {1}", position.first, position.second);
    int startingPosition = dxlJSON_.at("Limits").at("StartValue").get<int>();
    ASSERT_EQ(startingPosition, std::round(radianToDegree(position.first)));
    ASSERT_EQ(startingPosition, std::round(radianToDegree(position.second)));
}

TEST_F(PanTiltShould, getPositionWithoutInitialization) {
    panTilt_.reset(new crf::devices::pantilt::DynamixelPanTilt(dynamixelMock_));
    std::pair<double, double> position = panTilt_->getPosition();
    ASSERT_EQ(-1, position.first);
    ASSERT_EQ(-1, position.second);
}

TEST_F(PanTiltShould, setNegativePosition) {
    std::pair<double, double> position = {-2, -3};
    std::pair<double, double> givenPosition;
    auto degreeToRadian = [this](uint16_t position) {
        return (position * M_PI * dxlJSON_.at("Unit_degree").get<float>()) / 180;
    };
    EXPECT_CALL(*dynamixelMock_, writeDynamixel(_, _)).WillOnce(Return(true)).
        WillOnce(Return(true)).WillOnce(Invoke(
        [&givenPosition, &degreeToRadian](const uint8_t& index, const uint16_t& goalPosition) {
            givenPosition.first = goalPosition;
            return true;
        })).WillOnce(Invoke(
        [&givenPosition, &degreeToRadian](const uint8_t & index, const uint16_t & goalPosition) {
            givenPosition.second = goalPosition;
            return true;
        }));

    panTilt_.reset(new crf::devices::pantilt::DynamixelPanTilt(dynamixelMock_));
    ASSERT_TRUE(panTilt_->initialize());
    ASSERT_TRUE(panTilt_->setPosition(position));
    ASSERT_TRUE(panTilt_->deinitialize());
    ASSERT_EQ(zeroPosition_.at(0), std::round(givenPosition.first * 10) / 10.0f);
    ASSERT_EQ(zeroPosition_.at(1), std::round(givenPosition.second * 10) / 10.0f);
}
