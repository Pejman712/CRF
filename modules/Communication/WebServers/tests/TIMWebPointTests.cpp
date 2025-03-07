/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license. 
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 * 
 * Author: Giacomo Lunghi CERN EN/STI/ECE
 * 
 *  ==================================================================================================
 */

#include <memory>
#include <nlohmann/json.hpp>
#include <random>
#include <string>
#include <vector>

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <boost/optional.hpp>

#include "EventLogger/EventLogger.hpp"
#include "WebServers/TIM/TIMWebPoint.hpp"
#include "Mocks/Communication/TIMMock.hpp"

#define CV_TIMEOUT std::chrono::milliseconds(500)

using testing::_;
using testing::Return;
using testing::Invoke;
using testing::NiceMock;

using crf::robots::tim::TIMMock;
using crf::communication::webservers::TIMWebPoint;

class TIMWebPointShould : public ::testing::Test {
 protected:
    TIMWebPointShould() :
        logger_("TIMWebPointShould"),
        timInitialized_(false) {
            logger_->info("{} BEGIN",
            testing::UnitTest::GetInstance()->current_test_info()->name());

            tim_.reset(new NiceMock<TIMMock>);
            connected_ = true;
    }

    void SetUp() override {
        connected_ = true;
        ON_CALL(*tim_, initialize()).WillByDefault(Invoke([this]() {
            if (connected_) {
                std::unique_lock<std::mutex> lock(initializeMutex_);
                timInitialized_ = true;
                initializeCv_.notify_all();
                return true;
            }
            return false;
        }));
        ON_CALL(*tim_, deinitialize()).WillByDefault(Invoke([this]() {
            if (connected_) {
                std::unique_lock<std::mutex> lock(initializeMutex_);
                timInitialized_ = false;
                initializeCv_.notify_all();
                return true;
            }
            return false;
        }));
        ON_CALL(*tim_, isConnected()).WillByDefault(Invoke([this]() {
            if (connected_) {
                return timInitialized_;
            }
            return false;
        }));
        ON_CALL(*tim_, isMoving()).WillByDefault(Return(true));
        ON_CALL(*tim_, economyModeActive()).WillByDefault(Invoke([this]() {
            if (connected_) {
                return boost::optional<bool>(false);
            }
            return boost::optional<bool>(boost::none);;
        }));
        ON_CALL(*tim_, isCharging()).WillByDefault(Return(false));
        ON_CALL(*tim_, allPayloadRetracted()).WillByDefault(Return(false));
        ON_CALL(*tim_, getBatteryVoltage()).WillByDefault(Return(25.6));
        ON_CALL(*tim_, getVelocity()).WillByDefault(Return(1.1f));
        ON_CALL(*tim_, getPosition()).WillByDefault(Return(3256));
        ON_CALL(*tim_, getVelocity()).WillByDefault(Return(1.1f));
        ON_CALL(*tim_, getRPValue()).WillByDefault(Return(1.25));
        ON_CALL(*tim_, getOxygenPercentage()).WillByDefault(Return(20));
        ON_CALL(*tim_, getTemperature()).WillByDefault(Return(19));
    }

    ~TIMWebPointShould() {
        logger_->info("{} END with {}",
            testing::UnitTest::GetInstance()->current_test_info()->name(),
            testing::UnitTest::GetInstance()->current_test_info()->result()->Passed());
    }

    crf::utility::logger::EventLogger logger_;
    std::shared_ptr<TIMMock> tim_;
    std::unique_ptr<TIMWebPoint> sut_;

    std::mutex initializeMutex_;
    std::condition_variable initializeCv_;

    bool timInitialized_;
    bool connected_;
};

TEST_F(TIMWebPointShould, initializeDeinitializeSequence) {
    connected_ = true;
    sut_.reset(new TIMWebPoint(tim_));

    ASSERT_TRUE(sut_->initialize());
    ASSERT_FALSE(sut_->initialize());
    ASSERT_TRUE(sut_->deinitialize());
    ASSERT_FALSE(sut_->deinitialize());

    ASSERT_TRUE(sut_->initialize());
    ASSERT_FALSE(sut_->initialize());
    ASSERT_TRUE(sut_->deinitialize());
    ASSERT_FALSE(sut_->deinitialize());
}

TEST_F(TIMWebPointShould, correctlyGetStatus) {
    connected_ = true;
    sut_.reset(new TIMWebPoint(tim_, std::chrono::milliseconds(1), std::chrono::milliseconds(1)));

    {
        std::unique_lock<std::mutex> lock(initializeMutex_);
        ASSERT_TRUE(sut_->initialize());
        ASSERT_TRUE(initializeCv_.wait_for(lock, CV_TIMEOUT, [this](){
            return timInitialized_;
        }));
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    auto statusString = sut_->getStatus();
    nlohmann::json json_ = nlohmann::json::parse(statusString);
    ASSERT_EQ(json_["status"].get<std::string>(), "online");
    ASSERT_NEAR(json_["position"].get<float>(), 3256, 1e-5);
    ASSERT_NEAR(json_["velocity"].get<float>(), 1.1, 1e-5);
    ASSERT_NEAR(json_["battery"].get<float>(), 25.6, 1e-5);
    ASSERT_EQ(json_["ischarging"].get<bool>(), false);
    ASSERT_EQ(json_["economymode"].get<bool>(), false);
    ASSERT_EQ(json_["ismoving"].get<bool>(), true);
    ASSERT_EQ(json_["payloadretracted"].get<bool>(), false);
    ASSERT_NEAR(json_["rpvalue"].get<float>(), 1.25, 1e-5);
    ASSERT_NEAR(json_["oxygen"].get<float>(), 20, 1e-5);
    ASSERT_NEAR(json_["temperature"].get<float>(), 19, 1e-5);

    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(TIMWebPointShould, returnsOfflineIfConnectionIsLost) {
    connected_ = true;
    sut_.reset(new TIMWebPoint(tim_, std::chrono::milliseconds(1), std::chrono::milliseconds(1)));

    {
        std::unique_lock<std::mutex> lock(initializeMutex_);
        ASSERT_TRUE(sut_->initialize());
        ASSERT_TRUE(initializeCv_.wait_for(lock, CV_TIMEOUT, [this](){
            return timInitialized_;
        }));
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    nlohmann::json json_ = nlohmann::json::parse(sut_->getStatus());
    ASSERT_EQ(json_["status"].get<std::string>(), "online");

    connected_ = false;
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    json_ = nlohmann::json::parse(sut_->getStatus());
    ASSERT_EQ(json_["status"].get<std::string>(), "offline");

    ASSERT_FALSE(sut_->deinitialize());
}

TEST_F(TIMWebPointShould, triesToReconnectIfConnectionIsLost) {
    connected_ = true;
    sut_.reset(new TIMWebPoint(tim_, std::chrono::milliseconds(1), std::chrono::milliseconds(1)));

    {
        std::unique_lock<std::mutex> lock(initializeMutex_);
        ASSERT_TRUE(sut_->initialize());
        ASSERT_TRUE(initializeCv_.wait_for(lock, CV_TIMEOUT, [this](){
            return timInitialized_;
        }));
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    nlohmann::json json_ = nlohmann::json::parse(sut_->getStatus());
    ASSERT_EQ(json_["status"].get<std::string>(), "online");

    connected_ = false;
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    json_ = nlohmann::json::parse(sut_->getStatus());
    ASSERT_EQ(json_["status"].get<std::string>(), "offline");

    // EXPECT_CALL(*tim_, initialize()).Times(testing::AtLeast(1));
    // std::this_thread::sleep_for(std::chrono::milliseconds(100));

    connected_ = true;
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    json_ = nlohmann::json::parse(sut_->getStatus());
    ASSERT_EQ(json_["status"].get<std::string>(), "online");

    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(TIMWebPointShould, correctlyGrowsAndShrinksHistoryIfTIMMoving) {
    bool moving = true;
    ON_CALL(*tim_, isMoving()).WillByDefault(Invoke([this, &moving]() {
        return moving;
    }));

    sut_.reset(new TIMWebPoint(tim_,
        std::chrono::milliseconds(1),
        std::chrono::milliseconds(1)));

    {
        std::unique_lock<std::mutex> lock(initializeMutex_);
        ASSERT_TRUE(sut_->initialize());
        ASSERT_TRUE(initializeCv_.wait_for(lock, CV_TIMEOUT, [this](){
            return timInitialized_;
        }));
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    nlohmann::json json_ = nlohmann::json::parse(sut_->getStatus());
    ASSERT_EQ(json_["status"].get<std::string>(), "online");

    // Give time for the history to fill
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    json_ = nlohmann::json::parse(sut_->getDataHistory());
    auto positions = json_["position"].get<std::vector<float> >();
    ASSERT_EQ(positions.size(), 100);

    moving = false;
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    json_ = nlohmann::json::parse(sut_->getDataHistory());
    positions = json_["position"].get<std::vector<float> >();
    ASSERT_EQ(positions.size(), 5);

    moving = true;
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    json_ = nlohmann::json::parse(sut_->getDataHistory());
    positions = json_["position"].get<std::vector<float> >();
    ASSERT_EQ(positions.size(), 100);

    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(TIMWebPointShould, goesInErrorStateIfIsConnectedButCantGetStatus) {
    ON_CALL(*tim_, getPosition()).WillByDefault(Return(boost::none));
    sut_.reset(new TIMWebPoint(tim_,
        std::chrono::milliseconds(1),
        std::chrono::milliseconds(1)));

    {
        std::unique_lock<std::mutex> lock(initializeMutex_);
        ASSERT_TRUE(sut_->initialize());
        ASSERT_TRUE(initializeCv_.wait_for(lock, CV_TIMEOUT, [this](){
            return timInitialized_;
        }));
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    nlohmann::json json_ = nlohmann::json::parse(sut_->getStatus());
    ASSERT_EQ(json_["status"].get<std::string>(), "error");

    ASSERT_TRUE(sut_->deinitialize());
}

