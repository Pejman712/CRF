/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license. 
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 * 
 * Author: Giacomo Lunghi CERN EN/STI/ECE
 * 
 *  ==================================================================================================
 */

#include <curlpp/cURLpp.hpp>
#include <curlpp/Options.hpp>
#include <memory>
#include <nlohmann/json.hpp>
#include <random>
#include <string>
#include <vector>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "EventLogger/EventLogger.hpp"
#include "WebServers/TIM/TIMWebPoint.hpp"
#include "WebServers/TIMWebServer.hpp"
#include "Mocks/Communication/TIMMock.hpp"

using testing::_;
using testing::Return;
using testing::Invoke;
using testing::NiceMock;

using crf::robots::tim::TIMMock;
using crf::communication::webservers::TIMWebPoint;
using crf::communication::webservers::TIMWebServer;

class TIMWebServerShould : public ::testing::Test {
 protected:
    TIMWebServerShould() :
        logger_("TIMWebServerShould"),
        timInitialized_(false) {
            logger_->info("{} BEGIN",
            testing::UnitTest::GetInstance()->current_test_info()->name());

            tim_.reset(new NiceMock<TIMMock>);
            webPoint_.reset(new TIMWebPoint(tim_,
                std::chrono::milliseconds(1),
                std::chrono::milliseconds(1)));

            testDirName_ = __FILE__;
            testDirName_ = testDirName_.substr(0,
                testDirName_.find("TIMWebServerTests.cpp"));
            testDirName_ += "configuration/";
    }

    void SetUp() override {
        ON_CALL(*tim_, initialize()).WillByDefault(Invoke([this]() {
            timInitialized_ = true;
            return true;
        }));
        ON_CALL(*tim_, deinitialize()).WillByDefault(Invoke([this]() {
            timInitialized_ = false;
            return true;
        }));
        ON_CALL(*tim_, isConnected()).WillByDefault(Invoke([this]() {
            return timInitialized_;
        }));

        ON_CALL(*tim_, isMoving()).WillByDefault(Return(true));
        ON_CALL(*tim_, economyModeActive()).WillByDefault(Return(false));
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

    ~TIMWebServerShould() {
        logger_->info("{} END with {}",
            testing::UnitTest::GetInstance()->current_test_info()->name(),
            testing::UnitTest::GetInstance()->current_test_info()->result()->Passed());
    }

    crf::utility::logger::EventLogger logger_;
    std::shared_ptr<TIMMock> tim_;
    std::shared_ptr<TIMWebPoint> webPoint_;
    std::unique_ptr<TIMWebServer> sut_;
    std::string testDirName_;

    bool timInitialized_;
};

TEST_F(TIMWebServerShould, throwExceptionOnWrongConfigFile) {
    std::string configFilename = testDirName_ + "exceptionConfig.json";

    ASSERT_THROW(sut_.reset(new TIMWebServer(configFilename)), std::invalid_argument);
}

TEST_F(TIMWebServerShould, initailizeDeinitializeSequenec) {
    std::string configFilename = testDirName_ + "TIMServerConfig.json";

    ASSERT_NO_THROW(sut_.reset(new TIMWebServer(configFilename)));
    ASSERT_FALSE(sut_->initialize());
    ASSERT_FALSE(sut_->deinitialize());

    ASSERT_TRUE(sut_->addTIM(4, webPoint_));
    ASSERT_FALSE(sut_->addTIM(4, webPoint_));

    ASSERT_TRUE(sut_->initialize());
    ASSERT_FALSE(sut_->initialize());
    ASSERT_TRUE(sut_->deinitialize());
    ASSERT_FALSE(sut_->deinitialize());

    ASSERT_TRUE(sut_->initialize());
    ASSERT_FALSE(sut_->initialize());
    ASSERT_TRUE(sut_->deinitialize());
    ASSERT_FALSE(sut_->deinitialize());
}

TEST_F(TIMWebServerShould, correctlyGetStatus) {
    std::string configFilename = testDirName_ + "TIMServerConfig.json";

    ASSERT_NO_THROW(sut_.reset(new TIMWebServer(configFilename)));
    ASSERT_TRUE(sut_->addTIM(4, webPoint_));
    ASSERT_TRUE(sut_->initialize());

    curlpp::Cleanup myCleanup;
    std::ostringstream os;
    os << curlpp::options::Url("http://localhost:8087/4/status");

    nlohmann::json dataJson;
    ASSERT_NO_THROW(dataJson = nlohmann::json::parse(os.str()));

    ASSERT_EQ(dataJson["status"].get<std::string>(), "online");
    ASSERT_NEAR(dataJson["position"].get<float>(), 3256, 1e-5);
    ASSERT_NEAR(dataJson["velocity"].get<float>(), 1.1, 1e-5);
    ASSERT_NEAR(dataJson["battery"].get<float>(), 25.6, 1e-5);
    ASSERT_EQ(dataJson["ischarging"].get<bool>(), false);
    ASSERT_EQ(dataJson["economymode"].get<bool>(), false);
    ASSERT_EQ(dataJson["ismoving"].get<bool>(), true);
    ASSERT_EQ(dataJson["payloadretracted"].get<bool>(), false);
    ASSERT_NEAR(dataJson["rpvalue"].get<float>(), 1.25, 1e-5);
    ASSERT_NEAR(dataJson["oxygen"].get<float>(), 20, 1e-5);
    ASSERT_NEAR(dataJson["temperature"].get<float>(), 19, 1e-5);

    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(TIMWebServerShould, correctlyGetHistory) {
    std::string configFilename = testDirName_ + "TIMServerConfig.json";

    ASSERT_NO_THROW(sut_.reset(new TIMWebServer(configFilename)));
    ASSERT_TRUE(sut_->addTIM(4, webPoint_));
    ASSERT_TRUE(sut_->initialize());

    // give enough time for history to fill
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    curlpp::Cleanup myCleanup;
    std::ostringstream os;
    os << curlpp::options::Url("http://localhost:8087/4/history");

    nlohmann::json dataJson;
    ASSERT_NO_THROW(dataJson = nlohmann::json::parse(os.str()));

    auto positions = dataJson["position"].get<std::vector<float> >();
    ASSERT_EQ(positions.size(), 100);

    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(TIMWebServerShould, failToGetDataIfWrongRequest) {
    std::string configFilename = testDirName_ + "TIMServerConfig.json";

    ASSERT_NO_THROW(sut_.reset(new TIMWebServer(configFilename)));
    ASSERT_TRUE(sut_->addTIM(4, webPoint_));
    ASSERT_TRUE(sut_->initialize());

    // give enough time for history to fill
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    curlpp::Cleanup myCleanup;
    std::ostringstream os;
    os << curlpp::options::Url("http://localhost:8087/5/status");
    ASSERT_EQ(os.str().length(), 0);

    os << curlpp::options::Url("http://localhost:8087/4/stats");
    ASSERT_EQ(os.str().length(), 0);

    os << curlpp::options::Url("http://localhost:8087/4/hstory");
    ASSERT_EQ(os.str().length(), 0);

    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(TIMWebServerShould, correctlyReturnsAllStatuses) {
    std::string configFilename = testDirName_ + "TIMServerConfig.json";

    ASSERT_NO_THROW(sut_.reset(new TIMWebServer(configFilename)));
    ASSERT_TRUE(sut_->addTIM(4, webPoint_));
    ASSERT_TRUE(sut_->initialize());

    curlpp::Cleanup myCleanup;
    std::ostringstream os;
    os << curlpp::options::Url("http://localhost:8087/list");
    nlohmann::json dataJson;

    ASSERT_NO_THROW(dataJson = nlohmann::json::parse(os.str()));
    auto timsList = dataJson.at("tims").get<std::vector<int> >();
    ASSERT_EQ(timsList.size(), 1);
    ASSERT_EQ(timsList[0], 4);

    std::ostringstream os_status;
    os_status << curlpp::options::Url("http://localhost:8087/all/status");
    ASSERT_NO_THROW(dataJson = nlohmann::json::parse(os_status.str()));

    ASSERT_EQ(dataJson.at("4")["status"], "online");
    ASSERT_NEAR(dataJson.at("4")["position"].get<float>(), 3256, 1e-5);
    ASSERT_NEAR(dataJson.at("4")["velocity"].get<float>(), 1.1, 1e-5);
    ASSERT_NEAR(dataJson.at("4")["battery"].get<float>(), 25.6, 1e-5);
    ASSERT_EQ(dataJson.at("4")["ischarging"].get<bool>(), false);
    ASSERT_EQ(dataJson.at("4")["economymode"].get<bool>(), false);
    ASSERT_EQ(dataJson.at("4")["ismoving"].get<bool>(), true);
    ASSERT_EQ(dataJson.at("4")["payloadretracted"].get<bool>(), false);
    ASSERT_NEAR(dataJson.at("4")["rpvalue"].get<float>(), 1.25, 1e-5);
    ASSERT_NEAR(dataJson.at("4")["oxygen"].get<float>(), 20, 1e-5);
    ASSERT_NEAR(dataJson.at("4")["temperature"].get<float>(), 19, 1e-5);

    ASSERT_TRUE(sut_->deinitialize());
}
