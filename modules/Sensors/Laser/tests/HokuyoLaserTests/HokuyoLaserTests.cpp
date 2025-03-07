/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Julia Kabalar CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <string>
#include <memory>
#include <vector>

#include "Laser/Hokuyo/HokuyoLaser.hpp"
#include "Laser/LaserConfiguration.hpp"

using crf::sensors::laser::HokuyoLaser;
using crf::sensors::laser::HokuyoConnectionType;

class HokuyoLaserShould: public ::testing::Test {
 protected:
    HokuyoLaserShould(): logger_("HokuyoLaserShould") {
        logger_->info("{} BEGIN", testing::UnitTest::GetInstance()->current_test_info()->name());
        config_ = std::make_shared<crf::sensors::laser::LaserConfiguration>();
        /*
         * The URG04LXUG01's default communication settings are:
         *   - Device name: "/dev/ttyACM0"
         *   - Standard baudrate: 115200
         */
        sut_.reset(new HokuyoLaser(HokuyoConnectionType::SERIAL, "/dev/ttyACM0", 115200));

        /*
         * The UST20LX's default (factory) IP settings are:
         *   - IP address: 192.168.0.10
         *   - Network mask: 255.255.255.0
         *   - Gateway: 192.168.0.1
         *   - Port: 10940
         */
        // sut_.reset(new HokuyoLaser(HokuyoConnectionType::ETHERNET, "192.168.0.10", 10940));
    }
    ~HokuyoLaserShould() {
        logger_->info("{} END with {}",
            testing::UnitTest::GetInstance()->current_test_info()->name(),
            testing::UnitTest::GetInstance()->current_test_info()->result()->Passed());
    }
    crf::utility::logger::EventLogger logger_;
    std::shared_ptr<crf::sensors::laser::LaserConfiguration> config_;
    std::unique_ptr<crf::sensors::laser::ILaser> sut_;
};

TEST_F(HokuyoLaserShould, DISABLED_returnFalseIfInitializedOrDeinitializedTwice) {
    ASSERT_TRUE(sut_->initialize());
    ASSERT_FALSE(sut_->initialize());
    ASSERT_TRUE(sut_->deinitialize());
    ASSERT_FALSE(sut_->deinitialize());
}

TEST_F(HokuyoLaserShould, returnEmptyScanIfNotInitialized) {
    ASSERT_EQ(nullptr, sut_->getPointCloud());
}

TEST_F(HokuyoLaserShould, DISABLED_returnCorrectScanAfterInitialized) {
    ASSERT_TRUE(sut_->initialize());
    ASSERT_EQ(sut_->getConfiguration()->getLaserParameters().scanSize,
        config_->getLaserParameters().scanSize);
    auto pointCloud = sut_->getPointCloud();
    ASSERT_TRUE(pointCloud);
    ASSERT_GT(pointCloud->size(), 0);
    ASSERT_TRUE(pointCloud->size() < config_->getLaserParameters().scanSize);
}
