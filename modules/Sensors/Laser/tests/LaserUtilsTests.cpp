/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license. 
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 * 
 * Author: Julia Kabalar CERN EN/SMM/MRO
 * 
 *  ==================================================================================================
 */

#include <array>

#include <gmock/gmock.h>
#include <gtest/gtest.h>


#include "EventLogger/EventLogger.hpp"
#include "Laser/LaserUtils.hpp"

using testing::_;
using testing::Return;

using crf::sensors::laser::LaserUtils;

class LaserUtilsShould: public ::testing::Test {
 protected:
    LaserUtilsShould():
    logger_("LaserUtilsShould") {
        logger_->info("{} BEGIN", testing::UnitTest::GetInstance()->current_test_info()->name());
    }
    ~LaserUtilsShould() {
        logger_->info("{} END with {}",
            testing::UnitTest::GetInstance()->current_test_info()->name(),
            testing::UnitTest::GetInstance()->current_test_info()->result()->Passed());
    }
    crf::utility::logger::EventLogger logger_;
};

TEST_F(LaserUtilsShould, convertCorrectly2Dscan) {
    float range = 4.0;
    float theta = 1.4;
    std::array<float, 2> dataPoint2D = {range, theta};
    auto xyArray = crf::sensors::laser::LaserUtils::SphericalToCartesian(dataPoint2D[0],
        dataPoint2D[1]);
    auto rangeThetaArray = crf::sensors::laser::LaserUtils::CartesianToSpherical(xyArray[0],
        xyArray[1]);
    for (int i = 0; i < dataPoint2D.size(); i++) {
        ASSERT_FLOAT_EQ(dataPoint2D[i], rangeThetaArray[i]);
    }
    ASSERT_FLOAT_EQ(rangeThetaArray[2], .0);
}

TEST_F(LaserUtilsShould, returnZeroOnNotRadianValues) {
    float r = 4.0;
    auto xyzArray = crf::sensors::laser::LaserUtils::SphericalToCartesian(r, 123, .0);
    for (int i = 0; i < xyzArray.size(); i++) {
        ASSERT_FLOAT_EQ(xyzArray[i], .0);
    }
    xyzArray = crf::sensors::laser::LaserUtils::SphericalToCartesian(r, -3.15);
    for (int i = 0; i < xyzArray.size(); i++) {
        ASSERT_FLOAT_EQ(xyzArray[i], .0);
    }
}

TEST_F(LaserUtilsShould, convertCorrectly3Dscan) {
    float radius = 4.0;
    float azimuth = 1.4;
    float polar = -1.4;
    std::array<float, 3> dataPoint3D = {radius, azimuth, polar};
    auto xyArray = crf::sensors::laser::LaserUtils::SphericalToCartesian(
        dataPoint3D[0], dataPoint3D[1], dataPoint3D[2]);
    auto radiusAzimuthPolarArray = crf::sensors::laser::LaserUtils::CartesianToSpherical(
        xyArray[0], xyArray[1], xyArray[2]);
    for (int i = 0; i < radiusAzimuthPolarArray.size(); i++) {
        ASSERT_FLOAT_EQ(radiusAzimuthPolarArray[i], radiusAzimuthPolarArray[i]);
    }
}
