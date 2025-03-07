/* Copyright 2017 CERN */

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <memory>

#include "EventLogger/EventLogger.hpp"
#include "SqlAdapter/SqlResult.hpp"

using crf::communication::sqladapter::SqlResult;

class SqlResultShould: public ::testing::Test {
 protected:
    SqlResultShould(): logger_("SqlResultShould") {
        logger_->info("{0} BEGIN",
            testing::UnitTest::GetInstance()->current_test_info()->name());
    }

    crf::utility::logger::EventLogger logger_;
    SqlResult sut_;

    ~SqlResultShould() {
            logger_->info("{0} END with {1}",
                testing::UnitTest::GetInstance()->current_test_info()->name(),
                testing::UnitTest::GetInstance()->current_test_info()->result()->Passed());
    }
};

TEST_F(SqlResultShould, returnCorrectNumOfRowsAndCorrectRow) {
    nlohmann::json firstRow = {{"xxx", "yyy"}};
    nlohmann::json secondRow = {{"data1", 1}, {"data2", 2}, {"data3", 3}};
    nlohmann::json thirdRow = {{"failure", true}};
    ASSERT_EQ(0, sut_.numRows());
    ASSERT_TRUE(sut_.insertRow(firstRow));
    ASSERT_TRUE(sut_.insertRow(secondRow));
    ASSERT_TRUE(sut_.insertRow(thirdRow));
    ASSERT_TRUE(sut_.insertRow("{\"dummyData\": 0}"_json));
    ASSERT_EQ(4, sut_.numRows());
    ASSERT_EQ(secondRow, sut_.getRow(1));
}

TEST_F(SqlResultShould, throwWhenTryingToAccessRowOutOfRange) {
    EXPECT_ANY_THROW(sut_.getRow(0));
    ASSERT_TRUE(sut_.insertRow("{\"DummyData\": 1}"_json));
    ASSERT_TRUE(sut_.insertRow("{\"DummyData\": 2}"_json));
    EXPECT_ANY_THROW(sut_.getRow(2));
    EXPECT_ANY_THROW(sut_.getRow(-1));
    for (int i = 0; i < sut_.numRows(); i++) {
        EXPECT_NO_THROW(sut_.getRow(i));
    }
}

TEST_F(SqlResultShould, allowCorrectIterationInRangeLoop) {
    int times = 0;
    for (const auto& row : sut_) {
        times += row["DummyData"].get<int>();
    }
    ASSERT_EQ(0, times);
    ASSERT_TRUE(sut_.insertRow("{\"DummyData\": 1}"_json));
    ASSERT_TRUE(sut_.insertRow("{\"DummyData\": 2}"_json));
    for (const auto& row : sut_) {
        times += row["DummyData"].get<int>();
    }
    ASSERT_EQ(3, times);
}
