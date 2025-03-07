/* © Copyright CERN 2021.  All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Jorge Playán Garai CERN BE/CEM/MRO
 *
 *  ==================================================================================================
*/

#include <gtest/gtest.h>

#include "crf/ResponseCode.hpp"
#include "EventLogger/EventLogger.hpp"

class ResponseCodeShould: public ::testing::Test {
 protected:
    ResponseCodeShould():
      logger_("ResponseCodeShould") {
        logger_->info("{} BEGIN",
            testing::UnitTest::GetInstance()->current_test_info()->name());
    }
    ~ResponseCodeShould() {
        logger_->info("{} END with {}",
            testing::UnitTest::GetInstance()->current_test_info()->name(),
            testing::UnitTest::GetInstance()->current_test_info()->result()->Passed());
    }

    crf::utility::logger::EventLogger logger_;
};

TEST_F(ResponseCodeShould, correctlyCompareResponses) {
    crf::ResponseCode sut1 = crf::ResponseCode(crf::Code::Empty);
    crf::ResponseCode sut2 = crf::ResponseCode(crf::Code::Empty);

    ASSERT_EQ(sut1, sut2);

    sut2 = crf::ResponseCode(crf::Code::OK);

    ASSERT_NE(sut1, sut2);
}

TEST_F(ResponseCodeShould, compileAndNotThrowWhenPrintingNumber) {
    crf::ResponseCode sut = crf::ResponseCode(crf::Code::Empty);

    std::cout << sut << std::endl;

    sut = crf::ResponseCode(crf::Code::OK);

    std::cout << sut << std::endl;
}

TEST_F(ResponseCodeShould, acceptSecondaryCodeAndThrowIfOneIsNotPresent) {
    crf::ResponseCode sut = crf::ResponseCode(crf::Code::OK);

    sut.detail(74);

    ASSERT_EQ(sut.detail(), 74);

    std::cout << sut << std::endl;
}

TEST_F(ResponseCodeShould, constructWithSecondaryCodeCorrectly) {
    crf::ResponseCode sut = crf::ResponseCode(crf::Code::Empty, 32);

    ASSERT_NE(sut.detail(), 43);
    ASSERT_EQ(sut.detail(), 32);

    std::cout << sut << std::endl;
}

TEST_F(ResponseCodeShould, useTheOperatorsOfResponseCodeCorrectlyWhenHavingASecondaryCode) {
    {
        crf::ResponseCode sut1 = crf::ResponseCode(crf::Code::Empty, 32);
        crf::ResponseCode sut2 = crf::ResponseCode(crf::Code::Empty, 32);

        ASSERT_TRUE(sut1 == sut2);
    }

    {
        crf::ResponseCode sut1 = crf::ResponseCode(crf::Code::Empty, 42);
        crf::ResponseCode sut2 = crf::ResponseCode(crf::Code::Empty, 32);

        ASSERT_TRUE(sut1 != sut2);
    }

    {
        crf::ResponseCode sut1 = crf::ResponseCode(crf::Code::Empty);
        crf::ResponseCode sut2 = crf::ResponseCode(crf::Code::Empty, 32);

        ASSERT_TRUE(sut1 != sut2);
    }

    {
        crf::ResponseCode sut1 = crf::ResponseCode(crf::Code::Empty);
        crf::ResponseCode sut2 = crf::ResponseCode(crf::Code::Empty);

        ASSERT_TRUE(sut1 == sut2);
    }
}
