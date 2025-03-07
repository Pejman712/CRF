/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Giacomo Lunghi CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */


#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <memory>
#include <string>

#include "EventLogger/EventLogger.hpp"
#include "SqlAdapter/MySqlAdapter.hpp"
#include "SqlAdapter/SqlResult.hpp"

using crf::communication::sqladapter::SqlResult;
using crf::communication::sqladapter::MySqlAdapter;

using testing::_;
using testing::DoAll;
using testing::DoDefault;
using testing::Invoke;
using testing::NiceMock;
using testing::Return;
using testing::SetArgPointee;

class MySqlAdapterShould : public ::testing::Test {
 protected:
    MySqlAdapterShould() :
        logger_("MySqlAdapterShould"),
        dbAddress_("dbod-timmiss.cern.ch"),
        username_("Tim_read"),
        password_("timdataread"),
        dbName_("TIM_DB"),
        port_(5500),
        sut_(new MySqlAdapter(dbAddress_, username_, password_, dbName_, port_)) {
            logger_->info("{} BEGIN",
                testing::UnitTest::GetInstance()->current_test_info()->name());
    }

    ~MySqlAdapterShould() {
        logger_->info("{} END with {}",
            testing::UnitTest::GetInstance()->current_test_info()->name(),
            testing::UnitTest::GetInstance()->current_test_info()->result()->Passed());
    }

    crf::utility::logger::EventLogger logger_;
    const std::string dbAddress_;
    const std::string username_;
    const std::string password_;
    const std::string dbName_;
    const int port_;

    std::unique_ptr<MySqlAdapter> sut_;
};

TEST_F(MySqlAdapterShould, DISABLED_correctlyConnectAndDisconnectMultipleTimes) {
    ASSERT_TRUE(sut_->connect());
    ASSERT_FALSE(sut_->connect());
    ASSERT_TRUE(sut_->disconnect());
    ASSERT_FALSE(sut_->disconnect());

    ASSERT_TRUE(sut_->connect());
    ASSERT_FALSE(sut_->connect());
    ASSERT_TRUE(sut_->disconnect());
    ASSERT_FALSE(sut_->disconnect());
}

TEST_F(MySqlAdapterShould, DISABLED_correctlyHandleMultipleClients) {
    {
        std::unique_ptr<MySqlAdapter> sut2(
            new MySqlAdapter(dbAddress_, username_, password_, dbName_, port_));

        ASSERT_TRUE(sut_->connect());
        ASSERT_TRUE(sut2->connect());
        ASSERT_FALSE(sut_->connect());
        ASSERT_FALSE(sut2->connect());
        ASSERT_TRUE(sut_->disconnect());
        ASSERT_FALSE(sut_->disconnect());
        // We also forget on purpose to disconnect sut2,
        // to see if the destructor takes care of it
    }

    ASSERT_TRUE(sut_->connect());
    ASSERT_FALSE(sut_->connect());
    ASSERT_TRUE(sut_->disconnect());
    ASSERT_FALSE(sut_->disconnect());
}

TEST_F(MySqlAdapterShould, DISABLED_correctlyQueryData) {
    ASSERT_TRUE(sut_->connect());
    auto result = sut_->executeQuery("SELECT * FROM TIM_DB.tim_missions WHERE ID_Mission=2;");
    ASSERT_EQ(result.numRows(), 1);
    ASSERT_EQ(result.getResult(), SqlResult::Result::Succeded);
    ASSERT_TRUE(sut_->disconnect());
}

TEST_F(MySqlAdapterShould, failsQueryIfNotConnected) {
    auto result = sut_->executeQuery("SELECT * FROM TIM_DB.tim_missions WHERE ID_Mission=2;");
    ASSERT_EQ(result.numRows(), 0);
    ASSERT_EQ(result.getResult(), SqlResult::Result::Failed);
}

TEST_F(MySqlAdapterShould, DISABLED_correctlyExecuteCommands) {
    ASSERT_TRUE(sut_->connect());
    auto result = sut_->executeQuery("START TRANSACTION;");
    ASSERT_EQ(result.numRows(), 0);
    ASSERT_EQ(result.getResult(), SqlResult::Result::Succeded);
    result = sut_->executeQuery("COMMIT;");
    ASSERT_EQ(result.numRows(), 0);
    ASSERT_EQ(result.getResult(), SqlResult::Result::Succeded);
    ASSERT_TRUE(sut_->disconnect());
}

