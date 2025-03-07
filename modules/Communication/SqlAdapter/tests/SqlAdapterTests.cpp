/* Copyright 2017 CERN */

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <memory>
#include <string>
#include <vector>

#include "EventLogger/EventLogger.hpp"
#include "SqlAdapter/SqlResult.hpp"
#include "SqlAdapter/MicrosoftSqlServerAdapter.hpp"

#include "Mocks/Communication/OdbcInterfaceMock.hpp"

using crf::communication::sqladapter::SqlResult;
using crf::communication::sqladapter::ISqlAdapter;
using crf::communication::sqladapter::MicrosoftSqlServerAdapter;

using crf::communication::sqladapter::OdbcInterfaceMock;

using testing::_;
using testing::DoAll;
using testing::DoDefault;
using testing::Invoke;
using testing::NiceMock;
using testing::Return;
using testing::SetArgPointee;

class SqlAdapterShould: public ::testing::Test {
 protected:
    SqlAdapterShould(): logger_("SqlAdapterShould") {
        logger_->info("{0} BEGIN",
            testing::UnitTest::GetInstance()->current_test_info()->name());
        odbcInterfaceMock_.reset(new NiceMock<OdbcInterfaceMock>);
        sut_.reset(new MicrosoftSqlServerAdapter(
            "PavlosTestSql", "skoupras", "rawRobot2018", odbcInterfaceMock_));
    }

    void SetUp() override {
        ON_CALL(*odbcInterfaceMock_, SQLAllocHandle(_, _, _)).WillByDefault(Invoke(
            [](SQLSMALLINT HandleType, SQLHANDLE InputHandle, SQLHANDLE* OutputHandlePtr) {
                if (*OutputHandlePtr || (HandleType != SQL_HANDLE_ENV && InputHandle == nullptr)) {
                    return SQL_ERROR;
                }
                *OutputHandlePtr = reinterpret_cast<void*>(1);
                return SQL_SUCCESS;
            }));
        ON_CALL(*odbcInterfaceMock_, SQLSetEnvAttr(_, _, _, _)).WillByDefault(Return(SQL_SUCCESS));
        ON_CALL(*odbcInterfaceMock_, SQLConnect(_, _, _, _, _, _, _))
            .WillByDefault(Return(SQL_SUCCESS));
        ON_CALL(*odbcInterfaceMock_, SQLDisconnect(_)).WillByDefault(Invoke(
            [](SQLHDBC ConnectionHandle) {
                if (!ConnectionHandle) {
                    return SQL_ERROR;
                }
                return SQL_SUCCESS;
            }));
        ON_CALL(*odbcInterfaceMock_, SQLFreeHandle(_, _)).WillByDefault(Invoke(
            [](SQLSMALLINT HandleType, SQLHANDLE Handle) {
                if (!Handle) {
                    return SQL_ERROR;
                }
                return SQL_SUCCESS;
            }));
    }

    ~SqlAdapterShould() {
        logger_->info("{0} END with {1}",
            testing::UnitTest::GetInstance()->current_test_info()->name(),
            testing::UnitTest::GetInstance()->current_test_info()->result()->Passed());
    }
    crf::utility::logger::EventLogger logger_;
    std::shared_ptr<NiceMock<OdbcInterfaceMock> > odbcInterfaceMock_;
    std::unique_ptr<ISqlAdapter> sut_;
};

TEST_F(SqlAdapterShould, returnTrueWhenConnectDisconnectAndFalseWhenConnectDisconnectTwice) {
    ASSERT_TRUE(sut_->connect());
    ASSERT_FALSE(sut_->connect());
    ASSERT_TRUE(sut_->disconnect());
    ASSERT_FALSE(sut_->disconnect());

    ASSERT_TRUE(sut_->connect());
    ASSERT_FALSE(sut_->connect());
    ASSERT_TRUE(sut_->disconnect());
    ASSERT_FALSE(sut_->disconnect());
}

TEST_F(SqlAdapterShould, returnFalseIfConnectOrDisconnectOperationFails) {
    EXPECT_CALL(*odbcInterfaceMock_, SQLConnect(_, _, _, _, _, _, _))
        .WillOnce(Return(SQL_ERROR)).WillRepeatedly(DoDefault());
    EXPECT_CALL(*odbcInterfaceMock_, SQLDisconnect(_))
        .WillOnce(Return(SQL_ERROR)).WillRepeatedly(DoDefault());
    ASSERT_FALSE(sut_->connect());
    // second attempt also false, because after failed attempt disconnect is required first
    ASSERT_FALSE(sut_->connect());
    // failed disconnect, but resources free anyway
    ASSERT_FALSE(sut_->disconnect());
    ASSERT_TRUE(sut_->connect());
    ASSERT_TRUE(sut_->disconnect());
}

TEST_F(SqlAdapterShould, returnEmptyResultIfNotPreviouslyConnected) {
    SqlResult result = sut_->executeQuery("SELECT TOP 2 SampleID, MotherID FROM SampleTable");
    ASSERT_EQ(0, result.numRows());
}

TEST_F(SqlAdapterShould, returnEmptyResultIfSomeOfDataFetchOperationsFail) {
    int numCols = 2;
    std::string query("SELECT TOP 2 SampleID, MotherID FROM SampleTable");
    SqlResult result;
    EXPECT_CALL(*odbcInterfaceMock_, SQLFetchScroll(_, _, _))
        .WillRepeatedly(Return(SQL_SUCCESS));
    ASSERT_TRUE(sut_->connect());
    EXPECT_CALL(*odbcInterfaceMock_, SQLExecDirect(_, _, _))
        .WillOnce(Return(SQL_ERROR)).WillRepeatedly(Return(SQL_SUCCESS));
    result = sut_->executeQuery(query);
    ASSERT_EQ(0, result.numRows());
    EXPECT_CALL(*odbcInterfaceMock_, SQLNumResultCols(_, _))
        .WillOnce(Return(SQL_ERROR))
        .WillRepeatedly(DoAll(SetArgPointee<1>(numCols), Return(SQL_SUCCESS)));
    result = sut_->executeQuery(query);
    ASSERT_EQ(0, result.numRows());
    EXPECT_CALL(*odbcInterfaceMock_, SQLColAttribute(_, _, _, _, _, _, _))
        .WillOnce(Return(SQL_ERROR))
        .WillRepeatedly(Return(SQL_SUCCESS));
    result = sut_->executeQuery(query);
    ASSERT_EQ(0, result.numRows());
    EXPECT_CALL(*odbcInterfaceMock_, SQLBindCol(_, _, _, _, _, _))
        .WillOnce(Return(SQL_ERROR)).WillRepeatedly(Return(SQL_SUCCESS));
    result = sut_->executeQuery(query);
    ASSERT_EQ(0, result.numRows());
}

TEST_F(SqlAdapterShould, returnCorrectResultFromExecutedQuery) {
    std::vector<std::string> columns = {
        "SampleID",
        "MotherID",
        "X_cm"
    };
    int numCols = columns.size();
    int numRows = 10;
    int fetchIteration = 0;
    std::string query("SELECT TOP " + std::to_string(numRows) + " ");
    for (const auto& column : columns) {
        query += column;
        query += ", ";
    }
    *(--(--query.end())) = ' ';
    query += " FROM SampleTable";
    std::vector<std::string> columnsSourceData;
    for (int i = 0; i < numCols; i++) {
        columnsSourceData.push_back("ColData" + std::to_string(i));
    }
    std::vector<char*> columnsTargetData(numCols);
    EXPECT_CALL(*odbcInterfaceMock_, SQLExecDirect(_, _, _))
        .WillRepeatedly(Return(SQL_SUCCESS));
    EXPECT_CALL(*odbcInterfaceMock_, SQLNumResultCols(_, _))
        .WillRepeatedly(DoAll(SetArgPointee<1>(numCols), Return(SQL_SUCCESS)));
    EXPECT_CALL(*odbcInterfaceMock_, SQLColAttribute(_, _, _, _, _, _, _))
        .WillRepeatedly(Invoke([columns]
            (SQLHSTMT, SQLUSMALLINT ColumnNumber, SQLUSMALLINT,
                SQLPOINTER CharacterAttributePtr, SQLSMALLINT, SQLSMALLINT*, SQLLEN*) {
            std::strcpy(reinterpret_cast<char*>(CharacterAttributePtr), columns[ColumnNumber - 1].c_str());  // NOLINT
            return SQL_SUCCESS;
        }));
    EXPECT_CALL(*odbcInterfaceMock_, SQLBindCol(_, _, _, _, _, _))
        .WillRepeatedly(Invoke(
            [&columnsTargetData](SQLHSTMT, SQLUSMALLINT ColumnNumber, SQLSMALLINT,
                SQLPOINTER TargetValuePtr, SQLLEN, SQLLEN*) {
                columnsTargetData[ColumnNumber - 1] = reinterpret_cast<char*>(TargetValuePtr);
                return SQL_SUCCESS;
            }));
    EXPECT_CALL(*odbcInterfaceMock_, SQLFetchScroll(_, _, _))
        .WillRepeatedly(Invoke([&columnsTargetData, columnsSourceData, &fetchIteration, numRows]
            (SQLHSTMT, SQLSMALLINT, SQLLEN) {
                fetchIteration++;
                if (fetchIteration > numRows) {
                    return SQL_NO_DATA;
                }
                for (int i = 0; i < columnsTargetData.size(); i++) {
                    std::strcpy(columnsTargetData[i], columnsSourceData[i].c_str()); // NOLINT
                }
                return SQL_SUCCESS;
            }));
    ASSERT_TRUE(sut_->connect());
    SqlResult result = sut_->executeQuery(query);
    ASSERT_EQ(numRows, result.numRows());
    for (const auto& row : result) {
        for (int i = 0; i < numCols; i++) {
            ASSERT_EQ(columnsSourceData[i], row[columns[i]]);
        }
    }
}

TEST_F(SqlAdapterShould, DISABLED_liveDemoWithDatabase) {
    sut_.reset(new MicrosoftSqlServerAdapter(
            "PavlosTestSql", "skoupras", "rawRobot2018"));
    std::vector<std::string> columns = {
        "SampleID",
        "MotherID",
        "X_cm",
        "Y_cm",
        "Z_cm"
    };
    int numCols = columns.size();
    int numRows = 100;
    int fetchIteration = 0;
    std::string query("SELECT TOP " + std::to_string(numRows) + " ");
    for (const auto& column : columns) {
        query += column;
        query += ", ";
    }
    *(--(--query.end())) = ' ';
    query += " FROM SampleTable";
    ASSERT_TRUE(sut_->connect());
    SqlResult result = sut_->executeQuery(query);
    ASSERT_EQ(numRows, result.numRows());
    for (const auto& row : result) {
        for (int i = 0; i < numCols; i++) {
            try {
                logger_->info("{} = {}", columns[i], row[columns[i]].get<std::string>());
            } catch (const std::exception& e) {
                logger_->error("Exception: {}", e.what());
                ASSERT_TRUE(false);
            }
        }
    }
}
