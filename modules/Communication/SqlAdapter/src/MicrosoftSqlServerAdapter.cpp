/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Giacomo Lunghi CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#include <stdlib.h>
#include <sstream>
#include <fstream>

#include <nlohmann/json.hpp>

#include "SqlAdapter/SqlResult.hpp"
#include "SqlAdapter/MicrosoftSqlServerAdapter.hpp"
#include "SqlAdapter/OdbcInterface.hpp"  // this include goes last on purpose

#include <sqlext.h>

#define MAX_BUFFER_SIZE 1024

namespace crf {
namespace communication {
namespace sqladapter {

MicrosoftSqlServerAdapter::MicrosoftSqlServerAdapter(const std::string& dsnName,
    const std::string& userId, const std::string& passwd,
    std::shared_ptr<IOdbcInterface> odbcInterface):
    logger_("MicrosoftSqlServerAdapter"),
    dsnName_(dsnName),
    userId_(userId),
    passwd_(passwd),
    odbcInterface_(odbcInterface),
    hdlEnv_(nullptr),
    hdlDbc_(nullptr) {
    logger_->debug("CTor");
    if (odbcInterface_ == nullptr) {
        logger_->info("Provided nullptr odbcInterface. Creating a default one");
        odbcInterface_.reset(new OdbcInterface);
    }
}

MicrosoftSqlServerAdapter::~MicrosoftSqlServerAdapter() {
    logger_->debug("DTor");
    disconnect();
}

bool MicrosoftSqlServerAdapter::connect() {
    logger_->info("connect");
    if (isConnected()) {
        logger_->warn("Previously connected. Disconnect first");
        return false;
    }
    SQLRETURN ret;
    ret = odbcInterface_->SQLAllocHandle(SQL_HANDLE_ENV, SQL_NULL_HANDLE, &hdlEnv_);
    if (!SQL_SUCCEEDED(ret)) {
        logger_->warn("Failed to allocate odbc env handle");
        return false;
    }
    ret = odbcInterface_->SQLSetEnvAttr(hdlEnv_, SQL_ATTR_ODBC_VERSION,
        (SQLPOINTER) SQL_OV_ODBC3, SQL_IS_UINTEGER);
    if (!SQL_SUCCEEDED(ret)) {
        logger_->warn("Failed to set env attributes");
        return false;
    }
    ret = odbcInterface_->SQLAllocHandle(SQL_HANDLE_DBC, hdlEnv_, &hdlDbc_);
    if (!SQL_SUCCEEDED(ret)) {
        logger_->warn("Failed to allocate odbc DBC handle");
        return false;
    }
    ret = odbcInterface_->SQLConnect(hdlDbc_, (SQLCHAR*)dsnName_.c_str(),
        SQL_NTS,(SQLCHAR*)userId_.c_str(),SQL_NTS,
        (SQLCHAR*)passwd_.c_str(), SQL_NTS);
    if(!SQL_SUCCEEDED(ret)) {
        logger_->warn("Could not connect to database");
        return false;
    }
    logger_->info("Connected to database");
    return true;
}

bool MicrosoftSqlServerAdapter::disconnect() {
    logger_->info("disconnect");
    SQLRETURN ret(0);
    bool result = true;
    if (!isConnected()) {
        logger_->warn("Not previously initialized");
        result = false;
    }
    /*
     * Continue to free the resources even if connection was not established
     * There is no robust way to check the connection status in all cases,
     * so this is the best way to avoid memory leaks and broken connections.
     */
    if (hdlDbc_) {
        ret = odbcInterface_->SQLDisconnect(hdlDbc_);
    }
    if(!SQL_SUCCEEDED(ret)) {
        logger_->warn("Error disconnecting. Transaction still open?");
        result = false;
    }
    if (hdlDbc_) {
        odbcInterface_->SQLFreeHandle(SQL_HANDLE_DBC, hdlDbc_);
        hdlDbc_ = nullptr;
    }
    if (hdlEnv_) {
        odbcInterface_->SQLFreeHandle(SQL_HANDLE_ENV, hdlEnv_);
        hdlEnv_ = nullptr;
    }
    return result;
}

SqlResult MicrosoftSqlServerAdapter::executeQuery(const std::string& query) {
    logger_->info("executeQuery");
    logger_->debug("query: {}", query);
    if (!isConnected()) {
        logger_->warn("Not connected to database. Returning empty result");
        return SqlResult();
    }
    SQLRETURN ret(0);
    SQLHSTMT hdlStmt(nullptr);
    ret = odbcInterface_->SQLAllocHandle(SQL_HANDLE_STMT, hdlDbc_, &hdlStmt);
    if (!SQL_SUCCEEDED(ret)) {
        logger_->warn("Failed to allocate odbc Stmt handle. Returning empty result");
        return SqlResult();
    }
    ret = odbcInterface_->SQLExecDirect(hdlStmt, (SQLCHAR*)query.c_str(), SQL_NTS);
    if(!SQL_SUCCEEDED(ret)) {
        logger_->warn("Error executing statement. Returning empty result");
        return SqlResult();
    }
    SQLSMALLINT numCols(0);
    ret = odbcInterface_->SQLNumResultCols(hdlStmt, &numCols);
    if (numCols == 0) {
        logger_->warn("Database returned 0 columns. Returning empty result");
        return SqlResult();
    }
    logger_->info("numCols: {}", numCols);
    std::vector<std::unique_ptr<char[]> > dataBuff;
    std::vector<std::unique_ptr<char[]> > columnLabels;
    for (int i = 0; i < numCols; i++) {
        dataBuff.push_back(std::make_unique<char[]>(MAX_BUFFER_SIZE));
        columnLabels.push_back(std::make_unique<char[]>(MAX_BUFFER_SIZE));
    }
    for (int i = 0; i < numCols; i++) {
        SQLSMALLINT bufferLenUsed;
        ret = odbcInterface_->SQLColAttribute(hdlStmt, (SQLUSMALLINT)i + 1, SQL_DESC_LABEL,
            (SQLPOINTER)columnLabels[i].get(), (SQLSMALLINT)MAX_BUFFER_SIZE, &bufferLenUsed, NULL);
        if (!SQL_SUCCEEDED(ret)) {
            logger_->warn("Cannot get col {} attribute", i+1);
            return SqlResult();
        }
        logger_->debug("Col {} attr: {}", i+1, (const char*)columnLabels[i].get());
        ret = odbcInterface_->SQLBindCol(hdlStmt, i+1, SQL_C_CHAR,
            (SQLPOINTER)dataBuff[i].get(), sizeof(dataBuff[i]), NULL);
        if (!SQL_SUCCEEDED(ret)) {
            logger_->warn("Cannot bind col {}", i+1);
            return SqlResult();
        }
    }
    SqlResult result;
    while(SQL_SUCCEEDED(ret = odbcInterface_->SQLFetchScroll(hdlStmt, SQL_FETCH_NEXT,1))) {
        nlohmann::json j;
        for (int i = 0; i < numCols; i++) {
            logger_->debug("col({}): {}", i+1, dataBuff[i].get());
            j[columnLabels[i].get()] = dataBuff[i].get();
        }
        result.insertRow(j);
    }
    return result;
}

bool MicrosoftSqlServerAdapter::isConnected() const {
    /*
     * Even though the alternative in this case might seem wrong
     * (not possible to have Dbc without Env), it is a robust way to guarantee
     * that the resources will always be freed in case of some changes in the production
     * code.
     */
    return hdlEnv_ || hdlDbc_;
}

}  // namespace sqladapter
}  // namespace communication
}  // namespace crf
