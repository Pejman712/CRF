/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Giacomo Lunghi CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#include <memory>
#include <string>
#include <vector>

#include "SqlAdapter/MySqlAdapter.hpp"
#include "SqlAdapter/SqlResult.hpp"

namespace crf {
namespace communication {
namespace sqladapter {

int MySqlAdapter::instances_count = 0;

MySqlAdapter::MySqlAdapter(const std::string& address,
    const std::string& username,
    const std::string password,
    const std::string& dbName,
    int port) :
        logger_("MySqlAdapter"),
        address_(address),
        username_(username),
        password_(password),
        dbName_(dbName),
        port_(port),
        connection_(nullptr) {
            logger_->debug("CTor");
            if (instances_count == 0) {
                logger_->info("First instance, going to initialise mysql_library");
                if (mysql_library_init(0, nullptr, nullptr) != 0) {
                    logger_->critical("Could not initialize mysql library");
                    throw std::runtime_error("Could not initialize mysql library");
                }
            }
            instances_count++;
}

MySqlAdapter::~MySqlAdapter() {
    logger_->debug("DTor");
    if (connection_) {
        disconnect();
    }
    instances_count--;
    if (instances_count == 0) {
        logger_->info("Last instance, going to deinitialize mysql_library");
        mysql_library_end();
    }
}

bool MySqlAdapter::connect() {
    logger_->debug("connect");
    if (connection_) {
        logger_->warn("Already connected");
        return false;
    }
    connection_.reset(mysql_init(nullptr), mysql_close);
    if (!connection_) {
        logger_->error("Could not initialize connection object: {}",
            mysql_error(connection_.get()));
        return false;
    }
    const char* dbNameCh = dbName_ == "" ? nullptr : dbName_.c_str();
    if (mysql_real_connect(connection_.get(), address_.c_str(), username_.c_str(),
        password_.c_str(), dbNameCh, port_, nullptr, 0) == nullptr) {
            logger_->error("Could not connect to the database: {}",
                mysql_error(connection_.get()));
            return false;
    }
    return true;
}

bool MySqlAdapter::disconnect() {
    logger_->debug("disconnect");
    if (!connection_) {
        logger_->warn("Already disconnected");
        return false;
    }
    connection_.reset();
    return true;
}

SqlResult MySqlAdapter::executeQuery(const std::string& query) {
    logger_->debug("executeQuery");
    if (!connection_) {
        logger_->warn("Not connected");
        return SqlResult(SqlResult::Result::Failed);
    }
    if (mysql_query(connection_.get(), query.c_str()) != 0) {
        logger_->error(mysql_error(connection_.get()));
        return SqlResult(SqlResult::Result::Failed);
    }
    std::unique_ptr<MYSQL_RES, void(*)(MYSQL_RES*)> result(
        mysql_store_result(connection_.get()), mysql_free_result);
    if (!result) {
        logger_->info("The query returned an empty result");
        return SqlResult(SqlResult::Result::Succeded);
    }
    int num_fields = mysql_num_fields(result.get());
    MYSQL_ROW row;
    SqlResult sqlResult(SqlResult::Result::Succeded);
    std::vector<std::string> columns;
    std::vector<enum_field_types> fieldTypes;
    MYSQL_FIELD* field;
    while ((field = mysql_fetch_field(result.get()))) {
        columns.push_back(std::string(field->name, field->name_length));
        fieldTypes.push_back(field->type);
    }
    while ((row = mysql_fetch_row(result.get()))) {
        nlohmann::json j;
        for (int i = 0; i < num_fields; i++) {
            j[columns[i]] = row[i] == nullptr ? "" : row[i];
        }
        sqlResult.insertRow(j);
    }
    return sqlResult;
}

}  // namespace sqladapter
}  // namespace communication
}  // namespace crf
