#pragma once

/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Giacomo Lunghi CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#include <memory>
#include <mysql/mysql.h>
#include <string>

#include "EventLogger/EventLogger.hpp"
#include "SqlAdapter/ISqlAdapter.hpp"

namespace crf {
namespace communication {
namespace sqladapter {

class MySqlAdapter : public ISqlAdapter {
 public:
    MySqlAdapter() = delete;
    MySqlAdapter(const std::string& address,
        const std::string& username,
        const std::string password,
        const std::string& dbName = "",
        int port = 0);
    MySqlAdapter(MySqlAdapter&&) = delete;
    MySqlAdapter(const MySqlAdapter&) = delete;
    ~MySqlAdapter() override;
    bool connect() override;
    bool disconnect() override;
    SqlResult executeQuery(const std::string& query) override;

 private:
    utility::logger::EventLogger logger_;
    std::string address_;
    std::string username_;
    std::string password_;
    std::string dbName_;
    int port_;
    std::shared_ptr<MYSQL> connection_;
    static int instances_count;
};

}  // namespace sqladapter
}  // namespace communication
}  // namespace crf
