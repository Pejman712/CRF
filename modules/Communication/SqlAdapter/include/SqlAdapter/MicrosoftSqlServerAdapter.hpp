#pragma once

/* © Copyright CERN 2018. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Pawel Ptasznik CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#include <memory>
#include <string>
#include "EventLogger/EventLogger.hpp"
#include "SqlAdapter/ISqlAdapter.hpp"
#include "SqlAdapter/IOdbcInterface.hpp"

namespace crf {
namespace communication {
namespace sqladapter {

class MicrosoftSqlServerAdapter: public ISqlAdapter {
 public:
    MicrosoftSqlServerAdapter() = delete;
    MicrosoftSqlServerAdapter(const MicrosoftSqlServerAdapter& other) = delete;
    MicrosoftSqlServerAdapter(MicrosoftSqlServerAdapter&& other) = delete;
    MicrosoftSqlServerAdapter(const std::string& dsnName, const std::string& userId,
        const std::string& passwd, std::shared_ptr<IOdbcInterface> odbcInterface = nullptr);
    ~MicrosoftSqlServerAdapter() override;
    bool connect() override;
    bool disconnect() override;
    SqlResult executeQuery(const std::string& query) override;
 private:
    bool isConnected() const;
    utility::logger::EventLogger logger_;
    std::string dsnName_;
    std::string userId_;
    std::string passwd_;
    std::shared_ptr<IOdbcInterface> odbcInterface_;
    SQLHENV hdlEnv_;
    SQLHDBC hdlDbc_;
};

}  // namespace sqladapter
}  // namespace communication
}  // namespace crf
