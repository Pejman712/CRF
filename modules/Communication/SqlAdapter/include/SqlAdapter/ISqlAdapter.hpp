#pragma once

/* © Copyright CERN 2018. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Pawel Ptasznik CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#include <string>

namespace crf {
namespace communication {
namespace sqladapter {

// FwD, full definition in SqlResult.hpp
class SqlResult;

class ISqlAdapter {
 public:
    virtual ~ISqlAdapter() = default;
    virtual bool connect() = 0;
    virtual bool disconnect() = 0;
    virtual SqlResult executeQuery(const std::string& query) = 0;
};

}  // namespace sqladapter
}  // namespace communication
}  // namespace crf
