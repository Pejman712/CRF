/* © Copyright CERN 2018. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Pawel Ptasznik CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#include <exception>
#include <nlohmann/json.hpp>

#include "SqlAdapter/SqlResult.hpp"

namespace crf {
namespace communication {
namespace sqladapter {

SqlResult::Iterator::Iterator(SqlResult* sqlResult, int position):
    sqlResult_(sqlResult),
    position_(position) {}

SqlResult::Iterator::reference SqlResult::Iterator::operator*() const {
    if (!sqlResult_) {
        throw std::runtime_error("Iterator pointing to null object");
    }
    return sqlResult_->getRow(position_);
}

SqlResult::Iterator::pointer SqlResult::Iterator::operator->() const {
    if (!sqlResult_) {
        throw std::runtime_error("Iterator pointing to null object");
    }
    return &sqlResult_->getRow(position_);
}

SqlResult::Iterator& SqlResult::Iterator::operator++() {
    if (!sqlResult_) {
        throw std::runtime_error("Iterator pointing to null object");
    }
    if (position_ < sqlResult_->numRows()) {
        position_++;
    }
    return *this;
}

SqlResult::Iterator SqlResult::Iterator::operator++(int) {
    Iterator retval = *this;
    ++(*this);
    return retval;
}

bool SqlResult::Iterator::operator==(const Iterator& other) const {
    return sqlResult_ == other.sqlResult_ && position_ == other.position_;
}

bool SqlResult::Iterator::operator!=(const Iterator& other) const {
    return !(other == *this);
}

SqlResult::SqlResult() : SqlResult(SqlResult::Result::Succeded) {}

SqlResult::SqlResult(SqlResult::Result result) :
    logger_("SqlResult"),
    result_(),
    resultEnum_(result) {
        logger_->debug("CTor");
}

int SqlResult::numRows() const {
    return result_.size();
}

const nlohmann::json& SqlResult::getRow(int num) const {
    if (numRows() <= num || num < 0) {
        logger_->warn("Requested row number {} from the result containing {} rows",
            num, numRows());
        throw std::out_of_range("Row: " + std::to_string(num) + " is out of range");
    }
    return result_[num];
}

bool SqlResult::insertRow(const nlohmann::json& row) {
    result_.push_back(row);
    return true;
}

SqlResult::Result SqlResult::getResult() const {
    return resultEnum_;
}

SqlResult::Iterator SqlResult::begin() {
    return Iterator(this, 0);
}
SqlResult::Iterator SqlResult::end() {
    return Iterator(this, numRows());
}

}  // namespace sqladapter
}  // namespace communication
}  // namespace crf
