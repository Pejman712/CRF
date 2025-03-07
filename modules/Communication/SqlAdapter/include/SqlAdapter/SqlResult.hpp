#pragma once

/* © Copyright CERN 2018. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Pawel Ptasznik CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#include <iterator>
#include <vector>

#include <nlohmann/json.hpp>

#include "EventLogger/EventLogger.hpp"

namespace crf {
namespace communication {
namespace sqladapter {

class SqlResult {
 public:
    enum Result { Succeded, Failed };

    class Iterator: public std::iterator<std::forward_iterator_tag,
        nlohmann::json,
        int,
        const nlohmann::json*,
        const nlohmann::json&> {
     public:
        reference operator*() const;
        pointer operator->() const;
        Iterator& operator++();
        Iterator operator++(int);
        bool operator==(const Iterator& other) const;
        bool operator!=(const Iterator& other) const;
     private:
        friend SqlResult;
        Iterator(SqlResult* sqlResult, int position);
        SqlResult* sqlResult_;
        int position_;
    };

    SqlResult();
    explicit SqlResult(Result result);

    int numRows() const;
    const nlohmann::json& getRow(int num) const;
    bool insertRow(const nlohmann::json& row);
    Result getResult() const;
    Iterator begin();
    Iterator end();

 private:
    utility::logger::EventLogger logger_;
    std::vector<nlohmann::json> result_;
    Result resultEnum_;
};

}  // namespace sqladapter
}  // namespace communication
}  // namespace crf
