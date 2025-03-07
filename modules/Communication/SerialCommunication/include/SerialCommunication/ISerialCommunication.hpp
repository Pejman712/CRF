/* © Copyright CERN 2018. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Pawel Ptasznik CERN EN/SMM/MRO 2018
 *
 *  ==================================================================================================
 */

#pragma once

#include <string>

#include "CommonInterfaces/IInitializable.hpp"

namespace crf {
namespace communication {
namespace serialcommunication {

class ISerialCommunication: public utility::commoninterfaces::IInitializable {
 public:
    ~ISerialCommunication() override = default;

    virtual int read(std::string* buff, int length) = 0;
    virtual int write(const std::string& buff) = 0;
};

}  // namespace serialcommunication
}  // namespace communication
}  // namespace crf
