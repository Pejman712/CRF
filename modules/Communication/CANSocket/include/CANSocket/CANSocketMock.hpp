/* © Copyright CERN 2017. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Zsolt Pasztori CERN EN/STI/ECE 2017
 *
 *  ==================================================================================================
 */

#pragma once

#include <gmock/gmock.h>
#include <string>
#include "CANSocket/ICANSocket.hpp"

namespace crf {
namespace communication {
namespace cansocket {

class CANSocketMock: public ICANSocket {
 public:
    MOCK_METHOD0(initialize,
        bool());
    MOCK_METHOD0(deinitialize,
        bool());
    MOCK_CONST_METHOD0(getName,
        std::string());
    MOCK_METHOD1(write,
        int(can_frame* frame));
    MOCK_METHOD1(read,
        int(can_frame* frame));
};

}  // namespace cansocket
}  // namespace communication
}  // namespace crf
