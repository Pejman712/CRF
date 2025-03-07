/* © Copyright CERN 2017. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Pawel Ptasznik CERN EN/STI/ECE 2017
 *
 *  ==================================================================================================
 */

#pragma once

#include <string>

#include <gmock/gmock.h>

#include "SerialCommunication/ISerialCommunication.hpp"

namespace crf {
namespace communication {
namespace serialcommunication {

class SerialCommunicationMock : public ISerialCommunication {
 public:
  MOCK_METHOD0(initialize,
      bool());
  MOCK_METHOD0(deinitialize,
      bool());
  MOCK_METHOD2(read,
      int(std::string* buff, int length));
  MOCK_METHOD1(write,
      int(const std::string& buff));
};

}  // namespace serialcommunication
}  // namespace communication
}  // namespace crf
