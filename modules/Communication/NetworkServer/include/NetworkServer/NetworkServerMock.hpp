#pragma once
/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Arturs Ivanovs
 *
 *  ==================================================================================================
 */
#include <string>
#include <gmock/gmock.h>

#include "NetworkServer/INetworkServer.hpp"

namespace crf {
namespace communication {
namespace networkserver {

class NetworkServerMock : public INetworkServer {
 public:
  MOCK_METHOD1(acceptConnection,
      bool(bool));
  MOCK_METHOD0(disconnect,
      bool());
  MOCK_METHOD2(send,
      bool(const Packets::PacketHeader& header, const std::string& buffer));
  MOCK_METHOD2(receive,
      bool(Packets::PacketHeader* header, std::string* buffer));
  MOCK_CONST_METHOD0(isConnected,
      bool());
};

}  // namespace networkserver
}  // namespace communication
}  // namespace crf
