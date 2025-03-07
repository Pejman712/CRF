#pragma once

/* © Copyright CERN 2018. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Pawel Ptasznik CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#include "NetworkClient/INetworkClient.hpp"
#include <string>

namespace crf {
namespace communication {
namespace networkclient {

class NetworkClientMock : public INetworkClient {
 public:
  MOCK_METHOD1(connect,
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

}  // namespace networkclient
}  // namespace communication
}  // namespace crf
