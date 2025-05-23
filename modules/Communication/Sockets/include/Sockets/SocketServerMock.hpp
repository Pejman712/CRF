/* © Copyright CERN 2020. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Giacomo Lunghi CERN EN/SMM/MRO 2020
 *
 *  ==================================================================================================
 */

#pragma once

#include <optional>
#include <memory>

#include <gmock/gmock.h>

#include "Sockets/ISocket.hpp"

namespace crf {
namespace communication {
namespace sockets {

class SocketServerMock : public ISocketServer {
 public:
  MOCK_METHOD0(open, bool());
  MOCK_METHOD0(close, bool());
  MOCK_METHOD0(isOpen, bool());
  MOCK_METHOD0(acceptConnection, std::optional<std::shared_ptr<ISocket> >());
};

}  // namespace sockets
}  // namespace communication
}  // namespace crf
