/* Copyright 2017 CERN */

#pragma once

#include <gmock/gmock.h>

#include "IPC/IPC.hpp"

class IpcMock : public IPC {
 public:
  MOCK_METHOD0(open,
      bool());
  MOCK_METHOD0(close,
      bool());
  MOCK_METHOD2(write,
      bool(const std::string& bytes, const Packets::PacketHeader& header));
  MOCK_METHOD2(read,
      bool(std::string& bytes, Packets::PacketHeader& header));  // NOLINT
};
