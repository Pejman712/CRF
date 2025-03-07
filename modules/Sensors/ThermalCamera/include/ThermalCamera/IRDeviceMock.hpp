/* © Copyright CERN 2017. All rights reserved. This software is released under a CERN proprietary software license. 
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 * 
 * Author: Pawel Ptasznik CERN EN/STI/ECE
 * 
 *  ==================================================================================================
 */

#pragma once

#include <gmock/gmock.h>

#include <libirimager/IRDevice.h>

namespace evo {

class IRDeviceMock : public IRDevice {
 public:
  MOCK_METHOD0(startStreaming,
      int());
  MOCK_METHOD0(stopStreaming,
      int());
  MOCK_METHOD1(setRawFrameCallback,
      void(fptrIRRawFrame cb));
  MOCK_METHOD1(setClient,
      void(IRImagerClient* client));
  MOCK_METHOD0(run,
      void());
  MOCK_METHOD0(exit,
      void());
};

}  // namespace evo
