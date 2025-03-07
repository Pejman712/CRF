#pragma once

/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Giacomo Lunghi CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#include <string>
#include <vector>

#include "Tools/ITool.hpp"

namespace crf {
namespace devices {
namespace tools {

class ToolMock : public ITool {
 public:
  MOCK_METHOD0(initialize,
      bool());
  MOCK_METHOD0(deinitialize,
      bool());
  MOCK_METHOD2(setValue,
      bool(const std::string& name, bool value));
  MOCK_METHOD2(setValue,
      bool(const std::string& name, int value));
  MOCK_METHOD2(setValue,
      bool(const std::string& name, float value));
  MOCK_CONST_METHOD1(getValue,
      boost::optional<boost::any>(const std::string& name));
  MOCK_METHOD1(getValueType,
      boost::optional<ToolValueTypes>(const std::string& name));
  MOCK_METHOD0(getValueNames,
      std::vector<std::string>());
};

}  // namespace tools
}  // namespace devices
}  // namespace crf
