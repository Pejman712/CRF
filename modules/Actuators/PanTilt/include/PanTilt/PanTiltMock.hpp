#pragma once
/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Carlos Veiga Almagro & Jorge Camarero Vera CERN EN/STI/ECE
 *
 *  ==================================================================================================
 */

#include <gmock/gmock.h>

#include <utility>

#include "PanTilt/IPanTilt.hpp"

namespace crf {
namespace devices {
namespace pantilt {

class MockIPanTilt : public IPanTilt {
 public:
    MOCK_METHOD0(initialize, bool());
    MOCK_METHOD0(deinitialize, bool());

    MOCK_METHOD0(getPosition, std::pair<double, double>());
    MOCK_METHOD1(setPosition, bool(const std::pair<double, double> &));
};

}  // namespace pantilt
}  // namespace devices
}  // namespace crf
