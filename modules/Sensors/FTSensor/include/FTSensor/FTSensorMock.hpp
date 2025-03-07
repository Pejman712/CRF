/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Zsolt Pasztori CERN EN/SMM/MRO 2019
 *
 *  ==================================================================================================
 */

#pragma once

#include "FTSensor/IFTSensor.hpp"
#include <string>

namespace crf {
namespace sensors {
namespace ftsensor {

class FTSensorMock : public IFTSensor {
 public:
    MOCK_METHOD0(initialize, bool());
    MOCK_METHOD0(deinitialize, bool());
    MOCK_METHOD0(getFT, crf::utility::types::TaskForceTorque());
    MOCK_METHOD0(getRawFT, crf::utility::types::TaskForceTorque());
    MOCK_METHOD2(getFTGravityFree, crf::utility::types::TaskForceTorque(
        const utility::types::TaskPose&, bool inWorldCoordinateSystem));
    MOCK_METHOD2(updateBias, bool(
        const utility::types::TaskPose&, const std::string& logFilePath));
    MOCK_METHOD0(isCalibrated, bool());
};

}  // namespace ftsensor
}  // namespace sensors
}  // namespace crf
