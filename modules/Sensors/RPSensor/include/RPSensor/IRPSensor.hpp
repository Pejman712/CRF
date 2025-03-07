/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Giacomo Lunghi CERN EN/SMM/MRO 2019
 *
 *  ==================================================================================================
 */

#pragma once

#include <optional>

#include "CommonInterfaces/IInitializable.hpp"

namespace crf {
namespace sensors {
namespace rpsensor {

class IRPSensor : public utility::commoninterfaces::IInitializable {
 public:
    virtual ~IRPSensor() = default;

    bool initialize() override = 0;
    bool deinitialize() override = 0;

    /*
     * @brief
     * @return
     */
    virtual std::optional<float> getDoseRate() = 0;
    /*
     * @brief
     * @return
     */
    virtual std::optional<float> getCumulativeDose() = 0;
    /*
     * @brief
     * @return
     */
    virtual bool resetCumulativeDose() = 0;
};

}  // namespace rpsensor
}  // namespace sensors
}  // namespace crf
