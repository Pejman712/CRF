/* © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Francesco Riccardi CERN BE/CEM/MRO
 *
 *  ==================================================================================================
 */

#pragma once

#include <memory>

#include "Shielding/IShielding.hpp"
#include "EtherCATDevices/IEtherCATMotor.hpp"
#include "EventLogger/EventLogger.hpp"

namespace crf::actuators::shielding {

class RadioactiveSourceShielding : public IShielding {
 public:
    RadioactiveSourceShielding() = delete;
    RadioactiveSourceShielding(std::shared_ptr<crf::devices::ethercatdevices::IEtherCATMotor> motor,
        const int cycleTime = 1000000, const int totalTime = 20000);
    RadioactiveSourceShielding(const RadioactiveSourceShielding&) = delete;
    RadioactiveSourceShielding(RadioactiveSourceShielding&&) = delete;
    ~RadioactiveSourceShielding() override;

    bool initialize() override;
    bool deinitialize() override;

    bool open() override;
    bool close() override;
    std::optional<bool> isOpen() override;
    std::optional<bool> isClosed() override;
    bool resetFaultState() override;
    std::optional<bool> isInFault() override;

 private:
    std::shared_ptr<crf::devices::ethercatdevices::IEtherCATMotor> ecMotor_;
    int cycleTime_;
    int totalTime_;
    crf::utility::logger::EventLogger logger_;
    bool isInitialized_;

    /**
     * @brief Motor Nominal Current is 2800 mA.
     */
    const uint32_t motorRatedCurrent_ = 2800;
    /**
     * @brief This value has to be equal to the motorRatedCurrent (according to the ELMO
     *        Documentation).
     */
    const uint32_t motorRatedTorque_ = 2800;
    /**
     * @brief Max Current for this motor is 4[A], so the value is evaluated as
     *        4[A] * 1000 * 1000 / 2800 = 1428. i.e. it is 142.8% of the Nominal Current.
     */
    const uint16_t motorMaxCurrent_ = 2000;
    /**
     * @brief This value has to be equal to the motorMaxCurrent (according to the ELMO
     *        Documentation).
     */
    const uint16_t motorMaxTorque_ = 2000;
    /**
     * @brief This the motor position in which the shielding is open
     */
    const int32_t openPosition_ = -527557;
    /**
     * @brief This the motor position in which the shielding is close
     */
    const int32_t closePosition_ = -4000;
    /**
     * @brief Admissible error to consider the shielding close or open
     */
    const int32_t positionThreshold_ = 4000;
    /**
     * @brief According to ELMO Software
     */
    const int32_t velocity_ = 87381;
    /**
     * @brief According to ELMO Software
     */
    const uint32_t acceleration_ = 87381;

    bool functionTimeCheck(std::function<std::optional<bool>()> function);
};

}  // namespace crf::actuators::shielding
