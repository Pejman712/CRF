/* © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Francesco Riccardi CERN BE/CEM/MRO 2021
 *
 *  ==================================================================================================
 */

#pragma once

#include <memory>

#include <boost/optional/optional.hpp>

#include "MechanicalStabilizer/IMechanicalStabilizer.hpp"
#include "EtherCATDevices/IEtherCATMotor.hpp"
#include "EventLogger/EventLogger.hpp"

namespace crf::actuators::mechanicalstabilizer {

class TIMStabilizer : public IMechanicalStabilizer {
 public:
    TIMStabilizer() = delete;
    TIMStabilizer(std::shared_ptr<crf::devices::ethercatdevices::IEtherCATMotor> motor,
        const int cycleTime = 1000000, const int totalTime = 20000);
    TIMStabilizer(const TIMStabilizer&) = delete;
    TIMStabilizer(TIMStabilizer&&) = delete;
    ~TIMStabilizer() override;

    bool initialize() override;
    bool deinitialize() override;

    bool activate() override;
    bool deactivate() override;
    std::optional<bool> isActivated() override;
    std::optional<bool> isDeactivated() override;
    bool resetFaultState() override;
    std::optional<bool> isInFault() override;

 private:
    std::shared_ptr<crf::devices::ethercatdevices::IEtherCATMotor> ecMotor_;
    int cycleTime_;
    int totalTime_;
    crf::utility::logger::EventLogger logger_;
    bool isInitialized_;
    bool isActivated_;

    /**
     * @brief Motor Nominal Current is 3600 mA.
     */
    const uint32_t motorRatedCurrent_ = 3600;
    /**
     * @brief This value has to be equal to the motorRatedCurrent (according to the ELMO
     *        Documentation).
     */
    const uint32_t motorRatedTorque_ = 3600;
    /**
     * @brief Max Current for this motor is 6[A], so the value is evaluated as
     *        6[A] * 1000 * 1000 / 2800 = 2142. i.e. it is 214.5% of the Nominal Current.
     */
    const uint16_t motorMaxCurrent_ = 2142;
    /**
     * @brief This value has to be equal to the motorMaxCurrent (according to the ELMO
     *        Documentation).
     */
    const uint16_t motorMaxTorque_ = 2142;
    /**
     * @brief Closing torque for this application is 2.8[A], so the value is evaluated as
     *        2.8[A] * 1000 * 1000 / 2800 = 1000. i.e. it is 100.0% of the Nominal Current.
     */
    const int16_t closingTorque_ = 1000;
    /**
     * @brief According to ELMO Software 1000 = 1428 RPM motor side.
     */
    const int32_t openingVelocity_ = -1000;
    /**
     * @brief According to ELMO Software 7000 = 10000 RPM/s motor side.
     */
    const uint32_t acceleration_ = 7000;

    bool functionTimeCheck(std::function<std::optional<bool>()> function);
};

}  // namespace crf::actuators::mechanicalstabilizer
