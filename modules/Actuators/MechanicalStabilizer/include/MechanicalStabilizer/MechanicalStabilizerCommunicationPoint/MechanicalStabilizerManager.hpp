/* © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Alejandro Diaz Rosales CERN BE/CEM/MRO 2021
 *
 *  ==================================================================================================
 */

#pragma once

#include <memory>
#include <thread>
#include <chrono>

#include <nlohmann/json.hpp>

#include "DeviceManager/DeviceManagerWithPriorityAccess/DeviceManagerWithPriorityAccess.hpp"
#include "MechanicalStabilizer/IMechanicalStabilizer.hpp"
#include "EventLogger/EventLogger.hpp"

namespace crf::actuators::mechanicalstabilizer {

/**
 * @brief The mechanical stabilizer manager class ensures thread safe access to the stabilizer. It
 *        is also responsible to optimize the resources. If no one is requesting control the
 *        stabilizer is not deinitialized, since it can be dangerous if other devices are moving.
 *        It also determines which requester has the priority to use it. The stabilizer must not be
 *        initialized prior to this class.
 */
class MechanicalStabilizerManager : public crf::utility::devicemanager::DeviceManagerWithPriorityAccess {  // NOLINT
 public:
    explicit MechanicalStabilizerManager(std::shared_ptr<IMechanicalStabilizer> stabilizer,
        const std::chrono::milliseconds& initializationTimeout = std::chrono::seconds(0),
        const std::chrono::milliseconds& controlAccessTimeout = std::chrono::seconds(10));
    MechanicalStabilizerManager(const MechanicalStabilizerManager& other) = delete;
    MechanicalStabilizerManager(MechanicalStabilizerManager&& other) = delete;
    MechanicalStabilizerManager() = delete;
    ~MechanicalStabilizerManager();

    nlohmann::json getStatus() override;

    /**
     * @brief Requests the control of the stabilizer given the priority. If a higher priority is
     *        given, the control over the rest will be revoked.
     * @param priority the priority of the system from 1 to 2^32, being the 1 the number with
     *        highest priority.
     * @return A shared pointer of the stabilizer if the access was granted.
     * @return nullptr if there is someone else with a higher priority and you cant control the
     *         stabilizer.
     */
    bool lockControl(const uint32_t &priority);
    /**
     * @brief Notifies that the control done by the client with the given priority stops
     *        controlling the arm.
     * @param priority the priority of the system from 1 to 2^32, being the 1 the number with
     *        highest priority.
     * @return True if the unlock of the stabilizer control was successful.
     * @return False if the unlock of the stabilizer control failed.
     */
    bool unlockControl(const uint32_t &priority);
    /**
     * @brief This function is used to close the mechanical stabilizer.
     * @param priority the priority of the system from 1 to 2^32, being the 1 the number with
     *        highest priority.
     * @return true if the mechanical stabilizer has been closed correctly.
     * @return false if it failed or if you didn't request the control.
     */
    bool activate(const uint32_t &priority);
    /**
     * @brief This function is used to open the mechanical stabilizer.
     * @param priority the priority of the system from 1 to 2^32, being the 1 the number with
     *        highest priority.
     * @return true if the mechanical stabilizer has been opened correctly.
     * @return false if it failed or if you didn't request the control.
     */
    bool deactivate(const uint32_t &priority);
    /**
     * @brief Reset the fault state of the mechanical stabilizer motor.
     * @param priority the priority of the system from 1 to 2^32, being the 1 the number with
     *        highest priority.
     * @return True if the reset is succesful.
     * @return False otherwise.
     */
    bool resetFaultState(const uint32_t &priority);

 private:
    std::shared_ptr<crf::actuators::mechanicalstabilizer::IMechanicalStabilizer> stabilizer_;
    crf::utility::logger::EventLogger logger_;
};

}  // namespace crf::actuators::mechanicalstabilizer
