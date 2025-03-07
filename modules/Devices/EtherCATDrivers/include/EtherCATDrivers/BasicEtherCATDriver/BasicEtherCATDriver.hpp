/* © Copyright CERN 2023. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Natalija Topalovic CERN BE/CEM/MRO 2023
 *
 *  ==================================================================================================
 */

#pragma once

#include <memory>
#include <optional>
#include <cstdint>

#include "CommonInterfaces/IInitializable.hpp"
#include "EtherCATDrivers/EtherCATMaster/EtherCATMaster.hpp"

#include "EventLogger/EventLogger.hpp"

namespace crf::devices::ethercatdrivers {

/**
 * @ingroup group_basic_ethercat_driver
 * @brief Basic driver for all the EtherCAT slaves. This driver is in charge of
 * moving the slave from INIT to OPERATIONAL. The default implementation has no
 * IO Map bindings, this is supposed to be overwritten and implemented by derived classes.
 *
 */
class BasicEtherCATDriver : public utility::commoninterfaces::IInitializable {
 public:
    BasicEtherCATDriver() = delete;
    BasicEtherCATDriver(std::shared_ptr<EtherCATMaster> master, const uint16_t& id);
    BasicEtherCATDriver(const BasicEtherCATDriver&) = delete;
    BasicEtherCATDriver(BasicEtherCATDriver&&) = delete;
    ~BasicEtherCATDriver() override;

    /**
     * @brief Initializes the EtherCAT motor. The initialize method is responsible of initializing
     *        the status of the motor, setup the default PDOs and check that the connection to the
     *        motor is successful. The initialize method will not change the status of the motor.
     * @return True the initialization ended correctly.
     * @return False otherwise.
     */
    bool initialize() override;

    /**
     * @brief Deinitialize the EtherCAT motor. The deinitialize method is responsible of
     *        deinitializing the motor. The motor is brought to a safe state.
     * @return True the deinitialization ended correctly.
     * @return False otherwise.
     */
    bool deinitialize() override;

    /**
     * @brief Get the EtherCAT ID of the motor. It is the ID used to identify the EtherCAT Motor.
     *        In case of more motors, it must correspond to the position number on the motors
     *        chain.
     * @return The EtherCAT ID of the motor.
     */
    int getID() const;

    /**
     * @brief Gets the EtherCAT state of the motor.
     * @return boost::none if the motor is not initialized.
     * @return The EtherCAT state.
     */
    std::optional<uint16_t> getEtherCatState() const;

    /**
     * @brief Returns if the motor is connected and it's in the EtherCAT State Operation Mode.
     * @return true the motor is alive and in Operation Mode.
     * @return false if the the motor is not initialized or is not in Operation Mode.
     */
    bool isAlive() const;

 protected:
    /**
     * @brief Function that binds the IO map to the respective Inputs and Outputs.
     *
     * @return true if the binding was succesfull
     * @return false otherwise
     */
    virtual bool bindIOMap();

    std::atomic<bool> initialized_;
    std::atomic<bool> ioMapBound_;

 private:
    std::shared_ptr<EtherCATMaster> master_;
    uint16_t id_;

    utility::logger::EventLogger logger_;
};

}  // namespace crf::devices::ethercatdrivers
