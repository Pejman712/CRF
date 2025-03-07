/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Giacomo Lunghi CERN EN/SMM/MRO 2019
 *
 *  ==================================================================================================
 */

#pragma once

#include <memory>
#include <utility>

#include "CommonInterfaces/IInitializable.hpp"
#include "CANOpenDevices/ObjectDictionary.hpp"

namespace crf {
namespace devices {
namespace canopendevices {

class ICANOpenDevice : public utility::commoninterfaces::IInitializable {
 public:
    ~ICANOpenDevice() override = default;

    /*
     * @brief Initializes the CANOpen Device. The initialize method is responsible of
     *        initializing the status of the device, setup the default PDOs and check that the
     *        connection to the device is successful. The initialize method will not change the
     *        status of the device.
     * @return True if the initialization ended correctly.
     * @return False otherwise.
     */
    bool initialize() override = 0;
    /*
     * @brief Deinitialize the CANOpen Device. The deinitialize method is responsible of
     *        deinitializing the device. The device is brought to a safe state.
     * @return True if the deinitialization ended correctly.
     * @return False otherwise.
     */
    bool deinitialize() override = 0;

    /*
     * @brief Get the CAN ID of the device. In the case of CANOpen devices, the canID is an address
     *        comprised between 0 and 127.
     * @return The CAN ID of the device.
     */
    virtual int getCANID() = 0;
    /*
     * @brief Returns if the motor is connected and responding. The method checks that the time
     *        passed since the last heartbeat was received from the motor is lower than a threshold.
     * @return True the motor is alive.
     * @return False otherwise.
     */
    virtual bool isAlive() = 0;
    /*
     * @brief Get the Object Dictionary of the device.
     * @return Pointer to the Object Dictionary.
     */
    virtual std::shared_ptr<ObjectDictionary> getObjectDictionary() = 0;
};

}  // namespace canopendevices
}  // namespace devices
}  // namespace crf
