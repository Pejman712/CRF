/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Francesco Riccardi CERN EN/SMM/MRO 2019
 *
 *  ==================================================================================================
 */

#pragma once

#include <chrono>
#include <memory>

#include "CANOpenDevices/ICANOpenDevice.hpp"
#include "CommonInterfaces/IInitializable.hpp"

namespace crf {
namespace devices {
namespace canopendevices {

/*
 * @brief ICANOpenContext interface. The ICANOpenContext interface is responsible of reading all
 *        the can frame arriving on the CAN network. Once a can frame is received, it is redirected
 *        to the ObjectDictionary of the proper ICANOpenDevice. It is also responsible to send Sync
 *        and Guard messages to the entire CAN line.
 */
class ICANOpenContext : public utility::commoninterfaces::IInitializable {
 public:
    ~ICANOpenContext() override = default;

    /*
     * @brief initialize the context. It starts a thread reading from the can socket. If the sync
     *        frequency or the guard frequency are set, the sync/guard thread is started as well
     * @return true everything was initialized properly
     * @return false otherwise
     */
    bool initialize() override = 0;
    /*
     * @brief deinitialize the context. It stops the thread reading from the can socket. If the
     *        sync frequency or the guard frequency are set, the sync/guard thread is stopped as
     *        well.
     * @return true everything was deinitialized properly.
     * @return false otherwise.
     */
    bool deinitialize() override = 0;

    /*
     * @brief Add an ICANOpenDevice to the context. The CAN ID of the device must be unique.
     * @return true the device was correctly added to the context.
     * @return false a device with the same CAN ID was already added.
     */
    virtual bool addDevice(std::shared_ptr<ICANOpenDevice>) = 0;
    /*
     * @brief Sends the Sync message to update the set PDOs.
     * @return true sync was correctly sent.
     * @return false otherwise.
     */
    virtual bool sendSync() = 0;
    /*
     * @brief Sends the Guard message to all the devices.
     * @return true guard was correctly sent.
     * @return false otherwise.
     */
    virtual bool sendGuard() = 0;
    /*
     * @brief Set the frequency to which the sync will be automatically sent. It is still possible
     *        to send the sync message manually with sendSync(). A thread will be created at the
     *        initialization which will be responsible of sending out the sync message. It can only
     *        be set before initialization.
     * @param frequency the frequency to which the sync is sent.
     * @return true the frequency was correctly set.
     * @return false otherwise.
     */
    virtual bool setSyncFrequency(const std::chrono::milliseconds& frequency) = 0;
    /*
     * @brief Set the frequency to which the guard will be automatically sent. It is still possible
     *        to send the guard message manually with sendGuard(). A thread will be created at the
     *        initialization which will be responsible of sending out the guard message. It can
     *        only be set before initialization.
     * @param frequency the frequency to which the guard is sent
     * @return true the frequency was correctly set
     * @return false otherwise
     */
    virtual bool setGuardFrequency(const std::chrono::milliseconds& frequency) = 0;
};

}  // namespace canopendevices
}  // namespace devices
}  // namespace crf
