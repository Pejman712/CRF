/* © Copyright CERN 2023. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Jorge Playan Garai CERN BE/CEM/MRO 2023
 *
 *  ==================================================================================================
 */

#pragma once

#include <string>
#include <chrono>
#include <memory>
#include <atomic>
#include <optional>
#include <thread>
#include <condition_variable>

#include "CommonInterfaces/IInitializable.hpp"
#include "SOEMAPI/SOEMAPI.hpp"

#include "EventLogger/EventLogger.hpp"

using crf::communication::soemapi::ISOEMAPI;

namespace crf::devices::ethercatdrivers {

/**
 * @ingroup group_ethercat_master
 * @brief The EtherCATMaster is the class in-charge of managing all the salves. All
 * the requests from the drivers pass through the master and get sent in the network.
 * The master's task is to initialize all the slaves and configure them.
 */
class EtherCATMaster : public utility::commoninterfaces::IInitializable {
 public:
    EtherCATMaster() = delete;
    EtherCATMaster(const EtherCATMaster&) = delete;
    EtherCATMaster(EtherCATMaster&&) = delete;
    EtherCATMaster(
        const std::string& ifname,
        const uint& numberOfSlaves,
        const std::chrono::microseconds& cycleTime,
        const int& ioMapSize = 4096,
        std::shared_ptr<ISOEMAPI> soemApi = nullptr);
    ~EtherCATMaster() override;

    /**
     * @brief initialize the EtherCATMaster. It binds the socket to the ifname passed to the
     *        EtherCATMaster constructor. It searches and configures all the slaves connected to the
     *        EtherCAT Network and puts them in PRE_OP Mode. The configures the IO map and moves the
     *        slaves to OPERATIONAL
     * @return True if everything was initialized properly, all slaves are in OPERATIONAL Mode and the
     *         number of slaves is the one expected.
     * @return False otherwise.
     */
    bool initialize() override;

    /**
     * @brief deinitialize the master
     */
    bool deinitialize() override;

    /**
     * @brief Retrieve the pointer to the mapped outputs on the IO map
     */
    std::optional<uint8_t*> retrieveOutputs(uint16_t slaveID) const;

    /**
     * @brief Retrieve the pointer to the mapped inputs on the IO map
     */
    std::optional<uint8_t*> retrieveInputs(uint16_t slaveID) const;

    /**
     * @brief Writes the targeted state in the slave with the according id. If id = 0 then the function
     * writes the targeted state to all the slaves. Inside the function, function
     * statecheck(slaveID, targetState, EC_TIMEOUTSTATE) is used from SOEM library to change the state
     * of the slave/s.
     * @param: slaveID - id of the slave (0 if all slaves)
     * @param: targetState - targeted state of the slave
     * @return: true if the state was successfuly written, false othrwise
     */
    bool writeSlaveState(const uint16_t& slaveID, const ec_state& targetState);

    /**
     * @brief Function that moves the slave or all the slaves from safe operational mode to operational mode.
     * Before moving to operational, process data is sent by the master and received by the slaves. Upon
     * finishing that, slave or all the slaves, can go to operational mode.
     * @param: slaveID - id of the slave (0 if all slaves)
     */
    bool goIntoOperationalMode(uint16_t slaveID);

    /**
     * @brief Reads the current state of the slave with the according id. If id = 0 then the function
     * reads the current state of all the slaves. Inside the function the state of the slave
     * is retrieved from the ec_slave array.
     * @param: slaveID - id of the slave (0 if all slaves)
     * @return: current state of the slave
     */
    uint16_t readSlaveState(uint16_t slaveID);

    /**
     * @brief Checks if the selected slave is in operational mode. If slaveID = 0 then it checks all the
     * slaves.
     */
    bool slaveCommunicationCheck(uint16_t slaveID);

    /**
     * @brief Function that transfers process data (or output data) to the slaves from the master.
     * Function uses the API of SOEM to send process data.
     * @return: >0 if processdata is transmitted.
     */
    int sendProcessData();

    /**
     * @brief Function that allows master to receive process data from slaves.
     * @param timeout in microseconds
     * @return work counter
     */
    int receiveProcessData(const std::chrono::microseconds& timeout);

    void waitForPDOExchange();

 protected:
    /**
     * @brief Function to be overriten, it can run anything the user wants in the
     * PRE-OPERATIONAL state
     *
     * @return true if the task went okay
     * @return false otherwise
     */
    virtual bool onPreOperational();

    /**
     * @brief Function to be overriten, it can run anything the user wants in the
     * SAFE-OPERATIONAL state
     *
     * @return true if the task went okay
     * @return false otherwise
     */
    virtual bool onSafeOperational();

    /**
     * @brief Function to be overriten, it can run anything the user wants in the
     * OPERATIONAL state
     *
     * @return true if the task went okay
     * @return false otherwise
     */
    virtual bool onOperational();

    /**
     * @brief This function runs in a thread. Every cycle time it sends the process data
     * and receives it. This should be in real time or close to it.
     *
     */
    void processDataLoop();

    /**
     * @brief Error handler thread, checks that the working counter is correct. In case
     * of errors it implements the error handling.
     *
     */
    void workingCounterCheck();

    // In SOEM the slave ID 0 means all the slaves in the network
    const uint16_t allSlaves_ = 0;

    std::string ifname_;
    uint64_t numberOfSlaves_;
    int ioMapSize_;
    std::chrono::nanoseconds cycleTime_;
    std::shared_ptr<ISOEMAPI> soemApi_;

 private:
    /**
     * @brief Function to check with a timeout that the state of a slave is changed.
     *
     * @param slaveID ID of the slave
     * @param state State we are waiting for the slave to have
     * @param timeout Maximum time we are allowed to wait
     * @return true If the stated changed to the expected one
     * @return false If it didn't change
     */
    bool waitStateChange(
        const uint16_t& slaveID,
        const uint16_t& state,
        const std::chrono::seconds& timeout = std::chrono::seconds(1));

    std::thread processDataThread_;
    std::thread workingCounterCheckThread_;

    std::atomic<bool> stopThreads_;
    std::unique_ptr<char[]> IOMap_;
    int expectedWKC_;
    std::atomic<int> receivedWK_;
    bool initialized_;
    std::condition_variable processDataCV_;

    utility::logger::EventLogger logger_;

    /*
     * The following methods are taken directly from SOEM Redundancy test
     */

    int64_t integral_;

    std::chrono::nanoseconds calculateOffset(
        const std::chrono::nanoseconds& reftime,
        const std::chrono::nanoseconds& cycletime);

    const int rtPriority_ = 99;
};

}  // namespace crf::devices::ethercatdrivers
