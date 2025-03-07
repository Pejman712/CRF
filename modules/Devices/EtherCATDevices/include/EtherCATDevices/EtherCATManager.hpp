/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license. 
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 * 
 * Author: Francesco Riccardi CERN EN/SMM/MRO 2019
 * 
 *  ==================================================================================================
 */

#pragma once

#include <atomic>
#include <condition_variable>
#include <iostream>
#include <memory>
#include <mutex>
#include <pthread.h>
#include <string>
#include <optional>

#include <soem/ethercat.h>

#include "CommonInterfaces/IInitializable.hpp"
#include "EtherCATDevices/ISoemApi.hpp"
#include "EventLogger/EventLogger.hpp"

namespace crf {
namespace devices {
namespace ethercatdevices {

/**
 * @brief The ISoemApi interface is the wrapper on the SOEM library which expose all the functions
 *        of the SOEM to the user. It is in charge of the initialization of the EtherCAT
 *        communication on low level, and it synchronizes the reading/writing of the several
 *        entities to the IOMap.
 */
class EtherCATManager : public utility::commoninterfaces::IInitializable{
 public:
    EtherCATManager() = delete;
    EtherCATManager(const EtherCATManager&) = delete;
    EtherCATManager(EtherCATManager&&) = delete;
    EtherCATManager(const std::string& ifname, const uint nSlaves, const int sizeOfIOMap,
        const int cycleTime, std::shared_ptr<ISoemApi> soemApi = nullptr);
    ~EtherCATManager() override;

    /**
     * @brief initialize the EtherCATManager. It binds the socket to the ifname passed to the
     *        EtherCATManager constructor. It search and configure all the slaves connected to the
     *        EtherCAT Network and put them in PRE_OP Mode.
     * @return True if everything was initialized properly, all slaves are in PRE_OP Mode and the
     *         number of slaves is the one expected.
     * @return False otherwise.
     */
    bool initialize() override;
    /**
     * @brief deinitialize the SoemApi.
     */
    bool deinitialize() override;
    /**
     * @brief Configures the SOEM IOMap. Once all the slaves have sent the SDO to configure their
     *        own PDO, this function is called in order to generate the IOMap used for the
     *        processdata exchange. After the IOMap is created, all the slaves are in SAFE_OP Mode.
     *        SOEM Function to be called: ec_config_map(&IOmap).
     * @return True if the actual size of the IOMap is =< of the passed parameter and if all slaves
     *         are in PRE_OP Mode.
     * @return False otherwise.
     */
    bool configureIOMap();
    /**
     * @brief
     */
    inline bool ioMapConfigured() { return ioMapConfigured_; }
    /**
     * @brief
     */
    const std::optional<uint8_t*> retrieveOutputs(uint16_t slaveID) const;
    /**
     * @brief
     */
    const std::optional<uint8_t*> retrieveInputs(uint16_t slaveID) const;
    /**
     * @brief Sends a read SDO request. This function is use when an entity want to send and SDO in
     *        order to read the Object Dictionary a specific physical device. SOEM Function to be
     *        called: ec_SDOread(slave, index, subindex, CA, sizeof(buff), buff, EC_TIMEOUTSAFE). 
     * @param SlaveID is the number of the slave in the EtherCAT Network.
     * @param Index is the one of the register of the Object Dictionary.
     * @param SubIndex is the one of the register of the Object Dictionary to read.
     * @param Size is the size (in byte) of the value to be read.
     * @param CA is used for the CompleteAccess reading. In this mode, all the subindex of the
     *        specified index are read.
     * @return The read value if the reading is successful (i.e. if the working counter returned by
     *         the function is equal to 1).
     * @return False otherwise.
     */
    template<typename T>
    std::optional<T> readSDO(uint16_t slaveID, uint16_t index, uint8_t subIndex) const;
    /**
     * @brief Sends a Write SDO request. This function is used when an entity want to send and SDO
     *        in order to write a value in the Object Dictionary a specific physical device. SOEM
     *        Function to be called: ec_SDOwrite(slave, index, subindex, CA, sizeof(buff), buff,
     *        EC_TIMEOUTSAFE).
     * @param SlaveID is the number of the slave in the EtherCAT Network.
     * @param Index is the one of the register of the Object Dictionary.
     * @param SubIndex is the one of the register of the Object Dictionary to write.
     * @param BufferValue is the value that we want to write.
     * @param CA is used for the CompleteAccess writing. In this mode, all the subindex of the
     *        specified index are written.
     * @return True if the writing is successful (i.e. if the working counter returned by the
     *         function is equal to 1).
     * @return False otherwise.
     */
    template<typename T>
    bool writeSDO(uint16_t slaveID, uint16_t index, uint8_t subIndex, T value);
    /**
     * @brief Request a slave to enter in Init Mode. This function is used to ask a slave to enter
     *        in INIT Mode.
     * @param SlaveID is the number of the slave in the EtherCAT Network. 0 -> ask to all slaves.
     * @return True if the requested slave is gone in Init Mode.
     * @return False otherwise.
     */
    uint16_t getSlaveState(uint16_t slaveID);
    /**
     * @brief
     */
    bool enterInit(uint16_t slaveID = 0);
    /**
     * @brief Request a slave to enter in PRE_OP Mode. This function is used to ask a slave to
     *        enter in PRE_OP Mode. Usually, SOEM request to enter in PRE_OP Mode to all slaves
     *        after the initialization on the socket.
     * @param SlaveID is the number of the slave in the EtherCAT Network. 0 -> ask to all slaves.
     * @return True if the requested slave is gone in PRE_OP Mode.
     * @return False otherwise.
     */
    bool enterPreOp(uint16_t slaveID = 0);
    /**
     * @brief Request a slave to enter in SAFE_OP Mode. This function is used to ask a slave to
     *        enter in SAFE_OP Mode. Usually, SOEM request to enter in SAFE_OP Mode to all slaves
     *        after the configuration of the IOMap.
     * @param SlaveID is the number of the slave in the EtherCAT Network. 0 -> ask to all slaves.
     * @return True if the requested slave is gone in SAFE_OP Mode.
     * @return False otherwise.
     */
    bool enterSafeOp(uint16_t slaveID = 0);
    /**
     * @brief Request a slave to enter in OP Mode. This function is used to ask a slave to enter
     *        in OP Mode. Before asking to enter in this mode, a sequence of processdata sending
     *        and reading has to be done "to make outputs in slaves happy". SOEM Function to be
     *        called: ec_send_processdata(); ec_receive_processdata(EC_TIMEOUTRET).
     * @param SlaveID is the number of the slave in the EtherCAT Network. 0 -> ask to all slaves.
     * @return True if the requested slave is gone in OP Mode
     * @return False otherwise.
     */
    bool enterOp(uint16_t slaveID = 0);
    /**
     * @brief
     */
    bool slaveCommunicationCheck(uint16_t slaveID);
    /**
     * @brief Init mode check. This function is used to check if a defined slave is in Init Mode.
     * @param SlaveID is the number of the slave in the EtherCAT Network. 0 -> ask to all slaves.
     * @return True if the requested slave is in Init Mode.
     * @return False otherwise.
     */
    bool checkInit(uint16_t slaveID = 0);
    /**
     * @brief PreOp mode check. This function is used to check if a defined slave is in Init Mode.
     * @param SlaveID is the number of the slave in the EtherCAT Network. 0 -> ask to all slaves.
     * @return True if the requested slave is in PreOp Mode.
     * @return False otherwise.
     */
    bool checkPreOp(uint16_t slaveID = 0);
    /**
     * @brief SafeOp mode check. This function is used to check if a defined slave is in Init Mode.
     * @param SlaveID is the number of the slave in the EtherCAT Network. 0 -> ask to all slaves.
     * @return True if the requested slave is in SafeOp Mode.
     * @return False otherwise.
     */
    bool checkSafeOp(uint16_t slaveID = 0);
    /**
     * @brief Op mode check. This function is used to check if a defined slave is in Init Mode.
     * @param SlaveID is the number of the slave in the EtherCAT Network. 0 -> ask to all slaves.
     * @return True if the requested slave is in Op Mode.
     * @return False otherwise.
     */
    bool checkOp(uint16_t slaveID = 0);

 private:
    utility::logger::EventLogger logger_;
    int sizeOfIOMap_;
    std::unique_ptr<char[]> IOMap_;
    std::shared_ptr<ISoemApi> SoemApi_;
    std::mutex ioMapMutex_;
    std::string ifname_;
    int numberOfSlaves_;
    int expectedWKC_;
    std::atomic<int> wkc_;
    bool initialized_;
    std::atomic<bool> ioMapConfigured_;
    std::atomic<bool> inOP_;
    int64_t cycleTime_;
    int64_t toff_;
    int64_t gl_delta_;
    std::atomic<bool> startThreads_;
    pthread_t threadProcessData_;
    pthread_t threadCheck_;
    bool threadStartedFlag_;
    std::mutex threadStartedMtx_;
    std::condition_variable threadStartedCv_;
    bool dataExchanged_;
    std::mutex dataExchangedMtx_;
    std::condition_variable dataExchangedCv_;

    static void ecatthread(EtherCATManager* ptr);
    static void ecatcheck(EtherCATManager* ptr);
    void add_timespec(struct timespec* ts, int64_t addtime);
    void syncEC(int64_t reftime, int64_t cycletime, int64_t *offsettime);
};

}  // namespace ethercatdevices
}  // namespace devices
}  // namespace crf
