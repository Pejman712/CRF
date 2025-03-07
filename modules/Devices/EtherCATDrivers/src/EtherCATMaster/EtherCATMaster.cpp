/* © Copyright CERN 2023. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Jorge Playan Garai CERN BE/CEM/MRO 2023
 *
 *  ==================================================================================================
 */

#include <string>
#include <chrono>
#include <memory>
#include <atomic>
#include <optional>
#include <thread>
#include <pthread.h>

#include "EtherCATDrivers/EtherCATMaster/EtherCATMaster.hpp"

namespace crf::devices::ethercatdrivers {

EtherCATMaster::EtherCATMaster(
    const std::string& ifname,
    const uint& numberOfSlaves,
    const std::chrono::microseconds& cycleTime,
    const int& ioMapSize,
    std::shared_ptr<ISOEMAPI> soemApi):
    ifname_(ifname),
    numberOfSlaves_(numberOfSlaves),
    ioMapSize_(ioMapSize),
    cycleTime_(cycleTime),
    soemApi_(soemApi),
    stopThreads_(true),
    expectedWKC_(0),
    receivedWK_(0),
    initialized_(false),
    logger_("EtherCATMaster"),
    integral_(0) {
    logger_->debug("CTor");
    if (numberOfSlaves_ < 1) {
        throw std::invalid_argument("Slave number invalid, it must be higher than 0");
    }
    if (ioMapSize_ < 0) {
        throw std::invalid_argument("IO Map size invalid, it must be higher than 0");
    }
    if (cycleTime_ <= std::chrono::microseconds(0)) {
        throw std::invalid_argument("Cycle time invalid, it must be higher than 0");
    }
    if (soemApi_ == nullptr) {
        soemApi_ = std::make_shared<crf::communication::soemapi::SOEMAPI>();
    }
    IOMap_ = std::make_unique<char[]>(ioMapSize_);
}

EtherCATMaster::~EtherCATMaster() {
    logger_->debug("DTor");

    // Just in case the initialization failed after starting the threads.
    stopThreads_ = true;
    if (processDataThread_.joinable()) {
        processDataThread_.join();
    }
    if (workingCounterCheckThread_.joinable()) {
        workingCounterCheckThread_.join();
    }
    if (initialized_) deinitialize();
}

bool EtherCATMaster::initialize() {
    logger_->debug("initialize");
    if (initialized_) {
        logger_->warn("Already initialized");
        return false;
    }

    // Initialize SOEM
    if (soemApi_->init(const_cast<char*>(ifname_.c_str())) <= 0) {
        logger_->error("No socket connection on {}. Execute as root", ifname_.c_str());
        return false;
    }
    logger_->info("Initialization on {} successful", ifname_.c_str());

    // Configure network and move slaves to PRE-OPERATIONAL
    if (soemApi_->config_init(FALSE) <= 0) {
        logger_->error("No Slaves found");
        return false;
    }

    if (!waitStateChange(allSlaves_, EC_STATE_PRE_OP)) {
        logger_->error("Not all slaves were moved to PRE-OPERATIONAL");
        return false;
    }
    logger_->info("All slaves moved to PRE-OPERATIONAL");

    // Check slaves
    if (static_cast<uint64_t>(soemApi_->ec_slavecount()) != numberOfSlaves_) {
        logger_->warn("Only {} slaves found out of {} slaves!",
            soemApi_->ec_slavecount(), numberOfSlaves_);
        return false;
    }
    logger_->info("Found {} slaves", numberOfSlaves_);

    // TODO(any): We could further check that the slaves found match the expected ones
    // ideally parsing the ESI file too (jplayang)

    // Launch threads
    stopThreads_ = true;
    if (processDataThread_.joinable()) processDataThread_.join();
    if (workingCounterCheckThread_.joinable()) workingCounterCheckThread_.join();

    stopThreads_ = false;
    processDataThread_ = std::thread(&EtherCATMaster::processDataLoop, this);
    workingCounterCheckThread_ = std::thread(&EtherCATMaster::workingCounterCheck, this);

    // In here we set the priority of the thread (C style because there's no modern way)
    sched_param sch;
    sch.sched_priority = rtPriority_;
    int res = pthread_setschedparam(processDataThread_.native_handle(), SCHED_FIFO, &sch);
    if (res != 0) {
        if (res == ESRCH) {
            logger_->warn(
                "Failed to set priority for the EtherCAT process data thread"
                "error: {} \"No thread with this ID could be found\"",
                ESRCH);
        }
        if (res == EINVAL) {
            logger_->warn(
                "Failed to set priority for the EtherCAT process data thread"
                "error: {} \"Not recognized policy or param makes no sense for policy\"",
                EINVAL);
        }
        if (res == EPERM) {
            logger_->warn(
                "Failed to set priority for the EtherCAT process data thread"
                "error: {} \"No priviliges to set the specified policy and params\"",
                EPERM);
        }
    }

    if (!onPreOperational()) return false;

    // Allocate IO Map memory, this moves all slaves to SAFE-OP
    int usedMemory = soemApi_->config_overlap_map(IOMap_.get());
    if (usedMemory > ioMapSize_) {
        logger_->error("Memory necessary ({}) for the IOMap is bigger than the one specified ({}),"
            " allow more memory allocation", usedMemory, ioMapSize_);
        return false;
    }

    if (!waitStateChange(allSlaves_, EC_STATE_SAFE_OP)) {
        logger_->error("Not all slaves were moved to SAFE-OPERATIONAL");
        return false;
    }

    logger_->info("IOMap configured: ({}) bytes used from ({}) available",
        usedMemory, ioMapSize_);
    logger_->info("All slaves moved to SAFE-OPERATIONAL");

    if (!onSafeOperational()) return false;

    if (soemApi_->configdc()) {
        logger_->info("Distributed clock configured and propagation delays evaluated");
    }

    // Calculate expected work counter and configure DC
    expectedWKC_ = (soemApi_->ec_group()[0].outputsWKC * 2) + soemApi_->ec_group()[0].inputsWKC;
    soemApi_->readstate();
    logger_->info("Expected Working Counter: {}", expectedWKC_);

    // Move all slaves to operational
    if (!goIntoOperationalMode(allSlaves_)) {
        logger_->error("Could not move all slaves to OPERATIONAL");
        return false;
    }

    if (!waitStateChange(allSlaves_, EC_STATE_OPERATIONAL)) {
        logger_->error("Not all slaves were moved to SAFE-OPERATIONAL");
        return false;
    }
    logger_->info("All slaves moved to OPERATIONAL");

    if (!onOperational()) return false;

    initialized_ = true;
    return true;
}

bool EtherCATMaster::deinitialize() {
    logger_->debug("deinitialize");
    if (!initialized_) {
        logger_->warn("Already deinitialized");
        return false;
    }

    stopThreads_ = true;
    if (processDataThread_.joinable()) {
        processDataThread_.join();
    }
    if (workingCounterCheckThread_.joinable()) {
        workingCounterCheckThread_.join();
    }

    // Move slaves away from operational
    writeSlaveState(allSlaves_, EC_STATE_SAFE_OP);
    logger_->info("State of slaves is 0x{:02X}", readSlaveState(allSlaves_));
    soemApi_->close();
    initialized_ = false;
    return true;
}

std::optional<uint8_t*> EtherCATMaster::retrieveOutputs(uint16_t slaveID) const {
    if (!initialized_) return std::nullopt;
    if (slaveID > soemApi_->ec_slavecount()) return std::nullopt;
    return soemApi_->ec_slave()[slaveID].outputs;
}

std::optional<uint8_t*> EtherCATMaster::retrieveInputs(uint16_t slaveID) const {
    if (!initialized_) return std::nullopt;
    if (slaveID > soemApi_->ec_slavecount()) return std::nullopt;
    return soemApi_->ec_slave()[slaveID].inputs;
}

bool EtherCATMaster::writeSlaveState(const uint16_t& slaveID, const ec_state& targetState) {
    soemApi_->ec_slave()[slaveID].state = targetState;
    soemApi_->writestate(slaveID);
    soemApi_->statecheck(slaveID, targetState, EC_TIMEOUTSTATE * 5);
    soemApi_->readstate();
    return waitStateChange(slaveID, targetState);
}

bool EtherCATMaster::goIntoOperationalMode(uint16_t slaveID) {
    soemApi_->send_overlap_processdata();
    receivedWK_ = soemApi_->receive_processdata(EC_TIMEOUTRET);
    return writeSlaveState(slaveID, EC_STATE_OPERATIONAL);
}

uint16_t EtherCATMaster::readSlaveState(uint16_t slaveID) {
    soemApi_->readstate();
    return soemApi_->ec_slave()[slaveID].state;
}

int EtherCATMaster::sendProcessData() {
    return soemApi_->send_overlap_processdata();
}

int EtherCATMaster::receiveProcessData(const std::chrono::microseconds& timeout) {
    return soemApi_->receive_processdata(timeout.count());
}

bool EtherCATMaster::slaveCommunicationCheck(uint16_t slaveID) {
    if (slaveID > soemApi_->ec_slavecount()) return false;
    if (soemApi_->ec_slave()[slaveID].state == EC_STATE_OPERATIONAL) return true;
    logger_->error("Slave {} is not in OPERATIONAL", slaveID);
    return false;
}

void EtherCATMaster::waitForPDOExchange() {
    std::mutex mtx;
    std::unique_lock<std::mutex> lock(mtx);
    processDataCV_.wait_for(lock, cycleTime_*2);
}

// Protected

bool EtherCATMaster::onPreOperational() {
    return true;
}

bool EtherCATMaster::onSafeOperational() {
    return true;
}

bool EtherCATMaster::onOperational() {
    return true;
}

// Private

std::chrono::nanoseconds EtherCATMaster::calculateOffset(
    const std::chrono::nanoseconds& referenceTime, const std::chrono::nanoseconds& cycleTime) {

    // Get modulo between the times
    int64_t timeDifference = (referenceTime.count()- 50000) % cycleTime.count();

    // Check if we sync with the previous cycle or the next one
    if (timeDifference > (cycleTime.count() / 2)) {
        timeDifference -= cycleTime.count();
    }

    integral_ += timeDifference * cycleTime.count();

    // These gains might need to be adjusted dependning on the system and devices
    return std::chrono::nanoseconds(static_cast<int32_t>(-0.3*timeDifference - 1e-9*integral_));
}

void EtherCATMaster::processDataLoop() {
    std::chrono::high_resolution_clock::time_point startTime =
        std::chrono::high_resolution_clock::now();

    std::chrono::nanoseconds offset(0);
    std::size_t loopCounter = 0;

    soemApi_->send_overlap_processdata();
    while (!stopThreads_) {
        std::this_thread::sleep_until(startTime + cycleTime_*loopCounter + offset);

        receivedWK_ = soemApi_->receive_processdata(EC_TIMEOUTRET);

        /**
         * TODO(any): If we need an IO cycle it should be here (maybe a function to override?)
         * (jplayang)
         */

        if (soemApi_->ec_slave()[0].hasdc) {
            offset = calculateOffset(std::chrono::nanoseconds(ec_DCtime), cycleTime_);
        }
        soemApi_->send_overlap_processdata();

        // Notify exchange has been done
        processDataCV_.notify_all();
        loopCounter++;
    }
}

void EtherCATMaster::workingCounterCheck() {
    int number = 1;
    while (!stopThreads_) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        if (!initialized_) continue;
        if (receivedWK_ >= expectedWKC_) continue;
        if (soemApi_->ec_group()[allSlaves_].docheckstate) continue;

        // One or more slaves are not responding
        logger_->warn("One or more slaves are not responding");
        soemApi_->ec_group()[0].docheckstate = FALSE;
        soemApi_->readstate();
        for (number = 1; number <= soemApi_->ec_slavecount(); number++) {
            ec_slavet& slave = soemApi_->ec_slave()[number];
            if (slave.group != 0) continue;
            if (slave.state == EC_STATE_OPERATIONAL) continue;

            soemApi_->ec_group()[0].docheckstate = TRUE;
            logger_->warn("Slave {} is not in OPERATIONAL, trying to recover...", number);
            logger_->error("Slave {} State={} StatusCode={} : {}", number, slave.state,
                slave.ALstatuscode, ec_ALstatuscode2string(slave.ALstatuscode));

            if (slave.state == (EC_STATE_SAFE_OP + EC_STATE_ERROR)) {
                logger_->error("Slave {} is in SAFE_OP + ERROR, attempting ack", number);
                writeSlaveState(number, static_cast<ec_state>(EC_STATE_SAFE_OP + EC_STATE_ACK));
            } else if (slave.state == EC_STATE_SAFE_OP) {
                logger_->warn("Slave {} is in SAFE_OP, attempting change to OPERATIONAL", number);
                writeSlaveState(number, EC_STATE_OPERATIONAL);
            } else if (slave.state > EC_STATE_NONE) {
                logger_->info("Slave {} has no valid state, trying to reconfigure", number);
                if (soemApi_->reconfig_slave(number, EC_TIMEOUTRET)) {
                    slave.islost = FALSE;
                    logger_->info("Slave {} reconfigured", number);
                }
            } else if (!slave.islost) {
                logger_->info("Slave {} not working but not lost, trying to move to OPERATIONAL",
                    number);
                writeSlaveState(number, EC_STATE_OPERATIONAL);
                if (slave.state == EC_STATE_NONE) {
                    slave.islost = TRUE;
                    logger_->error("Slave {} lost", number);
                }
            }
            if (slave.islost) {
                logger_->error("Slave {} is lost, trying to recover", number);
                if (slave.state != EC_STATE_NONE) {
                    slave.islost = FALSE;
                    logger_->info("Slave {} found.", number);
                    continue;
                }
                if (soemApi_->recover_slave(number, EC_TIMEOUTRET)) {
                    slave.islost = FALSE;
                    logger_->info("Slave {} recovered.", number);
                }
            }
        }
        if (!soemApi_->ec_group()[0].docheckstate)
            logger_->info("OK : all slaves resumed OPERATIONAL.");
    }
}

bool EtherCATMaster::waitStateChange(
    const uint16_t& slaveID, const uint16_t& state, const std::chrono::seconds& timeout) {
    auto start = std::chrono::high_resolution_clock::now();
    while (readSlaveState(slaveID) != state) {
        std::this_thread::sleep_for(cycleTime_);
        auto end = std::chrono::high_resolution_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
        if (elapsed > timeout) {
            logger_->error("Slave {} did not change it's state to 0x{:02X}, it's still in 0x{:02X}",
                slaveID, state, readSlaveState(slaveID));
            return false;
        }
    }
    return true;
}

}  // namespace crf::devices::ethercatdrivers
