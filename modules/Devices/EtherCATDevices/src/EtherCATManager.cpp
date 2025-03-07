/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Francesco Riccardi CERN EN/SMM/MRO 2019
 *
 *  ==================================================================================================
 */

#include <memory>
#include <string>
#include <thread>

#include "EtherCATDevices/EtherCATManager.hpp"
#include "EtherCATDevices/SoemApi.hpp"

#define stack64k (64 * 1024)
#define EC_TIMEOUTMON 500
#define NSEC_PER_SEC 1000000000

namespace crf {
namespace devices {
namespace ethercatdevices {

EtherCATManager::EtherCATManager(const std::string& ifname, const uint nSlaves,
    const int sizeOfIOMap = 4096, const int cycleTime = 0, std::shared_ptr<ISoemApi> soemApi) :
    logger_("EtherCATManager"),
    sizeOfIOMap_(sizeOfIOMap),
    IOMap_(),
    SoemApi_(soemApi),
    ioMapMutex_(),
    ifname_(ifname),
    numberOfSlaves_(nSlaves),
    expectedWKC_(0),
    wkc_(0),
    initialized_(false),
    ioMapConfigured_(false),
    inOP_(false),
    cycleTime_(cycleTime),
    toff_(0),
    gl_delta_(0),
    startThreads_(false),
    threadProcessData_(),
    threadCheck_(),
    threadStartedFlag_(false),
    threadStartedMtx_(),
    threadStartedCv_(),
    dataExchanged_(false),
    dataExchangedMtx_(),
    dataExchangedCv_() {
    logger_->debug("CTor");
    if (nSlaves < 1) {
        logger_->error("Slave number invalid");
        throw std::invalid_argument("Slave number invalid, must be higher than 1");
    }
    if (SoemApi_ == nullptr)
        SoemApi_ = std::make_shared<SoemApi>();
    IOMap_ = std::make_unique<char[]>(sizeOfIOMap_);
}

EtherCATManager::~EtherCATManager() {
    logger_->debug("DTor");
    if (initialized_)
        deinitialize();
}

bool EtherCATManager::initialize() {
    logger_->info("Initialize Function");
    if (initialized_) {
        logger_->warn("Already initialized");
        return false;
    }
    if (SoemApi_->init(const_cast<char*>(ifname_.c_str())) <= 0) {
        logger_->error("No socket connection on {}", ifname_.c_str());
        logger_->error("ec_init on {} failed.", ifname_.c_str());
        logger_->error("No socket connection on {}. Excecute as root", ifname_.c_str());
        return false;
    }

    logger_->info("ec_init on {} succeeded.", ifname_.c_str());
    if (SoemApi_->config_init(FALSE) <= 0) {
        logger_->error("No Slaves found");
        return false;
    }

    if (SoemApi_->ec_slavecount() != numberOfSlaves_) {
        logger_->warn("{} slaves found and configured, {} slaves missing!",
            SoemApi_->ec_slavecount(), numberOfSlaves_ - SoemApi_->ec_slavecount());
        return false;
    }

    logger_->info("All slaves found and configured.");

    SoemApi_->statecheck(0, EC_STATE_PRE_OP,  EC_TIMEOUTSTATE * 3);
    if (SoemApi_->ec_slave()[0].state != EC_STATE_PRE_OP) {
        logger_->warn("Not all slaves reached pre-operational state.");
        SoemApi_->readstate();
        for (int i = 1; i <= SoemApi_->ec_slavecount(); i++) {
            if (SoemApi_->ec_slave()[i].state != EC_STATE_PRE_OP) {
                logger_->info("Slave {} State={} StatusCode={} : {}", i,
                    SoemApi_->ec_slave()[i].state, SoemApi_->ec_slave()[i].ALstatuscode,
                    ec_ALstatuscode2string(SoemApi_->ec_slave()[i].ALstatuscode));
            }
        }
        return false;
    }

    initialized_ = true;
    ioMapConfigured_ = false;
    return true;
}

bool EtherCATManager::deinitialize() {
    logger_->info("deinitialize");
    if (!initialized_) {
        logger_->error("Not initialized");
        return false;
    }

    startThreads_ = false;
    pthread_join(threadCheck_, NULL);
    pthread_join(threadProcessData_, NULL);

    SoemApi_->close();
    inOP_ = false;
    initialized_ = false;

    return true;
}

bool EtherCATManager::configureIOMap() {
    if (!initialized_) {
        logger_->error("Not initialized");
        return false;
    }

    int usedmem = SoemApi_->config_map(IOMap_.get());
    if (usedmem > sizeOfIOMap_) {
        logger_->error("Memory necessary ({}) for the IOMap is bigger than the one specified ({})",
            usedmem, sizeOfIOMap_);
        ioMapConfigured_ = false;
        return false;
    }
    SoemApi_->statecheck(0, EC_STATE_SAFE_OP,  EC_TIMEOUTSTATE * 3);
    if (SoemApi_->ec_slave()[0].state != EC_STATE_SAFE_OP) {
        logger_->warn("Not all slaves reached safe operational state.");
        SoemApi_->readstate();
        for (int i = 1; i <= SoemApi_->ec_slavecount(); i++) {
            if (SoemApi_->ec_slave()[i].state != EC_STATE_SAFE_OP) {
                logger_->info("Slave {} State={} StatusCode={} : {}", i,
                    SoemApi_->ec_slave()[i].state, SoemApi_->ec_slave()[i].ALstatuscode,
                    ec_ALstatuscode2string(SoemApi_->ec_slave()[i].ALstatuscode));
            }
        }
        return false;
    }

    if (SoemApi_->configdc()) {
        logger_->info("Distributed clock configured and propagation delays evaluated.");
    }
    expectedWKC_ = (SoemApi_->ec_group()[0].outputsWKC * 2) + SoemApi_->ec_group()[0].inputsWKC;
    logger_->info("Expected Working Counter: {}", expectedWKC_);

    SoemApi_->readstate();
    for (int cnt = 1; cnt <= ec_slavecount ; cnt++) {
        logger_->info("Slave:{} Name:{} Output size:{}bits Input size:{}bits State:{} delay:{}.{}",
            cnt, ec_slave[cnt].name, ec_slave[cnt].Obits, ec_slave[cnt].Ibits, ec_slave[cnt].state,
            static_cast<int>(ec_slave[cnt].pdelay), ec_slave[cnt].hasdc);
        logger_->info("Out:{},{} In:{},{}", (uintptr_t)ec_slave[cnt].outputs, ec_slave[cnt].Obytes,
            (uintptr_t)ec_slave[cnt].inputs, ec_slave[cnt].Ibytes);
    }

    startThreads_ = true;

    std::unique_lock<std::mutex> lock(threadStartedMtx_);
    threadStartedFlag_ = false;
    osal_thread_create_rt(reinterpret_cast<void*>(&(EtherCATManager::threadProcessData_)),
        stack64k * 2, reinterpret_cast<void*>(&(EtherCATManager::ecatthread)), this);
    if (!threadStartedCv_.wait_for(lock, std::chrono::seconds(5), [this](){
        return threadStartedFlag_; })) {
            logger_->warn("Thread process data did not start");
            return false;
    }

    osal_thread_create(reinterpret_cast<void*>(&(EtherCATManager::threadCheck_)),
        stack64k * 2, reinterpret_cast<void*>(&(EtherCATManager::ecatcheck)), this);
    threadStartedFlag_ = false;
    if (!threadStartedCv_.wait_for(lock, std::chrono::seconds(5), [this](){
        return threadStartedFlag_; })) {
            logger_->warn("Thread check data did not start");
            return false;
    }
    logger_->info("IOMap configured: ({}) bytes used", usedmem);

    ioMapConfigured_ = true;
    return true;
}

const std::optional<uint8_t*> EtherCATManager::retrieveOutputs(uint16_t slaveID) const {
    if (!initialized_)
        return std::nullopt;
    if (!ioMapConfigured_)
        return std::nullopt;
    if ((slaveID > SoemApi_->ec_slavecount()) || (slaveID == 0))
        return std::nullopt;
    return SoemApi_->ec_slave()[slaveID].outputs;
}

const std::optional<uint8_t*> EtherCATManager::retrieveInputs(uint16_t slaveID) const {
    if (!initialized_)
        return std::nullopt;
    if (!ioMapConfigured_)
        return std::nullopt;
    if ((slaveID > SoemApi_->ec_slavecount()) || (slaveID == 0))
        return std::nullopt;

    return SoemApi_->ec_slave()[slaveID].inputs;
}

template<typename T>
std::optional<T> EtherCATManager::readSDO(uint16_t slaveID, uint16_t index,
    uint8_t subIndex) const {
    if (!initialized_)
        return std::nullopt;
    int wkc = 0;
    T val;
    int size = sizeof(val);
    wkc = SoemApi_->SDOread(slaveID, index, subIndex, false, &size, &val, EC_TIMEOUTRXM);
    if (wkc != 1) {
        logger_->error("ReadSDO FAILED on slave number {}, index {}, subindex {}", slaveID, index,
            subIndex);
        return std::nullopt;
    }
    return val;
}

template<typename T>
bool EtherCATManager::writeSDO(uint16_t slaveID, uint16_t index, uint8_t subIndex, T val) {
    if (!initialized_)
        return false;

    int wkc = 0;
    int size = sizeof(val);
    wkc = SoemApi_->SDOwrite(slaveID, index, subIndex, false, size, &val, EC_TIMEOUTRXM);
    if (wkc != 1) {
        logger_->error("WriteSDO FAILED on slave number {}, index {}, subindex {}", slaveID, index,
            subIndex);
        return false;
    }
    return true;
}

uint16_t EtherCATManager::getSlaveState(uint16_t slaveID) {
    if (!initialized_)
        return 0;

    if (slaveID > SoemApi_->ec_slavecount())
        return 0;

    SoemApi_->readstate();
    return SoemApi_->ec_slave()[slaveID].state;
}

bool EtherCATManager::enterInit(uint16_t slaveID) {
    if (!initialized_)
        return false;

    if (slaveID > SoemApi_->ec_slavecount())
        return false;

    if (checkInit(slaveID)) {
        return true;
    }

    SoemApi_->ec_slave()[slaveID].state = EC_STATE_INIT;
    SoemApi_->writestate(slaveID);
    if (!checkInit(slaveID)) {
        logger_->warn("Slave {} can't enter in Init Mode", slaveID);
        return false;
    }

    if (slaveID == 0) {
        logger_->info("All Slaves in Init Mode");
    } else {
        logger_->info("Slave {} in Init Mode", slaveID);
    }

    SoemApi_->readstate();
    return true;
}

bool EtherCATManager::enterPreOp(uint16_t slaveID) {
    if (!initialized_)
        return false;

    if (slaveID > SoemApi_->ec_slavecount())
        return false;

    if (checkPreOp(slaveID)) {
        return true;
    }

    SoemApi_->ec_slave()[slaveID].state = EC_STATE_PRE_OP;
    SoemApi_->writestate(slaveID);
    if (!checkPreOp(slaveID)) {
        logger_->error("Slave {} can't enter in PreOp Mode", slaveID);
        return false;
    }

    if (slaveID == 0) {
        logger_->info("All Slaves in PreOp Mode");
    } else {
        logger_->error("Slave {} in PreOp Mode", slaveID);
    }
    SoemApi_->readstate();
    return true;
}

bool EtherCATManager::enterSafeOp(uint16_t slaveID) {
    if (!initialized_)
        return false;

    if (slaveID > SoemApi_->ec_slavecount())
        return false;

    if (checkSafeOp(slaveID)) {
        return true;
    }


    if (checkInit(slaveID)) {
        if (!enterPreOp(slaveID)) {
            logger_->error("Slave {} can't enter in SafeOp Mode", slaveID);
            return false;
        }
    }
    SoemApi_->ec_slave()[slaveID].state = EC_STATE_SAFE_OP;
    SoemApi_->writestate(slaveID);
    if (!checkSafeOp(slaveID)) {
        logger_->error("Slave {} can't enter in SafeOp Mode", slaveID);
        return false;
    }

    if (slaveID == 0) {
        logger_->info("All Slaves in SafeOp Mode");
    } else {
        logger_->info("Slave {} in SafeOp Mode", slaveID);
    }

    SoemApi_->readstate();
    return true;
}

bool EtherCATManager::enterOp(uint16_t slaveID) {
    if (!initialized_)
        return false;

    if (slaveID > SoemApi_->ec_slavecount())
        return false;

    if (checkOp(slaveID)) {
        return true;
    }

    if (checkInit(slaveID)) {
        if (!enterPreOp(slaveID)) {
            logger_->error("Slave {} can't enter in Op Mode", slaveID);
            return false;
        }
    }

    if (checkPreOp(slaveID)) {
        if (!enterSafeOp(slaveID)) {
            logger_->error("Slave {} can't enter in Op Mode", slaveID);
            return false;
        }
    }

    if (checkSafeOp(slaveID)) {
        SoemApi_->send_processdata();
        wkc_ = SoemApi_->receive_processdata(EC_TIMEOUTRET);
        SoemApi_->ec_slave()[slaveID].state = EC_STATE_OPERATIONAL;
        SoemApi_->writestate(slaveID);
        if (!checkOp(slaveID)) {
            logger_->error("Slave {} can't enter in Op Mode", slaveID);
            return false;
        }
    } else {
        return false;
    }

    SoemApi_->readstate();
    if (slaveID == 0) {
        logger_->info("All Slaves in Op Mode");
        std::this_thread::sleep_for(std::chrono::seconds(1));
        inOP_ = true;
    } else {
        logger_->info("Slave {} in Op Mode", slaveID);
    }

    return true;
}

bool EtherCATManager::slaveCommunicationCheck(uint16_t slaveID) {
    if (!initialized_)
        return false;

    if ((slaveID > SoemApi_->ec_slavecount()) || (slaveID == 0))
        return false;

    if (!ioMapConfigured_) return false;
    if (!inOP_) return false;
    if (SoemApi_->ec_slave()[slaveID].state == EC_STATE_OPERATIONAL) {
        return true;
    }
    return false;
}

bool EtherCATManager::checkInit(uint16_t slaveID) {
    if (!initialized_)
        return false;

    if (slaveID > SoemApi_->ec_slavecount())
        return false;

    return (SoemApi_->statecheck(slaveID, EC_STATE_INIT, EC_TIMEOUTSTATE) == EC_STATE_INIT);
}

bool EtherCATManager::checkPreOp(uint16_t slaveID) {
    if (!initialized_)
        return false;

    if (slaveID > SoemApi_->ec_slavecount())
        return false;

    return (SoemApi_->statecheck(slaveID, EC_STATE_PRE_OP, EC_TIMEOUTSTATE) == EC_STATE_PRE_OP);
}

bool EtherCATManager::checkSafeOp(uint16_t slaveID) {
    if (!initialized_)
        return false;

    if (slaveID > SoemApi_->ec_slavecount())
        return false;

    return (SoemApi_->statecheck(slaveID, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE) == EC_STATE_SAFE_OP);
}

bool EtherCATManager::checkOp(uint16_t slaveID) {
    if (!initialized_)
        return false;

    if (slaveID > SoemApi_->ec_slavecount())
        return false;

    return (SoemApi_->statecheck(slaveID, EC_STATE_OPERATIONAL, EC_TIMEOUTSTATE) ==
        EC_STATE_OPERATIONAL);
}

void EtherCATManager::add_timespec(struct timespec *ts, int64_t addtime) {
    int64_t sec, nsec;
    nsec = addtime % NSEC_PER_SEC;
    sec = (addtime - nsec) / NSEC_PER_SEC;
    ts->tv_sec += sec;
    ts->tv_nsec += nsec;
    if (ts->tv_nsec > NSEC_PER_SEC) {
        nsec = ts->tv_nsec % NSEC_PER_SEC;
        ts->tv_sec += (ts->tv_nsec - nsec) / NSEC_PER_SEC;
        ts->tv_nsec = nsec;
    }
}

void EtherCATManager::syncEC(int64_t reftime, int64_t cycletime, int64_t *offsettime) {
    static int64_t integral = 0;
    int64_t delta;
    delta = (reftime) % cycletime;
    if (delta > (cycletime / 2)) { delta = delta - cycletime; }
    if (delta > 0) { integral++; }
    if (delta < 0) { integral--; }
    *offsettime = -(delta / 100) - (integral / 20);
    gl_delta_ = delta;
}

void EtherCATManager::ecatthread(EtherCATManager* ptr) {
    {
        std::unique_lock<std::mutex> lock(ptr->threadStartedMtx_);
        ptr->threadStartedFlag_ = true;
        ptr->threadStartedCv_.notify_all();
    }
    struct timespec   ts, tleft;
    int ht;
    int64_t cycletime;
    ptr->toff_ = 0;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    ht = (ts.tv_nsec / 1000000) + 1;
    ts.tv_nsec = ht * 1000000;
    int dorun = 0;
    cycletime = ptr->cycleTime_ * 1000;
    ptr->SoemApi_->send_processdata();
    while (ptr->startThreads_) {
        dorun++;
        ptr->add_timespec(&ts, cycletime + ptr->toff_);
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, &tleft);
        std::unique_lock<std::mutex> lock(ptr->dataExchangedMtx_);
        ptr->wkc_ = ptr->SoemApi_->receive_processdata(EC_TIMEOUTRET);
        if (ptr->SoemApi_->ec_slave()[0].hasdc) ptr->syncEC(ec_DCtime, cycletime, &(ptr->toff_));
        ptr->SoemApi_->send_processdata();
        ptr->dataExchanged_ = true;
        ptr->dataExchangedCv_.notify_all();
    }
}

void EtherCATManager::ecatcheck(EtherCATManager* ptr) {
    crf::utility::logger::EventLogger logger("EtherCATManager");
    {
        std::unique_lock<std::mutex> lock(ptr->threadStartedMtx_);
        ptr->threadStartedFlag_ = true;
        ptr->threadStartedCv_.notify_all();
    }
    int slave = 1;
    while (ptr->startThreads_) {
        osal_usleep(10000);
        if (!ptr->inOP_ || !((ptr->wkc_ < ptr->expectedWKC_) ||
            ptr->SoemApi_->ec_group()[0].docheckstate)) {
                continue;
        }
        ptr->SoemApi_->ec_group()[0].docheckstate = FALSE;
        ptr->SoemApi_->readstate();
        for (slave = 1; slave <= ptr->SoemApi_->ec_slavecount(); slave++) {
            if (!(ptr->SoemApi_->ec_slave()[slave].group == 0) ||
                !(ptr->SoemApi_->ec_slave()[slave].state != EC_STATE_OPERATIONAL)) {
                    continue;
            }

            ptr->SoemApi_->ec_group()[0].docheckstate = TRUE;
            if (ptr->SoemApi_->ec_slave()[slave].state == (EC_STATE_SAFE_OP + EC_STATE_ERROR)) {
                logger->error("Slave {} is in SAFE_OP + ERROR, attempting ack.", slave);
                ptr->SoemApi_->ec_slave()[slave].state = (EC_STATE_SAFE_OP + EC_STATE_ACK);
                ptr->SoemApi_->writestate(slave);
            } else if (ptr->SoemApi_->ec_slave()[slave].state == EC_STATE_SAFE_OP) {
                logger->warn("Slave {} is in SAFE_OP, change to OPERATIONAL.", slave);
                ptr->SoemApi_->ec_slave()[slave].state = EC_STATE_OPERATIONAL;
                ptr->SoemApi_->writestate(slave);
            } else if (ptr->SoemApi_->ec_slave()[slave].state > EC_STATE_NONE) {
                if (ptr->SoemApi_->reconfig_slave(slave, EC_TIMEOUTMON)) {
                ptr->SoemApi_->ec_slave()[slave].islost = FALSE;
                logger->info("Slave {} reconfigured.", slave);
                }
            } else if (!ptr->SoemApi_->ec_slave()[slave].islost) {
                ptr->SoemApi_->statecheck(slave, EC_STATE_OPERATIONAL, EC_TIMEOUTRET);
                if (ptr->SoemApi_->ec_slave()[slave].state == EC_STATE_NONE) {
                    ptr->SoemApi_->ec_slave()[slave].islost = TRUE;
                    logger->error("Slave {} lost.", slave);
                }
            }
            if (ptr->SoemApi_->ec_slave()[slave].islost) {
                if (ptr->SoemApi_->ec_slave()[slave].state == EC_STATE_NONE) {
                    if (ptr->SoemApi_->recover_slave(slave, EC_TIMEOUTMON)) {
                        ptr->SoemApi_->ec_slave()[slave].islost = FALSE;
                        logger->info("Slave {} recovered.", slave);
                    }
                } else {
                    ptr->SoemApi_->ec_slave()[slave].islost = FALSE;
                    logger->info("Slave {} found.", slave);
                }
            }
        }
        if (!ptr->SoemApi_->ec_group()[0].docheckstate)
        logger->info("OK : all slaves resumed OPERATIONAL.");
    }
}

template std::optional<uint8_t> EtherCATManager::readSDO<uint8_t>(
    uint16_t, uint16_t, uint8_t) const;
template std::optional<int8_t> EtherCATManager::readSDO<int8_t>(
    uint16_t, uint16_t, uint8_t) const;
template std::optional<uint16_t> EtherCATManager::readSDO<uint16_t>(
    uint16_t, uint16_t, uint8_t) const;
template std::optional<int16_t> EtherCATManager::readSDO<int16_t>(
    uint16_t, uint16_t, uint8_t) const;
template std::optional<uint32_t> EtherCATManager::readSDO<uint32_t>(
    uint16_t, uint16_t, uint8_t) const;
template std::optional<int32_t> EtherCATManager::readSDO<int32_t>(
    uint16_t, uint16_t, uint8_t) const;

template bool EtherCATManager::writeSDO<uint8_t>(
    uint16_t, uint16_t, uint8_t, uint8_t);
template bool EtherCATManager::writeSDO<int8_t>(
    uint16_t, uint16_t, uint8_t, int8_t);
template bool EtherCATManager::writeSDO<uint16_t>(
    uint16_t, uint16_t, uint8_t, uint16_t);
template bool EtherCATManager::writeSDO<int16_t>(
    uint16_t, uint16_t, uint8_t, int16_t);
template bool EtherCATManager::writeSDO<uint32_t>(
    uint16_t, uint16_t, uint8_t, uint32_t);
template bool EtherCATManager::writeSDO<int32_t>(
    uint16_t, uint16_t, uint8_t, int32_t);

}  // namespace ethercatdevices
}  // namespace devices
}  // namespace crf
