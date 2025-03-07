/* © Copyright CERN 2023. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Natalija Topalovic 2023 BE/CEM/MRO
 *
 *  ==================================================================================================
 */
#pragma once

#include <cstdint>

#include <soem/ethercat.h>

namespace crf::communication::soemapi {

/**
 * @ingroup group_soemapi
 * @brief The ISOEMAPI interface is the wrapper on the SOEM library which expose all the functions
 *        of the SOEM to the user. It is mainly used in order to mock the SOEM Library for the
 *        tests.
 *
 */
class ISOEMAPI {
 public:
    virtual ~ISOEMAPI() = default;

    /**
     * @brief Initialise SOEM Lib.
     * @param ifname = Dev name, f.e. "eth0".
     * @return >0 if OK.
     */
    virtual int init(char* ifname) = 0;

    /**
     * @brief Enumerate and init all slaves.
     * @param usetable = TRUE when using configtable to init slaves, FALSE otherwise (we use
     *        FALSE).
     * @return Workcounter of slave discover datagram = number of slaves found.
     */
    virtual int config_init(uint8_t usetable) = 0;

    /**
     * @brief Returns the number of slaves found.
     * @return The number of slaves found.
     */
    virtual int ec_slavecount() = 0;

    /**
     * @brief Returns the pointer to the ec_slave array.
     * @return The pointer to the ec_slave array.
     */
    virtual ec_slavet* ec_slave() = 0;

    /**
     * @brief Returns the pointer to the ec_group array.
     * @return The pointer to the ec_group array.
     */
    virtual ec_groupt* ec_group() = 0;

    /**
     * @brief Map all Inputs and Outputs from slaves to IOmap in sequential order.
     * @param pIOmap = pointer to IOmap.
     * @return IOmap size.
     */
    virtual int config_overlap_map(void* pIOmap) = 0;

    /**
     * @brief Transmit processdata to slaves. Both the input and output processdata are
     *        transmitted. The outputs with the actual data, the inputs have a placeholder. The
     *        inputs are gathered with the receive_processdata function. If the processdata does
     *        not fit in one datagram, multiple are used. In order to recombine the slave response,
     *        a stack is used. / FIFO
     * @return >0 if processdata is transmitted.
     */
    virtual int send_overlap_processdata() = 0;

    /**
     * @brief Receive processdata from slaves. Second part from send_processdata(). Received
     *        datagrams are recombined with the processdata with help from the stack. If a datagram
     *        contains input processdata it copies it to the processdata structure.
     * @param timeout = Timeout in micros.
     * @return Work counter.
     */
    virtual int receive_processdata(int timeout) = 0;

    /**
     * @brief CoE SDO read, blocking. Single subindex or Complete Access.
     * @param slave = Slave number.
     * @param index = Index to read.
     * @param subindex = Subindex to read, must be 0 or 1 if CA is used.
     * @param CA = FALSE = single subindex. TRUE = Complete Access, all subindexes read.
     * @param psize	= Size in bytes of parameter buffer, returns bytes read from SDO.
     * @param p = Pointer to parameter buffer.
     * @param timeout = Timeout in us, standard is EC_TIMEOUTRXM.
     * @return Workcounter from last slave response.
     */
    virtual int SDOread(uint16_t slave, uint16_t index, uint8_t subindex,
        bool CA, int *psize, void *p, int timeout) = 0;

    /**
     * @brief CoE SDO write, blocking. Single subindex or Complete Access.
     * @param slave = Slave number.
     * @param index = Index to read.
     * @param subindex = Subindex to read, must be 0 or 1 if CA is used.
     * @param CA = FALSE = single subindex. TRUE = Complete Access, all subindexes read.
     * @param psize	= Size in bytes of parameter buffer, returns bytes read from SDO.
     * @param p = Pointer to parameter buffer.
     * @param timeout = Timeout in us, standard is EC_TIMEOUTRXM.
     * @return Workcounter from last slave response.
     */
    virtual int SDOwrite(uint16_t slave, uint16_t index, uint8_t subindex,
        bool CA, int psize, void *p, int timeout) = 0;

    /**
     * @brief Check actual slave state. This is a blocking function.
     * @param slave = Slave number, 0 = all slaves.
     * @param reqstate = Requested state.
     * @param timeout = Timeout value in us.
     * @return Requested state, or found state after timeout.
     */
    virtual uint16_t statecheck(uint16_t slave, uint16_t reqstate, int timeout) = 0;

    /**
     * @brief Write slave state, if slave = 0 then write to all slaves. The function does not check
     *        if the actual state is changed.
     * @param: slave - number of slave
     * @return 0.
     */
    virtual int writestate(uint16_t slave) = 0;

    /**
     * @brief Read all slave states in ec_slave.
     * @return lowest state found.
     */
    virtual int readstate() = 0;

    /**
     * @brief Close SOEM lib.
     */
    virtual void close() = 0;

    /**
     * @brief Reconfigure slave. slave.
     * @param slave = Slave to reconfigure.
     * @param timeout = local timeout f.e. EC_TIMEOUTRET3.
     * @return >0 if successful.
     */
    virtual int reconfig_slave(uint16_t slave, int timeout) = 0;

    /**
     * @brief Recover slave.
     * @param slave = Slave to recover.
     * @param timeout = local timeout f.e. EC_TIMEOUTRET3.
     * @return >0 if successful.
     */
    virtual int recover_slave(uint16_t slave, int timeout) = 0;

    /**
     * @brief Locate DC slaves, measure propagation delays.
     * @return true if slaves are found with DC.
     */
    virtual bool configdc() = 0;
};

}  // namespace crf::communication::soemapi
