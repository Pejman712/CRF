/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Francesco Riccardi CERN EN/SMM/MRO 2019
 *
 *  ==================================================================================================
 */
#pragma once

#include <soem/ethercat.h>

#include "SOEMAPI/ISOEMAPI.hpp"

namespace crf::communication::soemapi {

/**
 * @ingroup group_soemapi
 * @brief API class of SOEM to access the functions of the library.
 *
 */
class SOEMAPI : public ISOEMAPI {
 public:
    SOEMAPI() = default;
    SOEMAPI(const SOEMAPI&) = delete;
    SOEMAPI(SOEMAPI&&) = delete;
    ~SOEMAPI() override = default;

    int init(char* ifname) override;
    int config_init(uint8_t usetable) override;
    int config_overlap_map(void* pIOmap) override;
    int ec_slavecount() override;
    ec_slavet* ec_slave() override;
    ec_groupt* ec_group() override;
    int send_overlap_processdata() override;
    int receive_processdata(int timeout) override;
    int SDOread(uint16_t slave, uint16_t index, uint8_t subindex,
        bool CA, int *psize, void *p, int timeout) override;
    int SDOwrite(uint16_t slave, uint16_t index, uint8_t subindex,
        bool CA, int psize, void *p, int timeout) override;
    uint16_t statecheck(uint16_t slave, uint16_t reqstate, int timeout) override;
    int writestate(uint16_t slave) override;
    int readstate() override;
    void close() override;
    int reconfig_slave(uint16_t slave, int timeout) override;
    int recover_slave(uint16_t slave, int timeout) override;
    bool configdc() override;
};

}  // namespace crf::communication::soemapi
