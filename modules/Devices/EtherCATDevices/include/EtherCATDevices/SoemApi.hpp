/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license. 
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 * 
 * Author: Francesco Riccardi CERN EN/SMM/MRO 2019
 * 
 *  ==================================================================================================
 */

#pragma once

#include <soem/ethercat.h>

#include "EtherCATDevices/ISoemApi.hpp"

namespace crf {
namespace devices {
namespace ethercatdevices {

class SoemApi : public ISoemApi {
 public:
    SoemApi() = default;
    SoemApi(const SoemApi&) = delete;
    SoemApi(SoemApi&&) = delete;
    ~SoemApi() override = default;

    int config_init(uint8_t usetable) override;
    int config_map(void* pIOmap) override;
    int ec_slavecount() override;
    ec_slavet* ec_slave() override;
    ec_groupt* ec_group() override;
    int init(char* ifname) override;
    int send_processdata() override;
    int receive_processdata(int timeout) override;
    int SDOread(uint16_t slave, uint16_t index, uint8_t subindex, bool CA, int *psize, void *p,
        int timeout) override;
    int SDOwrite(uint16_t Slave, uint16_t Index, uint8_t SubIndex, bool CA, int psize, void *p,
        int Timeout) override;
    uint16_t statecheck(uint16_t slave, uint16_t reqstate, int timeout) override;
    int writestate(uint16_t slave) override;
    int readstate() override;
    void close() override;
    int reconfig_slave(uint16_t slave, int timeout) override;
    int recover_slave(uint16_t slave, int timeout) override;
    bool configdc() override;
};

}  // namespace ethercatdevices
}  // namespace devices
}  // namespace crf

