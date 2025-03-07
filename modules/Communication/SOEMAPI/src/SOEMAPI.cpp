/* © Copyright CERN 2023. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Jorge Playan Garai CERN EN/SMM/MRO 2023
 *
 *  ==================================================================================================
 */

#include <soem/ethercat.h>

#include "SOEMAPI/SOEMAPI.hpp"

namespace crf::communication::soemapi {

int SOEMAPI::init(char* ifname) {
    return ec_init(ifname);
}

int SOEMAPI::config_init(uint8_t usetable) {
    return ec_config_init(usetable);
}

int SOEMAPI::config_overlap_map(void* pIOmap) {
    return ec_config_overlap_map(pIOmap);
}

int SOEMAPI::ec_slavecount() {
    return ::ec_slavecount;
}

ec_slavet* SOEMAPI::ec_slave() {
    return ::ec_slave;
}

ec_groupt* SOEMAPI::ec_group() {
    return ::ec_group;
}

int SOEMAPI::send_overlap_processdata() {
    return ec_send_overlap_processdata();
}

int SOEMAPI::receive_processdata(int timeout) {
    return ec_receive_processdata(timeout);
}

int SOEMAPI::SDOread(uint16_t slave, uint16_t index, uint8_t subindex,
    bool CA, int *psize, void *p, int timeout) {
    return ec_SDOread(slave, index, subindex, CA, psize, p, timeout);
}

int SOEMAPI::SDOwrite(uint16_t slave, uint16_t index, uint8_t subindex,
    bool CA, int psize, void *p, int timeout) {
    return ec_SDOwrite(slave, index, subindex, CA, psize, p, timeout);
}

uint16_t SOEMAPI::statecheck(uint16_t slave, uint16_t reqstate, int timeout) {
    return ec_statecheck(slave, reqstate, timeout);
}

int SOEMAPI::writestate(uint16_t slave) {
    return ec_writestate(slave);
}

int SOEMAPI::readstate() {
    return ec_readstate();
}

void SOEMAPI::close() {
    return ec_close();
}

int SOEMAPI::reconfig_slave(uint16_t slave, int timeout) {
    return ec_reconfig_slave(slave, timeout);
}

int SOEMAPI::recover_slave(uint16_t slave, int timeout) {
    return ec_recover_slave(slave, timeout);
}

bool SOEMAPI::configdc() {
    return ec_configdc();
}

}  // namespace crf::communication::soemapi
