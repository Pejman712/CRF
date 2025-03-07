/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Francesco Riccardi CERN EN/SMM/MRO 2019
 *
 *  ==================================================================================================
 */

#include <soem/ethercat.h>

#include "EtherCATDevices/SoemApi.hpp"

namespace crf {
namespace devices {
namespace ethercatdevices {

int SoemApi::config_init(uint8_t usetable) {
    return ec_config_init(usetable);
}

int SoemApi::config_map(void* pIOmap) {
    return ec_config_overlap_map(pIOmap);
}

int SoemApi::ec_slavecount() {
    return ::ec_slavecount;
}

ec_slavet* SoemApi::ec_slave() {
    return ::ec_slave;
}

ec_groupt* SoemApi::ec_group() {
    return ::ec_group;
}

int SoemApi::init(char * ifname) {
    return ec_init(ifname);
}

int SoemApi::send_processdata() {
    return ec_send_overlap_processdata();
}

int SoemApi::receive_processdata(int timeout) {
    return ec_receive_processdata(timeout);
}

int SoemApi::SDOread(uint16_t slave, uint16_t index, uint8_t subindex, bool CA, int *psize,
    void *p, int timeout) {
    return ec_SDOread(slave, index, subindex, CA, psize, p, timeout);
}

int SoemApi::SDOwrite(uint16_t Slave, uint16_t Index, uint8_t SubIndex, bool CA, int psize,
    void *p, int Timeout) {
    return ec_SDOwrite(Slave, Index, SubIndex, CA, psize, p, Timeout);
}

uint16_t SoemApi::statecheck(uint16_t slave, uint16_t reqstate, int timeout) {
    return ec_statecheck(slave, reqstate, timeout);
}

int SoemApi::writestate(uint16_t slave) {
    return ec_writestate(slave);
}

int SoemApi::readstate() {
    return ec_readstate();
}

void SoemApi::close() {
    return ec_close();
}

int SoemApi::reconfig_slave(uint16_t slave, int timeout) {
    return ec_reconfig_slave(slave, timeout);
}

int SoemApi::recover_slave(uint16_t slave, int timeout) {
    return ec_recover_slave(slave, timeout);
}

bool SoemApi::configdc() {
    return ec_configdc();
}

}  // namespace ethercatdevices
}  // namespace devices
}  // namespace crf
