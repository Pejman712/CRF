/* © Copyright CERN 2020. All rights reserved. This software is released under a CERN proprietary software license. 
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 * 
 * Author: Giacomo LunghiCERN EN/SMM/MRO 2020
 * 
 *  ==================================================================================================
 */

#pragma once

#include "EtherCATDevices/ISoemApi.hpp"

namespace crf {
namespace devices {
namespace ethercatdevices {

class SoemApiMock : public ISoemApi {
 public:
    MOCK_METHOD1(init,
        int(char * ifname));
    MOCK_METHOD1(config_init,
        int(uint8_t usetable));
    MOCK_METHOD0(ec_slavecount,
        int());
    MOCK_METHOD0(ec_slave,
        ec_slavet*());
    MOCK_METHOD0(ec_group,
        ec_groupt*());
    MOCK_METHOD1(config_map,
        int(void* pIOmap));
    MOCK_METHOD0(send_processdata,
        int());
    MOCK_METHOD1(receive_processdata,
        int(int timeout));
    MOCK_METHOD7(SDOread,
        int(uint16_t slave, uint16_t index, uint8_t subindex, bool CA, int *psize, void *p, int timeout));  // NOLINT
    MOCK_METHOD7(SDOwrite,
        int(uint16_t Slave, uint16_t Index, uint8_t SubIndex, bool CA, int psize, void *p, int Timeout));  // NOLINT
    MOCK_METHOD3(statecheck,
        uint16_t(uint16_t slave, uint16_t reqstate, int timeout));
    MOCK_METHOD1(writestate,
        int(uint16_t slave));
    MOCK_METHOD0(readstate,
        int());
    MOCK_METHOD0(close,
        void());
    MOCK_METHOD2(reconfig_slave,
        int(uint16_t slave, int timeout));
    MOCK_METHOD2(recover_slave,
        int(uint16_t slave, int timeout));
    MOCK_METHOD0(configdc,
        bool());
};

}  // namespace ethercatdevices
}  // namespace devices
}  // namespace crf
