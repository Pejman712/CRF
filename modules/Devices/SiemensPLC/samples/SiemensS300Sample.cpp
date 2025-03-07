/* Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Giacomo Lunghi CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */
#include <iostream>
#include <string>

#include "SiemensPLC/SiemensPLCS7.hpp"
#include "SiemensPLC/SiemensPLCTypeConverter.hpp"

int main(int argc, char* argv[]) {
    crf::devices::siemensplc::SiemensPLCS7 plc("192.168.0.40", 0, 2);
    if (!plc.initialize()) {
        std::cout << "Could not connect to PLC" << std::endl;
        return -1;
    }

    auto dbBytes = plc.readDB(511, 22);
    float position = crf::devices::siemensplc::SiemensPLCTypeConverter::getFloat(dbBytes, 0);
    std::cout << "Train position = " << position << std::endl;
    float odometer = crf::devices::siemensplc::SiemensPLCTypeConverter::getFloat(dbBytes, 18);
    std::cout << "Odometer value = " << odometer << std::endl;

    if (!plc.deinitialize()) {
        std::cout << "Could not disconnect from PLC" << std::endl;
        return -1;
    }
    return 0;
}
