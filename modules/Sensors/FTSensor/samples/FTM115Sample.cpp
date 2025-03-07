/* © Copyright CERN 2018. All rights reserved. This software is released under a CERN proprietary software licence.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Zsolt Pasztori EN/SMM/MRO 2018
 *
 *  ==================================================================================================
 */

#include <memory>
#include <iostream>
#include <thread>
#include <fstream>

#include <CANSocket/CANSocket.hpp>
#include <FTSensor/FTM115/FTM115.hpp>

/*
 * Simple application to print some sensor measurements from the program
 */
int main(int argc, char* argv[]) {
    std::shared_ptr<crf::communication::cansocket::CANSocket> can =
        std::make_shared<crf::communication::cansocket::CANSocket>(argv[1]);
    crf::sensors::ftsensor::FTM115 ftSensor(can);
    ftSensor.initialize();
    while (true) {
        std::this_thread::sleep_for(std::chrono::seconds(10));
        auto ft = ftSensor.getFT();
        std::cout << ft <<std::endl;
    }
    return 0;
}
