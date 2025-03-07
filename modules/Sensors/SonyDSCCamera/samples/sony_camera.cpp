/* © Copyright CERN 2018. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Giacomo Lunghi CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#include <memory>
#include <iostream>

#include "IPC/FIFO.hpp"
#include "IPC/MMAP.hpp"
#include "SonyDSCCamera/SonyDSCCamera.hpp"

int main(int argc, char* argv[]) {
    std::shared_ptr<FIFO> command_input_fifo = FIFO::CreateReaderPtr(argv[1]);
    std::shared_ptr<MMAP> output_live_mmap = MMAP::CreateWriterPtr(argv[2], 2000000);
    std::shared_ptr<FIFO> command_output_fifo = FIFO::CreateWriterPtr(argv[3]);
    command_input_fifo->open();
    output_live_mmap->open();
    command_output_fifo->open();
    crf::sensors::SonyDSCCamera* camera = new crf::sensors::SonyDSCCamera(
        command_input_fifo, output_live_mmap, command_output_fifo);
    if (camera->open()) {
        printf("Camera correctly open\n");
    } else {
        printf("Could not open the camera\n");
        return -1;
    }
    getchar();
    if (camera->close()) {
        printf("Camera correctly closed\n");
    }
    return 0;
}
