/* @Author: Pawel Ptasznik
 * @Copyright CERN 2017
 */

#include <memory>
#include <string>
#include <sstream>
#include <thread>

#include "EventLogger/EventLogger.hpp"
#include "XsensMT/XsensMT.hpp"

#define MMAP_FILENAME "/tmp/mmap_imuXsens"
#define INTERVAL_MS 1

int main(int argc, char** argv) {
    std::unique_ptr<crf::sensors::imu::XsensMT> sensorAdapter;
    std::string mmapFilename;
    int intervalMs;
    crf::utility::logger::EventLogger logger("XsensMTExecutable");
    if (argc == 1) {
        intervalMs = INTERVAL_MS;
        mmapFilename = MMAP_FILENAME;
        logger->info("Executed without arguments. Starting with default values.");

    } else if (argc == 3) {
        mmapFilename = argv[1];
        std::stringstream strValue;
        strValue << argv[2];
        strValue >> intervalMs;
    } else {
        logger->warn("Incorrect number of arguments: {}", argc);
        logger->warn("Correct syntax is: ./XsensMti200 <MMAP_FILENAME> <INTERVAL>");
        logger->warn("Example.: ./XsensMti200 /tmp/mmap_imuXsensMti200 100");
        return 0;
    }

    logger->info("MMAP filename: {}", mmapFilename);
    logger->info("Interval: {}", intervalMs);

    sensorAdapter.reset(new crf::sensors::imu::XsensMT(XsControl::construct()));

    if (!sensorAdapter->initialize()) {
        logger->warn("Device initialization failed!");
        return 0;
    }

    while (true) {
        if (sensorAdapter->hasFreshData()) {
            sensorAdapter->getIMUDataPacket();
        }
        std::this_thread::sleep_for(std::chrono::duration<double, std::milli>(intervalMs));
    }

    return 0;
}
