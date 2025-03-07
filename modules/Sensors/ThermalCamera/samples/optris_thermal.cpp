#include <iostream>
#include <memory>

#include <opencv2/opencv.hpp>

#include <IPC/MMAP.hpp>
#include <ThermalCamera/OptrisDeviceFactory.hpp>
#include <ThermalCamera/OptrisPi.hpp>


int main() {
    std::shared_ptr<crf::sensors::thermalcamera::OptrisDeviceFactory> CameraFactory (new crf::sensors::thermalcamera::OptrisDeviceFactory("/home/robotronics/optris.cfg"));

    std::shared_ptr<MMAP> mmap = MMAP::CreateWriterPtr("/tmp/mmap_thermal", 440064);
    crf::sensors::thermalcamera::OptrisPi therm_camera (mmap, CameraFactory);

    while(true) {
        therm_camera.publishFrame(therm_camera.getFrame());
        usleep(100000);
    }

    return 0;
}