/* © Copyright CERN 2017. All rights reserved. This software is released under a CERN proprietary software license. 
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 * 
 * Author: Pawel Ptasznik CERN EN/STI/ECE
 * 
 *  ==================================================================================================
 */

#pragma once

#include <memory>
#include <mutex>
#include <thread>
#include <vector>

#include <libirimager/IRDevice.h>
#include <libirimager/IRImager.h>
#include <libirimager/IRImagerClient.h>

#include "IPC/IPC.hpp"
#include "EventLogger/EventLogger.hpp"
#include "ThermalCamera/IOptrisDeviceFactory.hpp"
#include "ThermalCamera/IThermalCamera.hpp"
#include "ThermalCamera/ThermalCameraPacket.hpp"

#include "IPC/MMAP.hpp"

namespace crf {
namespace sensors {
namespace thermalcamera {

class OptrisPiImagerClient: public evo::IRImagerClient {
 public:
    explicit OptrisPiImagerClient(std::shared_ptr<evo::IRImager> imager);
    ~OptrisPiImagerClient() override = default;

    /**
    * Callback method for raw frame events. The method is called when new data is acquired from device.
    * I think this is the only callback actually triggered by the device.
    * @param[in] data raw data
    * @param[in] size size of raw data in bytes
    */
    void onRawFrame(unsigned char* data, int size) override;
    /**
    * Callback method for thermal frames. Triggered by imager_ just after call to "process"
    * @param[in] data thermal image
    * @param[in] w width of thermal image
    * @param[in] h height of thermal image
    * @param[in] meta meta data container
    * @param[in] arg user arguments (passed to process method of IRImager class)
    *   which is basically an ugly hack, please don't use
    */
    void onThermalFrame(unsigned int16* data, unsigned int w, unsigned int h,
        evo::IRFrameMetadata meta, void* arg) override;
    void onThermalFrameEvent(unsigned int16* data, unsigned int w, unsigned int h,
        evo::IRFrameMetadata meta, void* arg) override;
    void onVisibleFrame(unsigned char* data, unsigned int w, unsigned int h,
        evo::IRFrameMetadata meta, void* arg) override;
    void onVisibleFrameEvent(unsigned char* data, unsigned int w, unsigned int h,
        evo::IRFrameMetadata meta, void* arg) override;
    void onFlagStateChange(evo::EnumFlagState flagstate, void* arg) override;

    std::vector<unsigned char> getRawData();
    std::vector<float> getThermalData();
    unsigned int getWidth();
    unsigned int getHeight();

 private:
    utility::logger::EventLogger logger_;
    std::shared_ptr<evo::IRImager> imager_;
    std::vector<unsigned char> rawData_;
    std::vector<float> thermalData_;
    unsigned int width_;
    unsigned int height_;
    std::mutex lock_;
};

class OptrisPi: public IThermalCamera {
 public:
    OptrisPi() = delete;
    ~OptrisPi() override;
    /**
     * @param ipc - used by method publishFrame to send a thermal data packet
     * @param factory - factory class used to obtain Optris IRDevice interface
     * and IRImager processing chain
     */
    explicit OptrisPi(std::shared_ptr<IPC> ipc, std::shared_ptr<IOptrisDeviceFactory> factory);
    OptrisPi(const OptrisPi&) = delete;

    /*
     * Overrides from IInitializable interface
     */
    bool initialize() override;
    bool deinitialize() override;

    cv::Mat getFrame() override;
    /*
     * I am not sure what format should I use to actually send this frame
     * through IPC. Protobuf approach seems nice, but the conversion takes some time.
     */
    bool publishFrame(const cv::Mat& frame) override;

    bool showFrame() override;
    bool streamVideo() override;

 private:
    // arg passed as a pointer, because cv::Mat requires non-const access to the underlying data
    void showOpenCvFrames(std::vector<float>* thermalData);

    utility::logger::EventLogger logger_;
    std::shared_ptr<IPC> ipc_;
    std::shared_ptr<IOptrisDeviceFactory> factory_;
    std::shared_ptr<evo::IRDevice> device_;
    std::unique_ptr<OptrisPiImagerClient> callbackClient_;
    std::thread captureImageThread_;
};

}  // namespace thermalcamera
}  // namespace sensors
}  // namespace crf
