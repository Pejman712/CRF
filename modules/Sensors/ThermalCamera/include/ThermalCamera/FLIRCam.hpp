/* © Copyright CERN 2018. All rights reserved. This software is released under a CERN proprietary software license. 
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 * 
 * Author: Leanne Attard EN-SMM-MRO
 * 
 *  ==================================================================================================
 */

#pragma once


#define _UNIX_  // TODO  // NOLINT

// ebus
#include <PvSampleUtils.h>
#include <PvDevice.h>
#include <PvDeviceGEV.h>
#include <PvDeviceU3V.h>
#include <PvStream.h>
#include <PvStreamGEV.h>
#include <PvStreamU3V.h>
#include <PvPipeline.h>
#include <PvBuffer.h>

// opencv
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"

// general
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

// ipc
#include "IPC/IPC.hpp"
#include "IPC/MMAP.hpp"

// general
#include "EventLogger/EventLogger.hpp"
#include "ThermalCamera/IThermalCamera.hpp"
#include "ThermalCamera/ThermalCameraPacket.hpp"

// flir
#include "ThermalCamera/FLIRFactory.hpp"
#include "ThermalCamera/FLIRDeviceParams.hpp"
#include "ThermalCamera/FLIRTempConverter.hpp"

namespace crf {
namespace sensors {
namespace thermalcamera {

class FLIRCam: public IThermalCamera {
 public:
    FLIRCam() = delete;
    ~FLIRCam() override;
    /**
     * @param ipc - used by method publishFrame to send a thermal data packet
     * @param factory - factory class used to obtain Optris IRDevice interface
     * and IRImager processing chain
     */
    explicit FLIRCam(std::shared_ptr<IPC> ipc, std::shared_ptr<IFLIRFactory> factory);
    FLIRCam(const FLIRCam&) = delete;

    bool initialize() override;
    bool deinitialize() override;
    cv::Mat getFrame() override;
    bool publishFrame(const cv::Mat& frame) override;
    bool showFrame() override;
    bool streamVideo() override;

 private:
    PvDevice *device_;
    PvPipeline *lPipeline;
    PvStream *lStream;
    bool pipelineStarted;
    bool streaming;
    bool deinitialized;
    bool initialized;
    PvGenParameterArray *lDeviceParams;
    PvGenParameterArray *lStreamParams;
    TempConverter*  temp_converter;
    std::shared_ptr<IFLIRFactory> factory_;
    DeviceParams* params;

    PvDevice * ConnectToDevice(const PvString &aConnectionID);
    PvStream * OpenStream(const PvString &aConnectionID);
    void ConfigureStream(PvDevice *aDevice, PvStream *aStream);
    bool getSingleFrame(cv::Mat& rawImage, cv::Mat& thermalImage);  // NOLINT
    cv::Mat rawToHeat(const cv::Mat raw);
    bool createPipelineAndStartStream();

    utility::logger::EventLogger logger_;
    std::shared_ptr<IPC> ipc_;
};

}  // namespace thermalcamera
}  // namespace sensors
}  // namespace crf
