/* © Copyright CERN 2018. All rights reserved. This software is released under a CERN proprietary software license. 
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 * 
 * Author: Leanne Attard EN-SMM-MRO
 * 
 *  ==================================================================================================
 */
#include "ThermalCamera/FLIRCam.hpp"
#include <algorithm>
#include <memory>
#include <vector>

PV_INIT_SIGNAL_HANDLER();

#define BUFFER_COUNT (16)

namespace crf {
namespace sensors {
namespace thermalcamera {

FLIRCam::FLIRCam(std::shared_ptr<IPC> ipc, std::shared_ptr<IFLIRFactory> factory):
    logger_("FLIRCam"),
    factory_(factory),
    ipc_(ipc),
    pipelineStarted(false),
    streaming(false),
    lDeviceParams(nullptr),
    device_(nullptr),
    params(nullptr),
    temp_converter(nullptr),
    lStream(nullptr),
    lPipeline(nullptr),
    deinitialized(false),
    initialized(false) {}

FLIRCam::~FLIRCam() {
    logger_->debug("DTor");
    deinitialize();
}

bool FLIRCam::initialize() {
    logger_->debug("initialize");
    if (initialized) {
      logger_->warn("Device already initialized!");
      return false;
    }

    lStream = nullptr;

    // PV_SAMPLE_INIT();
    PvString lConnectionID;

    logger_->debug("Going to selectThermalDevice");
    PvString* conn = factory_->selectThermalDevice(0);  // TODO index from config  // NOLINT

    if ((conn)->GetLength() > 0) {  // TODO index from config  // NOLINT
        logger_->debug("going to connect to device ... ");
        device_ = ConnectToDevice(*conn);
    }

    if (device_) {
        logger_->debug("going to OpenStream");
        lStream = OpenStream(*conn);
    }

    if (lStream) {
        // Get stream parameters
        logger_->debug("going to GetParameters");
        lStreamParams = lStream->GetParameters();

        logger_->debug("going to configure stream");
        ConfigureStream(device_, lStream);
    }

    // Get device parameters need to control streaming
    logger_->debug("Going to createDeviceParams");
    params = factory_->createDeviceParams(device_);
    if (!params) {
        logger_->warn("One or more parameters could not be loaded!");
        return false;
    }

    // create the temperature converter to change raw data to celsius
    temp_converter = factory_->createTempConverter(params);
    if (!temp_converter) {
        logger_->warn("Temperature converter could not be created!");
        return false;
    }

    if (!pipelineStarted) {
        if (createPipelineAndStartStream()) {
            pipelineStarted = true;
        }
        initialized = true;
        deinitialized = false;
        return pipelineStarted;
    }

    return false;
}

bool FLIRCam::deinitialize() {
    logger_->debug("deinitialize");
    bool result = true;

    if (deinitialized) {
        logger_->warn("Already deinitialized!");
        return false;
    }

    if (!params) {
        logger_->warn("Device Parameters not valid!");
        result = false;
    } else {
        logger_->debug("Getting the command to stop the acquisition");
        // send the AcquisitionStop command
        PvGenCommand *lStop = params->getCommand("AcquisitionStop");
        if (lStop) {
            // Tell the device to stop sending images.
            logger_->info("Sending AcquisitionStop command to the device");
            lStop->Execute();
        } else {
            logger_->warn("Device could not be stopped! Either it was already stopped or it was "
                "never started!");
            result =  false;
        }
    }

    if (device_) {
        // Disable streaming on the device
        logger_->info("Disable streaming on the controller.");
        device_->StreamDisable();
    }

    if (lPipeline) {
        // Stop the pipeline
        logger_->info("Stop pipeline");
        if (pipelineStarted)
            lPipeline->Stop();
    } else {
        logger_->warn("Pipeline does not exist!");
        result =  false;
    }

    pipelineStarted = false;

    if (lStream) {
        // Close the stream
        logger_->debug("Going to close the stream");
        lStream->Close();
        PvStream::Free(lStream);
    } else {
        logger_->warn("Stream does not exist!");
        result =  false;
    }
    logger_->debug("Going to disconnect device");
    // Disconnect the device
    if (device_) {
        if (device_->IsConnected()) {
            logger_->info("Disconnecting device");
            device_->Disconnect();
        }
        logger_->debug("Going to Free device_");
        PvDevice::Free(device_);
    }

    if (params)
        delete params;

    if (temp_converter)
        delete temp_converter;

    // TODO check if needed PV_SAMPLE_TERMINATE();  // NOLINT
    deinitialized = true;
    initialized = false;
    return result;
}

cv::Mat FLIRCam::getFrame() {
    cv::Mat frame;

    if (!device_) {
        logger_->warn("No device or empty client!");
        return cv::Mat();
    }

    cv::Mat rawImage;
    cv::Mat thermalData;
    bool result = getSingleFrame(rawImage, thermalData);
    if (!result) {
        logger_->warn("Getting thermal data failed");
        return cv::Mat();
    }

    return thermalData;
}

bool FLIRCam::publishFrame(const cv::Mat& frame) {
    if (frame.empty()) {
        return false;
    }

    Packets::ThermalCameraPacket cameraPacket;
    cameraPacket.temperatures = frame;

    ipc_->write(cameraPacket.serialize(), cameraPacket.getHeader());

    return true;
}

bool FLIRCam::showFrame() {
    if (!device_) {
        logger_->warn("No device!");
        return false;
    }
    cv::Mat frame;
    cv::Mat thermal;
    bool result = getSingleFrame(frame, thermal);

    if (!result || frame.empty()) {
        logger_->warn("Empty thermal frame");

        return false;
    }

    cv::imshow("Streaming", frame);
    cv::waitKey(1);
    return true;
}

bool FLIRCam::streamVideo() {
    if (!device_) {
        logger_->warn("No device!");
        return false;
    }
    if (lPipeline) {
        streaming = true;
        // Map a few GenICam stream stats counters
        PvGenFloat *lFrameRate = dynamic_cast<PvGenFloat *>(lStreamParams->Get("AcquisitionRate"));
        PvGenFloat *lBandwidth = dynamic_cast<PvGenFloat *>(lStreamParams->Get("Bandwidth"));

        // Acquire images until the user instructs us to stop.
        cout << endl << "<press a key to stop streaming>" << endl;

        while (!PvKbHit()) {  // TODO to check PvKbHit  // NOLINT
            cv::Mat frame;
            cv::Mat thermal;
            bool result = getSingleFrame(frame, thermal);
            showFrame();
            if (result) {
                double lFrameRateVal = 0.0;
                double lBandwidthVal = 0.0;
                lFrameRate->GetValue(lFrameRateVal);
                lBandwidth->GetValue(lBandwidthVal);
                string str_log = "FPS" + to_string(lFrameRateVal) + " " +
                    to_string(lBandwidthVal / 1000000.0) + "Mb/s";
                logger_->info(str_log);
            } else {
                logger_->warn("Empty Frame");
            }
        }
    } else {
        return false;
    }

    return true;
}

PvDevice * FLIRCam::ConnectToDevice(const PvString &aConnectionID) {
    PvDevice *device_;
    PvResult lResult;

    // Connect to the GigE Vision or USB3 Vision device
    logger_->info("Connecting to device.");
    device_ = PvDevice::CreateAndConnect(aConnectionID, &lResult);
    if (device_ == NULL) {
        logger_->warn("Unable to connect to device.");
    }

    return device_;
}

PvStream * FLIRCam::OpenStream(const PvString &aConnectionID) {
    PvStream *lStream;
    PvResult lResult;

    // Open stream to the GigE Vision or USB3 Vision device
    lStream = PvStream::CreateAndOpen(aConnectionID, &lResult);
    if (lStream == NULL) {
        cout << "Unable to stream from device." << endl;
    }

    return lStream;
}

void FLIRCam::ConfigureStream(PvDevice *aDevice, PvStream *aStream) {
    // If this is a GigE Vision device, configure GigE Vision specific streaming parameters
    PvDeviceGEV* lDeviceGEV = dynamic_cast<PvDeviceGEV *>(aDevice);
    if (lDeviceGEV != NULL) {
        PvStreamGEV *lStreamGEV = static_cast<PvStreamGEV *>(aStream);

        // Negotiate packet size
        lDeviceGEV->NegotiatePacketSize();

        // Configure device streaming destination
        lDeviceGEV->SetStreamDestination(lStreamGEV->GetLocalIPAddress(),
            lStreamGEV->GetLocalPort());
    }
}

bool FLIRCam::getSingleFrame(cv::Mat& rawImage, cv::Mat& tempData) {
    cv::Mat workImage;

    PvBuffer *lBuffer = NULL;
    PvResult lOperationResult;

    // Retrieve next buffer
    PvResult lResult = lPipeline->RetrieveNextBuffer(&lBuffer, 1000, &lOperationResult);
    if (lResult.IsOK()) {
        if (lOperationResult.IsOK()) {
            PvPayloadType lType;

            // If the buffer contains an image, display width and height.
            uint32_t lWidth = 0, lHeight = 0;
            lType = lBuffer->GetPayloadType();

            if (lType == PvPayloadTypeImage) {
                // Get image specific buffer interface.
                PvImage *lImage = lBuffer->GetImage();

                // Read width, height.
                lWidth = lImage->GetWidth();
                lHeight = lImage->GetHeight();
                // cout <<"Width:"<<lWidth<<" Height:"<< lHeight << endl;

                lImage->Alloc(lWidth, lHeight, PvPixelRGB16);
                // Get image data pointer so we can pass it to CV::MAT container
                unsigned char *img = lImage->GetDataPointer();
                // Copy/convert Pleora Vision image pointer to cv::Mat container
                cv::Mat lframe(lHeight, lWidth, CV_16UC1, img, cv::Mat::AUTO_STEP);
                lframe.copyTo(rawImage);

                tempData = rawToHeat(rawImage);
                /*ofstream temp_file("/home/pcen35089/temperature_values_celsius.csv");
                temp_file<<format(tempData, cv::Formatter::FMT_CSV) << endl;
                temp_file.close();*/
            } else {
                logger_->warn(" (buffer does not contain image)");
            }
        } else {
            logger_->warn(lOperationResult.GetCodeString().GetAscii());
        }

        // Release the buffer back to the pipeline
        lPipeline->ReleaseBuffer(lBuffer);
    } else {
        // Retrieve buffer failure
        logger_->warn(lResult.GetCodeString().GetAscii());
        return false;
    }

    return true;
}

cv::Mat FLIRCam::rawToHeat(const cv::Mat raw) {
    cv::Mat heat = cv::Mat::zeros(raw.rows, raw.cols, CV_32F);

    for (int r = 0; r< raw.rows; ++r) {
        float*  pheat = heat.ptr<float>(r);
        const uint16_t * praw = raw.ptr<uint16_t>(r);
        for (int c = 0; c< raw.cols; ++c) {
            pheat[c] = static_cast<double>(temp_converter->RawToCelsius(
                static_cast<double>(praw)[c]));
        }
    }
    return heat;
}

bool FLIRCam::createPipelineAndStartStream() {
    // Create the PvPipeline object
    lPipeline = new PvPipeline(lStream);
    PvResult result;

    if (lPipeline != NULL) {
        // Reading payload size from device
        uint32_t lSize = device_->GetPayloadSize();

        // Set the Buffer count and the Buffer size
        result = lPipeline->SetBufferCount(BUFFER_COUNT);
        if (!result.IsOK())
            return false;

        lPipeline->SetBufferSize(lSize);

        // Map the GenICam AcquisitionStart command
        PvGenCommand *lStart = params->getCommand("AcquisitionStart");
        result = lPipeline->Start();
        if (!result.IsOK())
            return false;

        // Enable streaming and send the AcquisitionStart command
        logger_->debug("Going to enable Enabling Stream");
        result = device_->StreamEnable();
        if (!result.IsOK())
            return false;

        logger_->debug("Going to enable exectute the acquisition command");
        result = lStart->Execute();
        if (!result.IsOK())
            return false;

        return true;
    } else {
      return false;
    }
}

}  // namespace thermalcamera
}  // namespace sensors
}  // namespace crf
