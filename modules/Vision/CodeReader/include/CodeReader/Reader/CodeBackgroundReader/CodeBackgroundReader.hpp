/* © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Jorge Playán Garai CERN BE/CEM/MRO 2021
 *
 *  ==================================================================================================
 */

#pragma once

#include <memory>
#include <thread>
#include <string>
#include <condition_variable>
#include <atomic>
#include <math.h>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <visp3/vision/vpKeyPoint.h>

#include "CodeReader/Detector/IDetector.hpp"
#include "CodeReader/Decoder/IDecoder.hpp"
#include "Types/Types.hpp"
#include "Cameras/ICamera.hpp"
#include "EventLogger/EventLogger.hpp"

namespace crf::vision::codereader {

class CodeBackgroundReader {
 public:
    CodeBackgroundReader() = delete;
    explicit CodeBackgroundReader(
        std::shared_ptr<crf::sensors::cameras::ICamera> camera,
        std::shared_ptr<IDecoder> decoder,
        std::shared_ptr<IDetector> detector = nullptr);
    ~CodeBackgroundReader();

    /**
     * @brief Function to start the detection of QR codes on an image feed from a camera
     */
    void start();
    /**
     * @brief Function to stop the detection of QR codes previoulsy started, in case it was
     *        not started then nothing will happen
     */
    void stop();
    /**
     * @brief  Function to know if a positive reading occurred.
     * @return True if there was a positive reading of a QR code from the camera
     * @return False if no reading has occurred or it has failed
     */
    bool isDetected();
    /**
     * @brief  Once the reading has occurred, this function will return the read value
     * @return A string with the decoded information obstained from the QR
     */
    std::string getCode();
    /**
     * @brief  Function to get the positition at which the QR code was detected from the camera
     * @param  Real dimension of the width of the QR code
     * @param  Real dimension of the height of the QR code
     * @return A task position with the X, Y, and Z to get the position and Roll,
     *         Pitch, Yaw  to get the orientation of the QR respective to the camera
     */
    crf::utility::types::TaskPose getCodeTaskPose();

 private:
    void execute();

    std::shared_ptr<crf::sensors::cameras::ICamera> camera_;
    std::shared_ptr<IDetector> detector_;
    std::shared_ptr<IDecoder> decoder_;

    std::atomic<bool> detected_;
    std::string content_;

    std::array<float, 8> result_;
    cv::Mat frame_;

    std::thread QRThread_;

    std::atomic<bool> stopThreads_;
    std::atomic<bool> record_;

    std::mutex mtx_;
    std::mutex mtxContent_;

    std::condition_variable recordCV_;
    std::string position_;

    const std::chrono::milliseconds loopTime_ = std::chrono::milliseconds(20);

    crf::utility::logger::EventLogger logger_;
};

}  // namespace crf::vision::codereader
