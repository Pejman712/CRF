/* © Copyright CERN 2022. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Victor Drame CERN BE/CEM/MRO 2022
 *
 * ====================================================================
*/

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

#include "../tests/DownloadImageFromURL.hpp"
#include "CodeReader/Detector/ArUcoCVDetector/ArUcoCVDetector.hpp"

namespace {
const char* about = "Detect ArUco marker images";
const char* keys  =
        "{d        |5    | dictionary: DICT_4X4_50=0, DICT_4X4_100=1, "
        "DICT_4X4_250=2, DICT_4X4_1000=3, DICT_5X5_50=4, DICT_5X5_100=5, "
        "DICT_5X5_250=6, DICT_5X5_1000=7, DICT_6X6_50=8, DICT_6X6_100=9, "
        "DICT_6X6_250=10, DICT_6X6_1000=11, DICT_7X7_50=12, DICT_7X7_100=13, "
        "DICT_7X7_250=14, DICT_7X7_1000=15, DICT_ARUCO_ORIGINAL = 16}"
        "{marker_length |0.1| length of marker edges}"
        "{h        |false | Print help }";
}  // namespace

int main(int argc, char **argv) {
    int wait_time = 1;  // in ms

    // Webcam
    cv::Mat intrinsicmtx = (cv::Mat_<double>(3, 3) <<
        504.7955256525734, 0.0,                 301.2475611322225,
        0.0,               504.5502629982254,   237.46392613009286,
        0.0,               0.0,                 1.0);

    cv::Mat distortionmtx = (cv::Mat_<double>(5, 1) <<
        0.07999694369751921,
        -0.19464815627786125,
        0.0015742447929707277,
        4.0620416199986476e-05,
        0.06297408919749693);

    cv::Mat frame;
    cv::CommandLineParser parser(argc, argv, keys);
    parser.about(about);

    if (parser.get<bool>("h")) {
        parser.printMessage();
        return 0;
    }

    if (!parser.check()) {
        parser.printErrors();
        return 1;
    }
    int dictionaryId = parser.get<int>("d");
    double markerLength = parser.get<double>("marker_length");

    cv::VideoCapture cap(0);
    auto arucoReader =
        std::make_shared<crf::vision::codereader::ArUcoCVDetector>(
            dictionaryId, intrinsicmtx, distortionmtx, markerLength);

    while (cap.isOpened()) {
        cap >> frame;
        cv::waitKey(wait_time);
        cv::imshow("test", frame);
        if (arucoReader->detect(frame)) {
            std::cout << "MarkerID: " << arucoReader->getMarkersIds()[0] << std::endl;
            std::cout << "TaskPose: " << arucoReader->getCodeTaskPose()[0] << std::endl;
            std::cout << "PositionInImage: " << arucoReader->getPositionInImage()[0] << std::endl;
        } else {
            std::cout << "No marker detected" << std::endl;
        }
    }
    return 0;
}
