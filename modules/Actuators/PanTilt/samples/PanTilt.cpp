/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license. 
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 * 
 * Author: Carlos Veiga Almagro CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#include <csignal>
#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include <utility>                  // std::pair, std::make_pair

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>      // for webCam
#include <opencv2/imgproc.hpp>      // for cvtColor

#include <nlohmann/json.hpp>

#include "Dynamixel/DynamixelSDK.hpp"
#include "EventLogger/EventLogger.hpp"
#include "PanTilt/DynamixelPanTilt.hpp"

std::unique_ptr<crf::devices::pantilt::DynamixelPanTilt> _dxl_PT;
crf::utility::logger::EventLogger logger("PanTilt sample");
double px;
double py;
cv::Size frameSize;

void onMouse(int, int, int, int, void*);
bool initParams(int, const char *[]);

int main(int argc, char const *argv[]) {
    if ( !initParams(argc, argv) ) return -1;

    cv::Mat frame;
    cv::VideoCapture cap;
    if (!cap.open(0)) {
        logger->error("ERROR: camera is missing");
        return -1;
    }

    int mouseParam = CV_EVENT_FLAG_LBUTTON;
    cv::namedWindow("Scene", CV_WINDOW_NORMAL);
    cv::setMouseCallback("Scene", onMouse, &mouseParam);  // mosue handler

    cap >> frame;
    frameSize = cv::Size(frame.size());
    while (true) {
        cap >> frame;
        cv::imshow("Scene", frame);
        if ( cv::waitKey(1) == 'q' ) break;
    }
    return _dxl_PT->deinitialize();
}

bool initParams(int argc, const char * argv[]) {
    if (argc < 3) {
        logger->error(R"(Needed argument no detected:
            1) USB port (Es. '/dev/ttyUSB0')
            2) Name of configuration file (./modules/Devices/Dynamixel/configuration/DXL-MX-28.json)
            )");
        return false;
    }
    std::ifstream dxl_data(argv[2]);
    if ((dxl_data.rdstate() && std::ifstream::failbit) != 0) return false;

    nlohmann::json dxl_JSON;
    dxl_data >> dxl_JSON;
    auto dxl = std::make_shared<crf::devices::dynamixelstepper::Dynamixel>(argv[1],
        dxl_JSON);

    _dxl_PT.reset(new crf::devices::pantilt::DynamixelPanTilt(dxl));
    // _dxl_PT.reset(new crf::devices::DynamixelPanTilt(argv[1], dxl_JSON));
    return _dxl_PT->initialize();
}

void onMouse(int event, int x, int y, int, void* param) {
    if ( event == CV_EVENT_LBUTTONDOWN ) {
        cv::Point pt = cv::Point(x, y);

        px += (pt.x - (frameSize.width / 2)) / static_cast<double>(frameSize.width);
        py += (pt.y - (frameSize.height / 2)) / static_cast<double>(frameSize.height);

        logger->debug("x = {0}  \t y = {1}", px, py);
        if (px < 0) {
            px = 0;
        }
        if (py < 0) {
            py = 0;
        }
        if (px > 5.5) {
            px = 5.5;
        }
        if (py > 5.5) {
            py = 5.5;
        }
        std::pair<double, double> coordinates(px, py);
        _dxl_PT->setPosition(coordinates);
    }

    return;
}
