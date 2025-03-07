/* © Copyright CERN 2018. All rights reserved. This software is released under a CERN proprietary software license. 
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 * 
 * Author: Carlos Veiga Almagro CERN EN/SMM/MRO
 * 
 *  ==================================================================================================
 */

#include <functional>
// #include <stdlib.h>

#include <Logger/Logger.hpp>
#include <DepthEstimation/ScrewDetection.hpp>
#include <DepthEstimation/TrackingArea.hpp>

int _mouseParam = CV_EVENT_FLAG_LBUTTON;
cv::Mat _frame;
crf::Vision::ScrewDetection _sd;
crf::Vision::TrackingArea _myTracker;
bool _takingDimensions = false;  // , _choosedArea = false;
cv::Rect_<double> _myArea, _areaAux;

crf::utility::logger::EventLogger _logger("Main-Screw_Detection");
/**
* Handler of the mouse
* It takes the event from mouse and chooses the option to carry out
* @event, event captured
* @x & @y, cursor coordinates from the event
*/
void onMouse(int event, int x, int y, int, void* param) {
    switch (event) {
        case CV_EVENT_LBUTTONUP:
            _myTracker.upgradeTracker(cv::Point(x, y));
            break;
        case CV_EVENT_RBUTTONDOWN:
            /** removing a screw of the chosen */
            std::cout << "(" << x << "," << y << ")" << std::endl;
            _myTracker.deleteTracker(cv::Point(x, y));
            break;
    }
}

int main(int argc, char** argv) {
    if (argc < 3) {
        _logger->error("Not enough parameters");
        std::cout << "Arg1- fifor_arm" << std::endl;
        std::cout << "Arg2- mmap_arm" << std::endl;

        return -1;
    }

    cv::VideoCapture cap;
    if (!cap.open(0)) {
        _logger->error("Impossible to open the camera");
        return -1;
    }
    cap >> _frame;

    cv::namedWindow("Scene", CV_WINDOW_AUTOSIZE);
    cv::setMouseCallback("Scene", onMouse, &_mouseParam);  // mouse handler
    // _frame = cv::imread("/home/cveigaal/Pictures/screw-plate.JPG", CV_LOAD_IMAGE_COLOR);
    if (_frame.empty()) {
        _logger->critical("Impossible to open the image");
        exit(0);
    }

    cv::resize(_frame, _frame, cv::Size(640, 420));
    _sd.setFrame(_frame);
    auto screws = _sd.searchTheScrews();
    auto maxValue = [=](int maxRadius) {
        for (const auto& crl : screws)
            if (crl[2] >= maxRadius) maxRadius = crl[2];
        return maxRadius;
    };
    _myTracker.setFromScratch(_frame, screws.size(), screws, maxValue(0));

    while (true) {
        cap >> _frame;

        // _sd.drawCircles(_frame, screws);
        // _myTracker.drawRois(_frame);
        _myTracker.updateTracker(_frame);
        cv::imshow("Scene", _frame);
        if ( cv::waitKey(20) == 27 ) break;
    }

    return 0;
}
