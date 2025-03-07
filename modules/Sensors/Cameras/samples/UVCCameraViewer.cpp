/* © Copyright CERN 2023. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Álvaro García González CERN BE/CEM/MRO 2023
 *
 *  ==================================================================================================
 */

#include <opencv2/opencv.hpp>
#include "Cameras/UVCCamera/UVCCamera.hpp"

int main() {
    // Open the default camera
    crf::sensors::cameras::UVCCamera cam("/dev/video0");

    cam.initialize();
    // Create a window to display the camera feed
    cv::namedWindow("Camera Feed", cv::WINDOW_NORMAL);

    crf::expected<bool> res = cam.setProperty(crf::sensors::cameras::Property::PAN, 7200);

    if (!res) std::puts("Camera property \"Pan\" could not be set!");

    int pan = cam.getProperty(crf::sensors::cameras::Property::PAN);
    std::cout << "Camera pan: " << pan << " degrees." << std::endl;

    while (true) {
        // Read a new frame from the camera
        cv::Mat frame = cam.captureImage();

        // Check if the frame was successfully captured
        if (frame.empty()) {
            std::cout << "End of video stream" << std::endl;
            break;
        }

        // Display the frame in the window
        cv::imshow("Camera Feed", frame);

        // Press ESC to exit the program
        if (cv::waitKey(1) == 27) {
            break;
        }
    }

    // Release the camera and close the window
    cv::destroyAllWindows();

    return 0;
}
