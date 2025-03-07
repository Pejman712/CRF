/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Sergio Villanueva Lorente CERN EN/SMM/MRO 
 *
 *  ==================================================================================================
 */

#include "RGBDCamera/RealSense2Grabber.hpp"
#include "RGBDCamera/IRGBDCamera.hpp"
#include "RGBDCamera/RGBDUtils.hpp"
#include "CameraViewer/CameraViewer.hpp"

#include <iostream>
#include <fstream>
#include <nlohmann/json.hpp>
#include <memory>
#include <string>
#include <chrono>
#include <vector>

bool isFileExist(const std::string filename) {
    /// Open the file
    FILE* f = fopen(filename.c_str(), "rb");

    /// in the event of a failure, return false
    if (!f)
        return false;

    fclose(f);

    return true;
}

void SaveImage(cv::Mat received_image) {
    int frame_num = 0;
    bool _imagenameflag = false;
    // Save image on disk as .jpeg and 100% quality
    std::vector<int> compression_params;
    compression_params.push_back(CV_IMWRITE_JPEG_QUALITY);
    compression_params.push_back(100);

    // current date/time based on current system
    time_t now = time(0);
    // convert now to string form
    char* dt = ctime(&now); //NOLINT
    std::string save_name = "../modules/Applications/Mapper3d/data/blm_image_";
    save_name.append(dt);
    save_name.append(".jpeg");
    // Check if the image file already exits to avoid overwriting
    while (!_imagenameflag) {
        if (!isFileExist(save_name)) {
            imwrite(save_name, received_image, compression_params);
            std::cout << "Image saved" << std::endl;
            _imagenameflag = true;
        } else {
            if (frame_num != 0) {
                save_name = save_name.substr(0, save_name.size()-3);
            }
            frame_num++;
            save_name.append("(");
            std::string frame_num_str = std::to_string(frame_num);
            save_name.append(frame_num_str);
            save_name.append(")");
        }
    }
}

int main(int argc, char **argv) {
    if (argc < 3) {
        std::cout << "Few arguments provide:" <<std::endl;
        std::cout << " [1] Path to RGBD Camera config file" << std::endl;
        std::cout << " [2] Save rate" << std::endl;
        std::cout << " [3] Number of images to be taken" << std::endl;
        return -1;
    }

    int saveRate = atoi(argv[2]);
    int imageNumber = atoi(argv[3]);

    // Open camera
    std::string configFileCamera(argv[1]);
    std::ifstream configCamera(configFileCamera);
    nlohmann::json jConfigCamera;
    configCamera >> jConfigCamera;
    auto camera = std::make_shared<crf::sensors::rgbdcamera::RealSense2Grabber>
        (jConfigCamera.at("RealSense"));
    std::shared_ptr<crf::sensors::rgbdcamera::IRGBDCamera> camera_(camera);
    if (!camera_->initialize()) {
        std::cout << "There is a problem to initialize the RGBD Camera" << std::endl;
        return false;
    }

    int i = 0;
    auto start = std::chrono::high_resolution_clock::now();
    while (i < imageNumber) {
        // Check elapsed time to follow save_rate
        auto finish = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed = finish - start;
        if (elapsed.count() >= saveRate) {
            std::cout << "Elapsed time: " << elapsed.count() << " s\n";
            cv::Mat color = camera_->getColorFrame();
            SaveImage(color);
            i++;
            start = std::chrono::high_resolution_clock::now();
        }
    }

    // If the letter 'q' is pressed the program process this frame pointcloud
    char key;
    bool endflag = false;
    std::cout << "KEYBOARD CONTROL"<< std::endl;
    while (!endflag) {
        std::cout << "Press 'q'+ ENTER to exit" << std::endl;
        std::cin >> key;
        if (key == 'q') endflag = true;
    }

    return 0;
}

