/* © Copyright CERN 2023. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Alvaro Garcia Gonzalez BE/CEM/MRO 2023
 *
 *  ==================================================================================================
 */

#pragma once

#include <vector>
#include <string>
#include "RGBDCameras/RealSenseCamera/RealSenseCamera.hpp"

namespace crf {
namespace sensors {
namespace rgbdcameras {

class D405RealSenseCamera : public RealSenseCamera {
 public:
    explicit D405RealSenseCamera(const std::string& serial_number);
    cv::rgbd::RgbdFrame captureImageAndDepth() override;
    std::vector<Profile> listProfiles() override;

    bool setProfile(const Profile& profile) override;
    bool setProfile(const Profile& streamprofile, const Profile& depthprofile) override;

 private:
    void parseProfileList() override;
};

}  // namespace rgbdcameras
}  // namespace sensors
}  // namespace crf
