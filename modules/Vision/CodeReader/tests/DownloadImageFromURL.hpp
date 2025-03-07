/* © Copyright CERN 2022. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Jorge Playán Garai CERN BE/CEM/MRO 2022
 *
 * Based on the file in https://gist.github.com/yiling-chen/6129189 and adapted to curlpp
 *
 *  ==================================================================================================
 */

#pragma once

#include <string>
#include <opencv2/opencv.hpp>

cv::Mat DownloadImageFromURL(const std::string& url);
