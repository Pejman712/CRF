/* © Copyright CERN 2022. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Jorge Playán Garai CERN BE/CEM/MRO 2022
 *
 * Based on the file in https://gist.github.com/yiling-chen/6129189 and adapted to curlpp
 *
 *  ==================================================================================================
 */

#include <stdio.h>
#include <stdlib.h>

#include <curlpp/cURLpp.hpp>
#include <curlpp/Easy.hpp>
#include <curlpp/Options.hpp>
#include <curlpp/Exception.hpp>
#include <curlpp/Infos.hpp>

#include "DownloadImageFromURL.hpp"

#include <opencv2/opencv.hpp>

namespace {
    char *m_pBuffer = NULL;
    size_t m_Size = 0;

    void* Realloc(void* ptr, size_t size) {
        if (ptr)
            return realloc(ptr, size);
        else
            return malloc(size);
    }

    size_t WriteMemoryCallback(char* ptr, size_t size, size_t nmemb) {
        size_t realsize = size * nmemb;

        m_pBuffer = (char*) Realloc(m_pBuffer, m_Size + realsize);  // NOLINT
        if (m_pBuffer == NULL) {
            realsize = 0;
        }
        memcpy(&(m_pBuffer[m_Size]), ptr, realsize);
        m_Size += realsize;
        return realsize;
    }
}  // namespace

cv::Mat DownloadImageFromURL(const std::string& url) {
    curlpp::Cleanup myCleanup;
    curlpp::Easy myRequest;
    cv::Mat image;

    curlpp::types::WriteFunctionFunctor functor(WriteMemoryCallback);
    myRequest.setOpt<curlpp::options::WriteFunction>(functor);
    myRequest.setOpt<curlpp::options::Url>(url);
    myRequest.perform();

    image = cv::imdecode(cv::Mat(1, m_Size, CV_8UC1, m_pBuffer), cv::IMREAD_UNCHANGED);
    m_pBuffer = NULL;
    m_Size = 0;
    return image;
}
