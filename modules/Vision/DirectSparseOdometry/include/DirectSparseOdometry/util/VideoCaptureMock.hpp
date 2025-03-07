/* © Copyright CERN 2017. All rights reserved. This software is released under a CERN proprietary software license. 
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 * 
 * Author: Pawel Ptasznik CERN EN/STI/ECE
 * 
 *  ==================================================================================================
 */

#pragma once

#include <gmock/gmock.h>
#include <opencv2/opencv.hpp>

namespace cv {

class VideoCaptureMock : public VideoCapture {
 public:
  MOCK_METHOD1(open,
      bool(const String& filename));
  MOCK_METHOD1(open,
      bool(int index));
  MOCK_CONST_METHOD0(isOpened,
      bool());
  MOCK_METHOD0(release,
      void());
  MOCK_METHOD0(grab,
      bool());
  MOCK_METHOD2(retrieve,
      bool(OutputArray, int));
  MOCK_METHOD1(read,
      bool(OutputArray image));
  MOCK_METHOD2(set,
      bool(int propId, double value));
  MOCK_CONST_METHOD1(get,
      double(int propId));
  MOCK_METHOD2(open,
      bool(const String& filename, int apiPreference));
  virtual VideoCapture& operator >> (Mat& image) override { return *this; }
  virtual VideoCapture& operator >> (UMat& image) override { return *this; }
};

}  // namespace cv
