#pragma once

#include <ueye.h>
#include <ueye_deprecated.h>

#include <sys/stat.h>
#include <vector>
#include <string>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <fstream>

// opencv
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "DirectSparseOdometry/opencv2/video/tracking.hpp"

class CamSetup {
  public:
    
    CamSetup();
    ~CamSetup();
    
    void GetExposureAndFPS(HIDS, double*, double*, double*, double);
    bool Camera(HIDS);
    
    cv::Mat GetFrame(HIDS);
    
    int GetWidth(HIDS);
    int GetHeight(HIDS);
    
    std::string ZeroPadNumber(int);
    std::string ToString(double);
    
  private:
    
    void GetMaxImageSize(HIDS, INT*, INT*);
    INT *pnSizeX;
    INT *pnSizeY;
    
    INT nRet;

//     double *expMin;
//     double *expMax;
//     double *expInc;
//     double fps;
  };