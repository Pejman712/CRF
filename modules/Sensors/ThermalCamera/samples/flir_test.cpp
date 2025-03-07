#include <iostream>
#include <memory>

#include <opencv2/opencv.hpp>

#include "ThermalCamera/FLIRCam.hpp"
#include "ThermalCamera/FLIRFactory.hpp"
//
// Main function
//
using crf::sensors::thermalcamera::FLIRFactory;

int main()
{
      string m = "/tmp/mmap_thermal";
      //std::shared_ptr<MMAP> mmap = MMAP::CreateWriterPtr((char*)m.c_str(), 440064);
      std::shared_ptr<IPC> ipc;
      auto factory = std::make_shared<FLIRFactory>();
      crf::sensors::thermalcamera::FLIRCam therm_camera (ipc, factory);
      if(!therm_camera.initialize())
      { 
	cout<<"Not init"<<endl;
        //TODO delete ?
	return -1;//TODO
      }
      cout <<"Test"<< endl;
      therm_camera.streamVideo();
      //therm_camera.getFrame();
}