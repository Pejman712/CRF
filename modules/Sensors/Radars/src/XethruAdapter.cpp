/* © Copyright CERN 2018. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Arturs Ivanovs CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

/*
 * TODO(aivanovs): or anyone else, whatever ...
 * This implementation has a lot of "if-else" which probably is not really correct
 * I detected 2 "control reaches end of non-void function" previously
 * Also style is shitty, sometimes indentation is 2 spaces, sometimes 4, sometimes 6 ...
 */

#include "Radars/XethruRadar/XethruAdapter.hpp"
#include <string>

namespace crf {
namespace sensors {
namespace xethruradar {

XethruAdapter::XethruAdapter(const std::string& deviceFilename) :
    logger_("XethruRadar"),
    deviceName_(deviceFilename),
    mc_(deviceName_, 0),
    x4m200_(mc_.get_x4m200()),
    initialized_(false),
    frameReady_(false),
    module_(),
    fwid_(),
    xep_(mc_.get_xep()),
    dataRecorder_(mc_.get_data_recorder()),
    respirationData_(),
    dataTypes_(XeThru::AllDataTypes) {
    logger_->debug("CTor");
}

XethruAdapter::~XethruAdapter() {
    logger_->debug("DTor");
    deinitialize();
}

bool XethruAdapter::initialize() {
    logger_->debug("initialize");
    uint8_t sensorMode;
    if (initialized_) {
        logger_->warn("Device already initialized!");
        return false;
    }
    logger_->debug("Checking connected Xethru hardware...");
    xep_.get_system_info(0x01, &module_);
    xep_.get_system_info(0x02, &fwid_);
    if (module_ == "X4M200") {
        logger_->debug("X4M200 Respiration sensor recognized");
        x4m200_.get_sensor_mode(&sensorMode);
        if (sensorMode == 19) {
            logger_->debug("Sensor is already stopped, waiting for instructions...");
        } else {
          if (x4m200_.set_sensor_mode(XTID_SM_STOP, 0) != 0) {
              logger_->warn("X4M200: set_sensor_mode failed");
              return false;
            } else {
              logger_->debug("Sensor stopped, waiting for instructions...");
            }
        }
      } else if (module_.empty()) {
      logger_->warn("Module name is empty. It is a known bug for X4M200 sensor.");

      x4m200_.get_sensor_mode(&sensorMode);
      if (sensorMode == 19) {
        logger_->debug("Sensor is already stopped, waiting for instructions...");
      } else {
        if (x4m200_.set_sensor_mode(XTID_SM_STOP, 0) != 0) {
            logger_->warn("X4M200: set_sensor_mode failed");
            return false;
        } else {
            logger_->debug("Sensor stopped, waiting for instructions...");
        }
      }
    } else {
        logger_->warn("Other sensor connected with name: ");
        logger_->warn(module_);
        logger_->warn("Only X4M200 is suppported, check the device!");
        return false;
      }
    int internal_queue = xep_.peek_message_data_float();
    if (internal_queue != 0) {
        logger_->warn("Buffer is not clear.");
        return false;
      } else {
        logger_->debug("Buffer is clear.");
    }
    logger_->debug("Device ready to use.");
    initialized_ = true;
    return true;
}

bool XethruAdapter::deinitialize() {
    logger_->debug("Deinitialize");
    if (module_.empty() == true) {
        logger_->warn("Restarting with XEP, device unknown.");

        xep_.module_reset();
        mc_.close();
        return true;
    } else if (module_ == "X4M200") {
        if (x4m200_.set_sensor_mode(XTID_SM_STOP, 0) != 0) {
            logger_->warn("set_sensor_mode failed");
            return false;
        }
        logger_->debug("X4M200 successfully restarted and stopped.");
        initialized_ = false;
        return true;
    }
    return false;
}

bool XethruAdapter::startStream(float fa1, float fa2, int sensitivity) {
      if (x4m200_.load_profile(XTS_ID_APP_RESPIRATION_2) != 0) {
          logger_->warn("load_profile failed");
          return false;
      }
      if (x4m200_.set_detection_zone(fa1, fa2) != 0) {
        logger_->warn("Detection zone set failed");
        return false;
      }
      float startZone;
      float endZone;
      x4m200_.get_detection_zone(&startZone, &endZone);
      logger_->debug("Detection start zone: ");
      logger_->debug(startZone);
      logger_->debug("Detection end zone: ");
      logger_->debug(endZone);
      x4m200_.set_sensitivity(sensitivity);
      if (x4m200_.set_output_control(XTS_ID_PULSEDOPPLER_FLOAT,
        XTID_OUTPUT_CONTROL_PD_FAST_ENABLE
         | XTID_OUTPUT_CONTROL_PD_SLOW_ENABLE) != 0) {
        logger_->warn("load_profile failed");
        return false;
      }
      if (x4m200_.set_sensor_mode(XTID_SM_RUN, 0) != 0) {
        logger_->warn("set_sensor_mode failed");
        return false;
      }
      logger_->debug("Sensor started stream");
      return true;
}

XeThru::RespirationData XethruAdapter::getFrame() {
  uint8_t sensorMode;
  x4m200_.get_sensor_mode(&sensorMode);
  if (sensorMode == 19) {
      logger_->warn("X4M200 is not streaming. Enable using function 'startStream'");
      return respirationData_;
  }
    if (x4m200_.peek_message_respiration_legacy() >0) {
        if (x4m200_.read_message_respiration_legacy(&respirationData_) != 0) {
            logger_->warn("read_message_respiration_legacy failed");
            return respirationData_;
        }
    logger_->debug("Frame: {0}, movement: {1}, respiration_rate(BPM): {2}, "\
        "sensor_state: {3}, signal_quality: {4}",
        respirationData_.frame_counter,
        respirationData_.movement,
        respirationData_.respiration_rate,
        respirationData_.sensor_state,
        respirationData_.signal_quality);
    return respirationData_;
    } else if (x4m200_.peek_message_respiration_legacy() == 0) {
      if (x4m200_.read_message_respiration_legacy(&respirationData_) != 0) {
          logger_->warn("read_message_respiration_legacy failed");
          return respirationData_;
      }
      logger_->debug("No Data available");
      return respirationData_;
    }
    return respirationData_;
}

bool XethruAdapter::startXethruRecording(const std::string& DataPath) {
    if (dataRecorder_.start_recording(dataTypes_, DataPath) != 0) {
        logger_->warn("Failed to start recording");
        return false;
    }
    logger_->debug("Started recording RespirationData");
    return true;
}

bool XethruAdapter::stopXethruRecording() {
    dataRecorder_.stop_recording(dataTypes_);
    logger_->debug("Finished recording RespirationData");
    return true;
}

bool XethruAdapter::stopStream() {
    if (module_ == "X4M200") {
      if (x4m200_.set_sensor_mode(XTID_SM_STOP, 0) != 0) {
        logger_->warn("set_sensor_mode failed");
        return false;
      }
      logger_->debug("X4M200 stopped.");
      return true;
    } else if (module_.empty()) {
      logger_->warn("Sensor type not recognized");
      return false;
    }
    return false;
}

}  // namespace xethruradar
}  // namespace sensors
}  // namespace crf
