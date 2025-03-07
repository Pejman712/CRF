/* © Copyright CERN 2020. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Carlos Prados Sesmero CERN EN/SMM/MRO 
 *
 * Original code
 * Velodyne HDL Packet Driver
 * Nick Rypkema (rypkema@mit.edu), MIT 2017
 * Shared library to decode a Velodyne packet
 *
 *  ==================================================================================================
 */

#include <cmath>
#include <stdint.h>
#include <iostream>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/foreach.hpp>
#include <string>
#include <deque>

#include "Laser/VelodyneHDL/VelodyneHDLPacketDecoder.hpp"

namespace crf {
namespace sensors {
namespace laser {

VelodyneHDLPacketDecoder::VelodyneHDLPacketDecoder():
    logger_("VelodyneHDLPacketDecoder") {
    logger_->debug("CTor");
    maxFramesNumber_ = 10;
    UnloadData();
    InitTables();
    LoadHDL32Corrections();
}

VelodyneHDLPacketDecoder::~VelodyneHDLPacketDecoder() {
    logger_->debug("DTor");
}

void VelodyneHDLPacketDecoder::SetMaxNumberOfFrames(unsigned int maxFramesNumber) {
    if (maxFramesNumber <= 0)
        return;
    maxFramesNumber_ = maxFramesNumber;
    while (frames_.size() >= maxFramesNumber_) {
        frames_.pop_front();
    }
}

bool VelodyneHDLPacketDecoder::DecodePacket(std::string* data, unsigned int* dataLength) {
    const unsigned char* data_char = reinterpret_cast<const unsigned char*>(data->c_str());
    return ProcessHDLPacket(const_cast<unsigned char*>(data_char), *dataLength);
}

bool VelodyneHDLPacketDecoder::ProcessHDLPacket(unsigned char *data, unsigned int dataLength) {
    if (dataLength != 1206) {
        logger_->warn("Data packet is not 1206 bytes");
        return false;
    }
    HDLDataPacket* dataPacket = reinterpret_cast<HDLDataPacket *>(data);
    if (resetAngle_) {
        initialAngle_ = dataPacket->firingData[0].rotationalPosition;
        resetAngle_ = false;
        // std::cout << "initialAngle_: " << initialAngle_ << std::endl;
    }
    for (int i = 0; i < HDL_FIRING_PER_PKT; ++i) {
        HDLFiringData firingData = dataPacket->firingData[i];
        int offset = (firingData.blockIdentifier == BLOCK_0_TO_31) ? 0 : 32;
        if ((((firingData.rotationalPosition < initialAngle_) &&
            (abs(firingData.rotationalPosition - initialAngle_) < 20)) ||
            ((firingData.rotationalPosition < 30) && (initialAngle_ < 30))) &&
            (!resetAngle_)) {
            // if ((firingData.rotationalPosition < initialAngle_) &&
            //    (abs(firingData.rotationalPosition - initialAngle_) < 30)) {
            // if (firingData.rotationalPosition < lastAzimuth_) {
            SplitFrame();
        }
        lastAzimuth_ = firingData.rotationalPosition;
        for (int j = 0; j < HDL_LASER_PER_FIRING; j++) {
            unsigned char laserId = static_cast<unsigned char>(j + offset);
            if (firingData.laserReturns[j].distance != 0.0) {
                PushFiringData(laserId, firingData.rotationalPosition, dataPacket->gpsTimestamp,
                    firingData.laserReturns[j], laserCorrections_[j + offset]);
            }
        }
    }
    return true;
}

void VelodyneHDLPacketDecoder::SplitFrame() {
    if (frames_.size() == maxFramesNumber_-1) {
        frames_.pop_front();
    }
    frames_.push_back(*frame_);
    delete frame_;
    frame_ = new HDLFrame();
    resetAngle_ = true;
}

void VelodyneHDLPacketDecoder::PushFiringData(unsigned char laserId, std::uint16_t azimuth,
    unsigned int timestamp, HDLLaserReturn laserReturn, HDLLaserCorrection correction) {
    double cosAzimuth, sinAzimuth;
    if (correction.azimuthCorrection == 0) {
        cosAzimuth = cosLookupTable_[azimuth];
        sinAzimuth = sinLookupTable_[azimuth];
    } else {
        double azimuthInRadians = toRadians(
            (static_cast<double>(azimuth)/100.0)-correction.azimuthCorrection);
        cosAzimuth = std::cos(azimuthInRadians);
        sinAzimuth = std::sin(azimuthInRadians);
    }
    double distanceM = laserReturn.distance*0.002+correction.distanceCorrection;
    double xyDistance = distanceM*correction.cosVertCorrection-correction.sinVertOffsetCorrection;
    double x = (xyDistance*sinAzimuth-correction.horizontalOffsetCorrection*cosAzimuth);
    double y = (xyDistance*cosAzimuth+correction.horizontalOffsetCorrection*sinAzimuth);
    double z = (distanceM*correction.sinVertCorrection+correction.cosVertOffsetCorrection);
    unsigned char intensity = laserReturn.intensity;
    frame_->x.push_back(x);
    frame_->y.push_back(y);
    frame_->z.push_back(z);
    frame_->intensity.push_back(intensity);
    frame_->laser_id.push_back(laserId);
    frame_->azimuth.push_back(azimuth);
    frame_->distance.push_back(distanceM);
    frame_->ms_from_top_of_hour.push_back(timestamp);
}

void VelodyneHDLPacketDecoder::SetCorrectionsFile(const std::string& correctionsFile) {
    if (correctionsFile == correctionsFile_) {
        return;
    }
    if (correctionsFile.length()) {
        LoadCorrectionsFile(correctionsFile);
    } else {
        LoadHDL32Corrections();
    }
    correctionsFile_ = correctionsFile;
    UnloadData();
}

void VelodyneHDLPacketDecoder::UnloadData() {
    lastAzimuth_ = 65000;
    frame_ = new HDLFrame();
    frames_.clear();
    initialAngle_ = 0;
    resetAngle_ = true;
}

void VelodyneHDLPacketDecoder::InitTables() {
    if (cosLookupTable_ == NULL && sinLookupTable_ == NULL) {
        cosLookupTable_ = static_cast<double*>(
            malloc(HDL_NUM_ROT_ANGLES * sizeof(*cosLookupTable_)));
        sinLookupTable_ = static_cast<double*>(
            malloc(HDL_NUM_ROT_ANGLES * sizeof(*sinLookupTable_)));
        for (unsigned int i = 0; i < HDL_NUM_ROT_ANGLES; i++) {
            double rad = toRadians(i / 100.0);
            cosLookupTable_[i] = std::cos(rad);
            sinLookupTable_[i] = std::sin(rad);
        }
    }
}

void VelodyneHDLPacketDecoder::LoadCorrectionsFile(const std::string& correctionsFile) {
    boost::property_tree::ptree pt;
    try {
        read_xml(correctionsFile, pt, boost::property_tree::xml_parser::trim_whitespace);
    } catch (boost::exception const&) {
        logger_->warn("Error reading calibration file: {}", correctionsFile);
        return;
    }
    BOOST_FOREACH(boost::property_tree::ptree::value_type &v,
        pt.get_child("boost_serialization.DB.points_")) {
        if (v.first == "item") {
            boost::property_tree::ptree points = v.second;
            BOOST_FOREACH(boost::property_tree::ptree::value_type &px, points) {
                if (px.first == "px") {
                    boost::property_tree::ptree calibrationData = px.second;
                    int index = -1;
                    double azimuth = 0;
                    double vertCorrection = 0;
                    double distCorrection = 0;
                    double vertOffsetCorrection = 0;
                    double horizOffsetCorrection = 0;
                    BOOST_FOREACH(boost::property_tree::ptree::value_type &item, calibrationData) {
                        if (item.first == "id_")
                            index = atoi(item.second.data().c_str());
                        if (item.first == "rotCorrection_")
                            azimuth = atof(item.second.data().c_str());
                        if (item.first == "vertCorrection_")
                            vertCorrection = atof(item.second.data().c_str());
                        if (item.first == "distCorrection_")
                            distCorrection = atof(item.second.data().c_str());
                        if (item.first == "vertOffsetCorrection_")
                            vertOffsetCorrection = atof(item.second.data().c_str());
                        if (item.first == "horizOffsetCorrection_")
                            horizOffsetCorrection = atof(item.second.data().c_str());
                    }
                    if (index != -1) {
                    laserCorrections_[index].azimuthCorrection = azimuth;
                    laserCorrections_[index].verticalCorrection = vertCorrection;
                    laserCorrections_[index].distanceCorrection = distCorrection/100.0;
                    laserCorrections_[index].verticalOffsetCorrection = vertOffsetCorrection/100.0;
                    laserCorrections_[index].horizontalOffsetCorrection =
                        horizOffsetCorrection/100.0;
                    laserCorrections_[index].cosVertCorrection = std::cos(
                        toRadians(laserCorrections_[index].verticalCorrection));
                    laserCorrections_[index].sinVertCorrection = std::sin(
                        toRadians(laserCorrections_[index].verticalCorrection));
                    }
                }
            }
        }
    }
    SetCorrectionsCommon();
}

void VelodyneHDLPacketDecoder::LoadHDL32Corrections() {
    double hdl32VerticalCorrections[] = {-30.6700000,  -9.3299999, -29.3300000,  -8.0000000,
        -28.0000000,  -6.6700001, -26.6700000,  -5.3299999, -25.3300000,  -4.0000000, -24.0000000,
         -2.6700001, -22.6700000,  -1.3300000, -21.3300000,   0.0000000, -20.0000000,   1.3300000,
        -18.6700000,   2.6700001, -17.3300000,   4.0000000, -16.0000000,   5.3299999, -14.6700000,
          6.6700001, -13.3300000,   8.0000000, -12.0000000,   9.3299999, -10.6700000,  10.6700000
    };
    for (int i = 0; i < HDL_LASER_PER_FIRING; i++) {
        laserCorrections_[i].azimuthCorrection = 0.0;
        laserCorrections_[i].distanceCorrection = 0.0;
        laserCorrections_[i].horizontalOffsetCorrection = 0.0;
        laserCorrections_[i].verticalOffsetCorrection = 0.0;
        laserCorrections_[i].verticalCorrection = hdl32VerticalCorrections[i];
        laserCorrections_[i].sinVertCorrection = std::sin(toRadians(hdl32VerticalCorrections[i]));
        laserCorrections_[i].cosVertCorrection = std::cos(toRadians(hdl32VerticalCorrections[i]));
    }
    for (int i = HDL_LASER_PER_FIRING; i < HDL_MAX_NUM_LASERS; i++) {
        laserCorrections_[i].azimuthCorrection = 0.0;
        laserCorrections_[i].distanceCorrection = 0.0;
        laserCorrections_[i].horizontalOffsetCorrection = 0.0;
        laserCorrections_[i].verticalOffsetCorrection = 0.0;
        laserCorrections_[i].verticalCorrection = 0.0;
        laserCorrections_[i].sinVertCorrection = 0.0;
        laserCorrections_[i].cosVertCorrection = 1.0;
    }
    SetCorrectionsCommon();
}

void VelodyneHDLPacketDecoder::SetCorrectionsCommon() {
    for (int i = 0; i < HDL_MAX_NUM_LASERS; i++) {
        HDLLaserCorrection correction = laserCorrections_[i];
        laserCorrections_[i].sinVertOffsetCorrection =
            correction.verticalOffsetCorrection*correction.sinVertCorrection;
        laserCorrections_[i].cosVertOffsetCorrection =
            correction.verticalOffsetCorrection*correction.cosVertCorrection;
    }
}

std::deque<VelodyneHDLPacketDecoder::HDLFrame> VelodyneHDLPacketDecoder::GetFrames() {
    return frames_;
}

void VelodyneHDLPacketDecoder::ClearFrames() {
    frames_.clear();
}

bool VelodyneHDLPacketDecoder::GetLatestFrame(VelodyneHDLPacketDecoder::HDLFrame* frame) {
    if (frames_.size()) {
        *frame = frames_.back();
        frames_.pop_back();
        frames_.clear();
        return true;
    }
    return false;
}

}  // namespace laser
}  // namespace sensors
}  // namespace crf
