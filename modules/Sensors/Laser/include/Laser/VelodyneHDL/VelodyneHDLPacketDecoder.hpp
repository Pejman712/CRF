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

#pragma once

#include <string>
#include <vector>
#include <deque>

#include "EventLogger/EventLogger.hpp"

namespace crf {
namespace sensors {
namespace laser {

#define toRadians(x) ((x)*M_PI/180.0)

const int HDL_NUM_ROT_ANGLES = 36001;
const int HDL_LASER_PER_FIRING = 32;
const int HDL_MAX_NUM_LASERS = 64;
const int HDL_FIRING_PER_PKT = 12;

enum HDLBlock {
    BLOCK_0_TO_31 = 0xeeff,
    BLOCK_32_TO_63 = 0xddff
};

#pragma pack(push, 1)
typedef struct HDLLaserReturn {
    std::uint16_t distance;
    unsigned char intensity;
} HDLLaserReturn;
#pragma pack(pop)

struct HDLFiringData {
    std::uint16_t blockIdentifier;
    std::uint16_t rotationalPosition;
    HDLLaserReturn laserReturns[HDL_LASER_PER_FIRING];
};

struct HDLDataPacket {
    HDLFiringData firingData[HDL_FIRING_PER_PKT];
    unsigned int gpsTimestamp;
    unsigned char blank1;
    unsigned char blank2;
};

struct HDLLaserCorrection {
    double azimuthCorrection;
    double verticalCorrection;
    double distanceCorrection;
    double verticalOffsetCorrection;
    double horizontalOffsetCorrection;
    double sinVertCorrection;
    double cosVertCorrection;
    double sinVertOffsetCorrection;
    double cosVertOffsetCorrection;
};

struct HDLRGB {
    uint8_t r;
    uint8_t g;
    uint8_t b;
};

class VelodyneHDLPacketDecoder {
 public:
    struct HDLFrame {
        std::vector<double> x;
        std::vector<double> y;
        std::vector<double> z;
        std::vector<unsigned char> intensity;
        std::vector<unsigned char> laser_id;
        std::vector<std::uint16_t> azimuth;
        std::vector<double> distance;
        std::vector<unsigned int> ms_from_top_of_hour;
    };

    VelodyneHDLPacketDecoder();
    virtual ~VelodyneHDLPacketDecoder();
    void SetMaxNumberOfFrames(unsigned int maxFramesNumber);
    bool DecodePacket(std::string* data, unsigned int* dataLength);
    void SetCorrectionsFile(const std::string& correctionsFile_);
    std::deque<HDLFrame> GetFrames();
    void ClearFrames();
    bool GetLatestFrame(HDLFrame* frame);
    void UnloadData();

 protected:
    void InitTables();
    void LoadCorrectionsFile(const std::string& correctionsFile);
    void LoadHDL32Corrections();
    void SetCorrectionsCommon();
    bool ProcessHDLPacket(unsigned char *data, unsigned int dataLength);
    void SplitFrame();
    void PushFiringData(unsigned char laserId, std::uint16_t azimuth, unsigned int timestamp,
        HDLLaserReturn laserReturn, HDLLaserCorrection correction);

 private:
    crf::utility::logger::EventLogger logger_;
    std::string correctionsFile_;
    unsigned int lastAzimuth_;
    unsigned int maxFramesNumber_;
    HDLFrame* frame_;
    std::deque<HDLFrame> frames_;
    double *cosLookupTable_;
    double *sinLookupTable_;
    HDLLaserCorrection laserCorrections_[HDL_MAX_NUM_LASERS];
    unsigned int initialAngle_;
    bool resetAngle_;
};

}  // namespace laser
}  // namespace sensors
}  // namespace crf
