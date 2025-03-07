/* © Copyright CERN 2022. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Natalija Topalovic CERN BE/CEM/MRO 2022
 *
 *  ==================================================================================================
 */

#pragma once

#include <cstdint>
#include <map>
#include <string>

#include "CANopenDrivers/RegistersSubIndexes.hpp"

namespace crf::devices::canopendrivers {

/**
 * @ingroup group_can_open_drivers
 * @brief Enum class that defines all the 1000s registers which explain the device
 * specifications and the RPDOs and TPDOs in the network.
 * (general registers for 301 CANOpen profile (the communication profile))
 */
struct CiA301Registers {
    enum Type : uint16_t {
        DeviceType = 0x1000,
        ErrorRegister = 0x1001,
        ManufacturerStatus = 0x1002,
        ErrorField = 0x1003,
        CobIDSync = 0x1005,
        CommunicationCyclePeriod = 0x1006,
        SynchronousWindowLength = 0x1007,
        ManufacturerDeviceName = 0x1008,
        ManufacturerHardwareVersion = 0x1009,
        ManufacturerSoftwareVersion = 0x100A,
        GuardTime = 0x100C,
        LifeTimeFactor = 0x100D,
        StoreParameters = 0x1010,
        RestoreDefaultParameters = 0x1011,
        HisghResolutionTimeStamp = 0x1013,
        CobIDEmcy = 0x1014,
        InhibitTimeEmergency = 0x1015,
        ConsumerHartbeatTime = 0x1016,
        ProducerHeartbeatTime = 0x1017,
        IdentityObject = 0x1018,
        ServerSDOParameter1 = 0x1200,
        RPDO1 = 0x1400,
        RPDO2 = 0x1401,
        RPDO3 = 0x1402,
        RPDO4 = 0x1403,
        RPDO5 = 0x1404,
        RPDO6 = 0x1405,
        RPDO7 = 0x1406,
        RPDO8 = 0x1407,
        RPDOMapping1 = 0x1600,
        RPDOMapping2 = 0x1601,
        RPDOMapping3 = 0x1602,
        RPDOMapping4 = 0x1603,
        RPDOMapping5 = 0x1604,
        RPDOMapping6 = 0x1605,
        RPDOMapping7 = 0x1606,
        RPDOMapping8 = 0x1607,
        TPDO1 = 0x1800,
        TPDO2 = 0x1801,
        TPDO3 = 0x1802,
        TPDO4 = 0x1803,
        TPDO5 = 0x1804,
        TPDO6 = 0x1805,
        TPDO7 = 0x1806,
        TPDO8 = 0x1807,
        TPDOMapping1 = 0x1A00,
        TPDOMapping2 = 0x1A01,
        TPDOMapping3 = 0x1A02,
        TPDOMapping4 = 0x1A03,
        TPDOMapping5 = 0x1A04,
        TPDOMapping6 = 0x1A05,
        TPDOMapping7 = 0x1A06,
        TPDOMapping8 = 0x1A07,
        SyncManagerType = 0x1C00,
        RxPDOAssign = 0x1C12,
        TxPDOAssign = 0x1C13,
        SMOutputParameter = 0x1C32,
        SMInputParameter = 0x1C32
    };
};

using CiA301 = CiA301Registers::Type;

const std::map<CiA301, std::string> CiA301RegisterToString = {
    {CiA301::DeviceType, "DeviceType"},
    {CiA301::ErrorRegister, "ErrorRegister"},
    {CiA301::ManufacturerStatus, "ManufacturerStatus"},
    {CiA301::ErrorField, "ErrorField"},
    {CiA301::CobIDSync, "CobIDSync"},
    {CiA301::CommunicationCyclePeriod, "CommunicationCyclePeriod"},
    {CiA301::SynchronousWindowLength, "SynchronousWindowLength"},
    {CiA301::ManufacturerDeviceName, "ManufacturerDeviceName"},
    {CiA301::ManufacturerHardwareVersion, "ManufacturerHardwareVersion"},
    {CiA301::ManufacturerSoftwareVersion, "ManufacturerSoftwareVersion"},
    {CiA301::GuardTime, "GuardTime"},
    {CiA301::LifeTimeFactor, "LifeTimeFactor"},
    {CiA301::StoreParameters, "StoreParameters"},
    {CiA301::RestoreDefaultParameters, "RestoreDefaultParameters"},
    {CiA301::HisghResolutionTimeStamp, "HisghResolutionTimeStamp"},
    {CiA301::CobIDEmcy, "CobIDEmcy"},
    {CiA301::InhibitTimeEmergency, "InhibitTimeEmergency"},
    {CiA301::ConsumerHartbeatTime, "ConsumerHartbeatTime"},
    {CiA301::ProducerHeartbeatTime, "ProducerHeartbeatTime"},
    {CiA301::IdentityObject, "IdentityObject"},
    {CiA301::ServerSDOParameter1, "ServerSDOParameter1"},
    {CiA301::RPDO1, "RPDO1"},
    {CiA301::RPDO2, "RPDO2"},
    {CiA301::RPDO3, "RPDO3"},
    {CiA301::RPDO4, "RPDO4"},
    {CiA301::RPDO5, "RPDO5"},
    {CiA301::RPDO6, "RPDO6"},
    {CiA301::RPDO7, "RPDO7"},
    {CiA301::RPDO8, "RPDO8"},
    {CiA301::RPDOMapping1, "RPDOMapping1"},
    {CiA301::RPDOMapping2, "RPDOMapping2"},
    {CiA301::RPDOMapping3, "RPDOMapping3"},
    {CiA301::RPDOMapping4, "RPDOMapping4"},
    {CiA301::RPDOMapping5, "RPDOMapping5"},
    {CiA301::RPDOMapping6, "RPDOMapping6"},
    {CiA301::RPDOMapping7, "RPDOMapping7"},
    {CiA301::RPDOMapping8, "RPDOMapping8"},
    {CiA301::TPDO1, "TPDO1"},
    {CiA301::TPDO2, "TPDO2"},
    {CiA301::TPDO3, "TPDO3"},
    {CiA301::TPDO4, "TPDO4"},
    {CiA301::TPDO5, "TPDO5"},
    {CiA301::TPDO6, "TPDO6"},
    {CiA301::TPDO7, "TPDO7"},
    {CiA301::TPDO8, "TPDO8"},
    {CiA301::TPDOMapping1, "TPDOMapping1"},
    {CiA301::TPDOMapping2, "TPDOMapping2"},
    {CiA301::TPDOMapping3, "TPDOMapping3"},
    {CiA301::TPDOMapping4, "TPDOMapping4"},
    {CiA301::TPDOMapping5, "TPDOMapping5"},
    {CiA301::TPDOMapping6, "TPDOMapping6"},
    {CiA301::TPDOMapping7, "TPDOMapping7"},
    {CiA301::TPDOMapping8, "TPDOMapping8"},
    {CiA301::SyncManagerType, "SyncManagerType"},
    {CiA301::RxPDOAssign, "RxPDOAssign"},
    {CiA301::TxPDOAssign, "TxPDOAssign"},
    {CiA301::SMOutputParameter, "SMOutputParameter"},
    {CiA301::SMInputParameter, "SMInputParameter"}
};

}  // namespace crf::devices::canopendrivers
