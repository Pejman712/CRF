/* © Copyright CERN 2018. All rights reserved. This software is released under a CERN proprietary software license. 
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 * 
 * Author: Pawel Ptasznik CERN EN/STI/ECE
 * 
 * Contributors: Alessandro Mosca CERN EN/STI/ECE, Giacomo Lunghi CERN EN/STI/ECE, 
 * Jorge Camarero Vera CERN EN/STI/ECE, Carlos Veiga Almagro CERN EN/STI/ECE, 
 * David Blanco Mulero CERN EN/STI/ECE, 
 *  ==================================================================================================
 */
#pragma once

#include <memory>
#include <functional>

#include <xsensdeviceapi.h>

#include "IMU/IIMU.hpp"
#include "EventLogger/EventLogger.hpp"

namespace crf {
namespace sensors {
namespace imu {

/**
 * @ingroup group_xsensemt
 * @brief 
 * 
 */
class XsOnDataAvailableCallback: public XsCallback {
 public:
    typedef std::function<void(const Packets::IMUDataPacket&)> UpdateFunction;
    explicit XsOnDataAvailableCallback(UpdateFunction updateFunction);
    ~XsOnDataAvailableCallback() override;

 protected:
    void onDataAvailable(XsDevice* dev, const XsDataPacket* packet) override;

 private:
    Packets::IMUDataPacket translate(const XsDataPacket* packet);
    utility::logger::EventLogger logger_;
    UpdateFunction updateFunction_;
};

/**
 * @ingroup group_xsensemt
 * @brief 
 * 
 */
class XsensMT: public IIMU {
 public:
    XsensMT() = delete;
    explicit XsensMT(XsControl* xsControl);
    ~XsensMT() override;
    bool initialize() override;
    bool deinitialize() override;
    IMUDataPacket getIMUData() override;

    // Additional interface specific for this sensor
    bool hasFreshData() const;

 private:
    bool enableMeasurement();
    bool disableMeasurement();
    void updateRecentData(const IMUDataPacket& packet);
    utility::logger::EventLogger logger_;
    XsControl* xsControl_;
    XsDevice* devicePtr_;
    XsOnDataAvailableCallback onDataAvailableCallback_;
    IMUDataPacket recentData_;
    bool hasUnreadData_;
    bool isInitialized_;
};

}  // namespace imu
}  // namespace sensors
}  // namespace crf
