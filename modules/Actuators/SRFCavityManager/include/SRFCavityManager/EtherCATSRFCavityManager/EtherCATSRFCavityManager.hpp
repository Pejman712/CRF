/* © Copyright CERN 2023. All rights reserved. This software is released under a CERN proprietary software licence.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Adrien Luthi CERN EN/SMM/MRO 2023
 *
 *  ===============================================================================================
 */

#pragma once

#include <nlohmann/json.hpp>
#include <memory>
#include <cmath>

#include "crf/expected.hpp"
#include "EtherCATDevices/EtherCATMotor.hpp"
#include "SRFCavityManager/ISRFCavityManager.hpp"

namespace crf::actuators::srfcavityManager {

class EtherCATSRFCavityManager : public ISRFCavityManager {
 public:
    EtherCATSRFCavityManager(const nlohmann::json& configJson,
      std::shared_ptr<crf::devices::ethercatdevices::IEtherCATMotor> cavityMotor);
    ~EtherCATSRFCavityManager() override;

    bool initialize() override;
    bool deinitialize() override;

    void setCavityOrientation(CavityOrientation orientation) override;
    CavityOrientation getCavityOrientation() override;
    bool setCavityType(double cavityLength) override;
    CavityType getCavityType() override;
    crf::expected<bool> enableMotor() override;
    crf::expected<double> getPosition() override;
    crf::expected<bool> setPosition(double position, bool absolute = false) override;
    crf::expected<bool> moveToOrigin() override;
    crf::expected<double> getVelocity() override;
    crf::expected<bool> setVelocity(double velocity) override;
    crf::expected<bool> isTurning() override;
    crf::expected<bool> stop() override;

 private:
    crf::utility::logger::EventLogger logger_;
    std::shared_ptr<crf::devices::ethercatdevices::IEtherCATMotor> cavityMotor_;

    bool isInitialized_;
    bool motorEnabled_;
    CavityType cavType_;
    CavityOrientation cavOr_;
    bool atOrigin_;
    double unitFactor_;
    float velocityProfile_;
    float accelerationProfile_;
    float decelerationProfile_;
    int referenceCavEncoder_;
    float maxVelocity_;
    float minVelocity_;
    int maxCurrent_;
    int maxTorque_;
    int orientation_;
    std::chrono::microseconds posCommandTimeout_;
    std::chrono::microseconds velCommandTimeout_;
    std::chrono::microseconds stopCommandTimeout_;
    std::chrono::microseconds updateInterval_;
    int oneTurnEnc_;

    /**
     * @brief Smallest increment to detect if the cavity angular velocity is constant.
     */
    const float velResolution_ = 0.008726646;
    /**
     * @brief Smallest position error allowed to define if the cavity is at origin.
     */
    const float posResolution_ = 0.001745329;
    /**
     * @brief Smallest velocity value used to detect if the cavity is turning. When the cavity is
     *        stopped, some noise is measured. This value is proportional to this noise value.
     */
    const float motorVelocityNoise_ = 0.001745329;
    /**
     * @brief Cavity length of the 400MHz (LHC) cavity threshold to detect used to detect which
     *        cavity type to set. The real 400MHz LHC one is 1095 mm long but some margin is
     *        added due to possible cavity imperfection.
     */
    const float cavityLenghtLHC = 1150;
    /**
     * @brief Cavity length of the 704MHz (5Cells) cavity threshold to detect used to detect which
     *        cavity type to set. The real 704MHz 5Cells one is 1397 mm long but some margin is
     *        added due to possible cavity imperfection.
     */
    const float cavityLenghtFiveCells = 1400;
    /**
     * @brief Cavity length of the 1.3GHz (FCC) cavity threshold to detect used to detect which
     *        cavity type to set. The real 1.3GHz FCC one is 400 mm long but some margin is added
     *        due to possible cavity imperfection.
     */
    const float cavityLenghtFCC = 430;

    /**
     * @brief This is a blocking function that send the position command to the EtherCATMotor in
     *        relative values. Note that the 'orientation_' variable allow to select the sens
     *        of rotation of the motor and therefore of the cavity.

     * @param position Target position requested in [rad].
     * @return crf::expected<bool> true is the position has been reached
     * @return crf::Code::RequestTimeout if the cavity did not reach the targeted value after
     *         'nextPosPointScanTimeoutSec' seconds (settable in JSON config file).
     */
    crf::expected<bool> setECMotorPosition(double position);
    /**
     * @brief This is a blocking function that waits until the current velocity of the cavity
     *        reaches a constant value of 'vel'.
     *
     * @param vel Constant velocity to reach [rad/sec].
     * @return crf::expected<bool> true if when the velocity is constant or a
     * @return crf::Code::RequestTimeout if the velocity did not reach the constant value 'vel'
     *         after 'velCstTimeout' seconds (settable in JSON config file).
     */
    crf::expected<bool> waitUntilVelocityReached(double vel);
    /**
     * @brief When getting the position of the cavity, it always has to be between [0, 2pi]. Since
     *        the motor used is multiturn, the encoder value get is not necessarily in this range.
     *        Therefore, this function transforms the encoder value 'encoderPos' into the range
     *        [0, 2pi] in encoder value.
     *
     * @param encoderPos The encoder position to set in the encoder frame equivalent of [0, 2pi].
     * @return double, the encoder position in the correct frame.
     */
    double setEncPosInCavFrame(int encoderPos);
    /**
     * @brief This function allows to switch to the 'opMode'. It compares the current one to the
     *        one asked and to the switch only if they are different.
     *
     * @param opMode The mode to set.
     * @return crf::expected<bool> true if the switch is a success and the error code
     * @return crf::Code::RequestToDeviceFailed if not.
     */
    crf::expected<bool> switchOpMode(int8_t opMode);
    /**
     * @brief This function reset the motor if needed and enable it to operation.
     *
     * @return true if the switch is a success, false otherwise.
     */
    bool resetMotorIfInFault();
};

}  // namespace crf::actuators::srfcavityManager
