/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Giacomo Lunghi EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#include <vector>

#include "RobotBase/RobotBaseDefaultKinematics.hpp"

namespace crf::actuators::robotbase {

RobotBaseDefaultKinematics::RobotBaseDefaultKinematics(
    const RobotBaseConfiguration& configuration) :
        lx_(configuration.getRobotParameters().wheelsDistanceX/2),
        ly_(configuration.getRobotParameters().wheelsDistanceY/2), // NOLINT
        wheelsRadius_(configuration.getRobotParameters().wheelsDiameter/2) {
            if (configuration.getNumberOfWheels() != 4) {
                throw std::runtime_error("For the moment, only 4 wheeled robots are supported");
            }
}

boost::optional<utility::types::TaskVelocity>
    RobotBaseDefaultKinematics::getTaskVelocity(const std::vector<float>& wheelsVelocity) { // NOLINT
        if (wheelsVelocity.size() != 4) {
            return boost::none;
        }

        float vx = (wheelsVelocity[0] + wheelsVelocity[1] + wheelsVelocity[2] + wheelsVelocity[3])
            * wheelsRadius_/ 4;
        float vy = (- wheelsVelocity[0] + wheelsVelocity[1] + wheelsVelocity[2] - wheelsVelocity[3])
            * wheelsRadius_/ 4;
        float wz = (- wheelsVelocity[0] + wheelsVelocity[1] - wheelsVelocity[2] + wheelsVelocity[3])
            * wheelsRadius_/ (4 * (lx_ + ly_));
        return utility::types::TaskVelocity({ vx, vy, 0, 0, 0, wz });
}

boost::optional<std::vector<float>> RobotBaseDefaultKinematics::getWheelsVelocity(const utility::types::TaskVelocity& vel) { // NOLINT
        std::vector<float> values;
        float w1 = 1/wheelsRadius_*(vel[0] - vel[1] - (lx_+ ly_)*vel[5]);
        float w2 = 1/wheelsRadius_*(vel[0] + vel[1] + (lx_+ ly_)*vel[5]);
        float w3 = 1/wheelsRadius_*(vel[0] + vel[1] - (lx_+ ly_)*vel[5]);
        float w4 = 1/wheelsRadius_*(vel[0] - vel[1] + (lx_+ ly_)*vel[5]);

        values.push_back(w1);
        values.push_back(w2);
        values.push_back(w3);
        values.push_back(w4);

        return values;
}

}  // namespace crf::actuators::robotbase
