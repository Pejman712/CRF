#pragma once
/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Giacomo Lunghi CERN EN/SMM/MNRO
 *
 *  ==================================================================================================
 */

#include <sys/time.h>

#include <string>
#include <memory>
#include <utility>
#include <vector>

#include <youbot/YouBotBase.hpp>

#include <RobotBase/RobotBase.hpp>

namespace crf {
namespace robots {

class YoubotBase : public RobotBase {
 public:
    explicit YoubotBase(const std::string& robotConfigFileName);
    ~YoubotBase() override;
    bool initialize() override;
    bool deinitialize() override;
    bool move(float linear, float angular, float transversal) override;

 private:
    struct timeval last_command_time;
    int64_t new_time, previous_time;
    float dx, dy, dyaw;
    float base_x, base_y, base_yaw;
    std::unique_ptr<youbot::YouBotBase> base;
    quantity<si::velocity> lv;
    quantity<si::velocity> tv;
    quantity<si::angular_velocity> av;
    void setVelocity(float linear, float angular, float transversal);
    void getVelocity() override;
    void run();
    std::atomic<bool> _stopThread;
    std::thread _controlThread;
};

}   // namespace robots
}   // namespace crf
