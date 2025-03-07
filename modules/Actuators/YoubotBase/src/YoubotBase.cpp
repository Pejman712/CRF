/* Â© Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Giacomo Lunghi CERN EN/SMM/MNRO
 *
 *  ==================================================================================================
 */
#include <memory>
#include <vector>
#include <utility>
#include <string>

#include "YoubotBase/YoubotBase.hpp"
#include "RobotBase/WheelsOdometry.hpp"

namespace crf {
namespace robots {

YoubotBase::YoubotBase(const std::string& robotConfigFileName) :
    RobotBase("Youbot", robotConfigFileName) {
    _baseOdometry = std::make_shared<WheelsOdometry>(_wheelsPositionX,
                    _wheelsPositionY,
                    _wheelsDiameter/2);
    _motorsVelocity.resize(_configuration->getNumberOfWheels());
    _motorsCurrent.resize(_configuration->getNumberOfWheels());
}

YoubotBase::~YoubotBase() {
    if (_controlThread.joinable()) {
        deinitialize();
    }
}

bool YoubotBase::initialize() {
    try {
        base = std::make_unique<youbot::YouBotBase>("youbot-base",
                "/home/robotronics/youbot_driver/config");

        _targetLinearVelocity = 0.0;
        _targetAngularVelocity = 0.0;
        _targetTransversalVelocity = 0.0;
    } catch (std::exception& ex) {
        cout << "WHAT: " << ex.what() << endl;

        return false;
    }

    try {
        base->doJointCommutation();
    } catch (std::exception& ex) {
        return false;
    }

    _stopThread = false;
    _controlThread = std::thread(&YoubotBase::run, this);

    return true;
}

bool YoubotBase::deinitialize() {
    _stopThread = true;

    if (_controlThread.joinable()) {
        _controlThread.join();
    }

    return true;
}

bool YoubotBase::move(float linear, float angular, float transversal) {
    gettimeofday(&last_command_time, NULL);

    if (linear > _maxLinearVelocity) {
        linear = _maxLinearVelocity;
    } else if (linear < -_maxLinearVelocity) {
        linear = -_maxLinearVelocity;
    }

    if (angular> _maxAngularVelocity) {
        angular = _maxAngularVelocity;
    } else if (angular < -_maxAngularVelocity) {
        angular = -_maxAngularVelocity;
    }

    if (transversal> _maxTransversalVelocity) {
        transversal = _maxTransversalVelocity;
    } else if (transversal < -_maxTransversalVelocity) {
        transversal = -_maxTransversalVelocity;
    }

    _targetLinearVelocity = linear;
    _targetAngularVelocity = angular;
    _targetTransversalVelocity = transversal;
}

void YoubotBase::getVelocity() {
    base->getBaseVelocity(lv, tv, av);
    _linearVelocity = tv.value();
    _transversalVelocity = tv.value();
    _angularVelocity = av.value();

    _actualPosition = _actualPosition*_baseOdometry->GetOdometryStep(_linearVelocity,
                                                    _angularVelocity,
                                                    _transversalVelocity);

    std::vector<youbot::JointSensedVelocity> velocityData;
    std::vector<youbot::JointSensedCurrent> currentData;
    base->getJointData(velocityData);
    base->getJointData(currentData);

    for (int i=0; i < 4; i++) {
        _motorsVelocity[i] = velocityData[i].angularVelocity.value();
        _motorsCurrent[i] = currentData[i].current.value();
    }
}


void YoubotBase::setVelocity(float linear, float angular, float transversal) {
    lv = linear*meter_per_second;
    tv = transversal*meter_per_second;
    av = angular*radian_per_second;
    base->setBaseVelocity(lv, tv, av);
}

void YoubotBase::run() {
    timeval start, end;

    while (!_stopThread) {
        getVelocity();

        float next_vel[3];
        next_vel[0] = _targetLinearVelocity;
        next_vel[1] = _targetAngularVelocity;
        next_vel[2] = _targetTransversalVelocity;

        timeval now;
        gettimeofday(&now, NULL);
        int64_t millscmd = last_command_time.tv_sec*1000 + last_command_time.tv_usec/1000;
        int64_t millsnow = now.tv_sec*1000+now.tv_usec/1000;

        if (millsnow - millscmd < 500) {
            setVelocity(_targetLinearVelocity, _targetAngularVelocity, _targetTransversalVelocity);
        } else {
            setVelocity(0.0f, 0.0f, 0.0f);
        }
    }
}

}  // namespace robots
}  // namespace crf
