/* © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Alejandro Diaz Rosales CERN BE/CEM/MRO 2021
 *
 *  ==================================================================================================
 */

#pragma once

#include <string>
#include <map>
#include <utility>

#include "EventLogger/EventLogger.hpp"

namespace crf::actuators::tim {

enum class LHCObstacleType {
    NotDefined = 0,
    SectorDoor = 1,
    VentialtionDoor = 2,
    Chicane = 3,
    Shielding = 4,
    OuterLimits = 5
};

/**
 * @brief Class that works as container to save the information related an obstacle in the LHC.
 */
class LHCObstacle {
 public:
    LHCObstacle();
    LHCObstacle(unsigned int identifier, LHCObstacleType type,
        float start, float end, float vel, bool retract);
    LHCObstacle(const LHCObstacle&);
    ~LHCObstacle() = default;
    LHCObstacle& operator=(LHCObstacle other);

    /**
     * @brief Resets the values of all the variables.
     */
    void clear();
    /**
     * @brief Function to check if the obstacle is empty.
     * 
     * @return true if is empty
     * @return false if is not.
     */
    bool isEmpty() const;
    /**
     * @brief Gives the number that tells which is the obstacle.
     * 
     * @return int8_t numeric identifier.
     */
    int8_t identifier() const;
    void identifier(int8_t input);
    /**
     * @brief Tells the type of obstacle given its charactersitics
     * 
     * @return LHCObstacleType
     */
    LHCObstacleType type() const;
    void type(LHCObstacleType input);
    /**
     * @brief Gives the point in which the obstacle area start.
     * 
     * @return float the position in DCUM (meters)
     */
    float startPosition() const;
    void startPosition(float input);
    /**
     * @brief Gives the point in which the obstacle area ends.
     * 
     * @return float the position in DCUM (meters)
     */
    float endPosition() const;
    void endPosition(float input);
    /**
     * @brief Gives the absolute value of the maximum velocity allowed along side the obstacle
     *        area.
     * 
     * @return float the velocity in m/s
     */
    float maximumVelocity() const;
    void maximumVelocity(float input);
    /**
     * @brief Tells if the deployable devices must be retracted to cross the obstacle section.
     * 
     * @return true if the devices have to be retracted.
     * @return false if the devices can be deployed.
     */
    bool mustRetractDevices() const;
    void mustRetractDevices(bool input);

 private:
    utility::logger::EventLogger logger_;
    std::atomic<bool> isEmpty_;

    std::atomic<int8_t> identifier_;
    std::atomic<LHCObstacleType> type_;
    std::atomic<float> startPosition_;
    std::atomic<float> endPosition_;
    std::atomic<float> maximumVelocity_;
    std::atomic<bool> mustRetractDevices_;
};

/**
 * @brief Compares LHCObstacle objects. If the identifiers are the same we consider them the
 *        same object but we organize them acording to the start position.
 */
struct CompareLHCObstacle {
    bool operator()(const LHCObstacle &obs1, const LHCObstacle &obs2) const {
        if (obs1.identifier() == obs2.identifier()) {
            return false;
        }
        return obs1.startPosition() < obs2.startPosition();
    }
};

}  // namespace crf::actuators::tim
