/* © Copyright CERN 2022.  All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Jorge Playan Garai CERN BE/CEM/MRO 2023
 *
 *  ==================================================================================================
*/

#pragma once

#include <memory>
#include <vector>
#include <map>

#include <nlohmann/json.hpp>

#include "EtherCATDevices/IEtherCATMotor.hpp"

#include "CommonInterfaces/IInitializable.hpp"
#include "EventLogger/EventLogger.hpp"

namespace crf::applications::missionmanager::sciencegateway {

enum class Figure {
    /**
     * @brief An undefined state in between figures when it can't read the position
     *
     */
    NotDefined = 0,

    /**
     * @brief First figure in the box
     *
     */
    First = 1,

    /**
     * @brief Second figure in the box
     *
     */
    Second = 2,

    /**
     * @brief Third figure in the box
     *
     */
    Third = 3,

    /**
     * @brief Fourth figure in the box
     *
     */
    Fourth = 4
};

class FigureCube: public utility::commoninterfaces::IInitializable {
 public:
    FigureCube() = delete;
    explicit FigureCube(
        std::shared_ptr<crf::devices::ethercatdevices::IEtherCATMotor> motor);
    ~FigureCube();

    bool deinitialize() override;
    bool initialize() override;

    bool randomizeFigure();
    Figure getCurrentFigure();

 private:
    bool isAlmostEqual(const double& value1, const double& value2, const double& epsilon = 1000);

    std::shared_ptr<crf::devices::ethercatdevices::IEtherCATMotor> motor_;

    crf::utility::logger::EventLogger logger_;

    const std::chrono::seconds timeout_ = std::chrono::seconds(17);

    const std::map<Figure, double> positionsMap = {
        {Figure::First, 21840},
        {Figure::Second, 152027},
        {Figure::Third, 286216},
        {Figure::Fourth, 417444}
    };
    const double defaultVelocity_ = 30000;
    const double deafultAcceleration_ = 30000;
    const double defaultDeceleration_ = 30000;
};

}  // namespace crf::applications::missionmanager::sciencegateway
