/* © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Jorge Playán Garai CERN BE/CEM/MRO
 *
 *
 *  ==================================================================================================
 */
#pragma once

#include <vector>
#include <memory>

#include <vtkRenderer.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderWindow.h>
#include <vtkSmartPointer.h>
#include <vtkChartXY.h>
#include <vtkTable.h>
#include <vtkPlot.h>
#include <vtkFloatArray.h>
#include <vtkContextView.h>
#include <vtkContextScene.h>
#include <vtkPen.h>

#include <vtkNew.h>
#include <vtkChartXYZ.h>
#include <vtkPlotLine3D.h>
#include <vtkNamedColors.h>
#include <vtkContext3D.h>
#include <vtkCamera.h>

#include <vtkPlotPoints.h>
#include <vtkAxis.h>
#include <vtkChartLegend.h>

#include "Types/Types.hpp"
#include "EventLogger/EventLogger.hpp"

namespace crf {
namespace utility {
namespace graphplot {

/**
 *  @brief Plots the signals of the robot of each joint over time
 *
 *  @param signals Vector of signals from the robot
 *
 */

void PlotJointSignals(
    const std::vector<crf::utility::types::JointSignals>& signals);

/**
 *  @brief Plots the positions and velocities of each joint over time
 *
 *  Should contain the vector of positions and references and another
 *  vector with the velocities and references.
 *  They should be organized in Position and Reference pairs
 *  Example: vector1<vector<pos1>, vector<refPos1>, vector<pos2>, vector<refPos2>, ...>
 *           vector2<vector<vel1>, vector<refVel1>, vector<vel1>, vector<refVel1>, ...>
 *
 */

void PlotJointPositionsOverTime(
    std::vector<std::vector<crf::utility::types::JointPositions>> positions,
    std::vector<std::vector<crf::utility::types::JointVelocities>> velocities);

/**
 *  @brief Represents positions and velocities of each joint over time
 *
 *  Should contain the vector of positions and references and another
 *  vector with the velocities and references.
 *  They should be organized in Position and Reference pairs
 *  Example: vector1<vector<pos1>, vector<refPos1>, vector<pos2>, vector<refPos2>, ...>
 *           vector2<vector<vel1>, vector<refVel1>, vector<vel1>, vector<refVel1>, ...>
 *
 */

void PlotTaskPoseOverTime(
    std::vector<std::vector<crf::utility::types::TaskPose>> positions);

/**
 *  @brief Represents several vectors with float points over time
 *
 *  Should contain a vector of vector of floats
 *  Each vector will represent a line in the same graph.
 *  Example: vector<vector<posx>, vector<posy>, vector<posz>, ...>
 *
 */

void PlotData(
    const std::vector<std::vector<float>>& data,
    const std::vector<float>& limits);

}  // namespace graphplot
}  // namespace utility
}  // namespace crf
