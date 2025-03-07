/* © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Jorge Playán Garai CERN BE/CEM/MRO
 *         Jose Vicente Marti Aviles  CERN BE/CEM/MRO
 *
 *
 *  ==================================================================================================
 */

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

#include "Types/Types.hpp"
#include "EventLogger/EventLogger.hpp"

#include "GraphPlot/GraphPlot.hpp"

#define WINDOW_WIDTH 1200
#define WINDOW_HEIGHT 1000

namespace crf {
namespace utility {
namespace graphplot {

void PlotJointSignals(const std::vector<crf::utility::types::JointSignals>& signals) {
    crf::utility::logger::EventLogger logger("PlotSignals");
    uint64_t numberOfJoints = 0;
    for (uint64_t i = 0; i < signals.size(); i++) {
        if (!signals.at(i).positions) continue;
        numberOfJoints = signals.at(i).positions.value().size();
        break;
    }

    if (numberOfJoints == 0) {
        logger->error("No positions recorded");
        return;
    }

    // Iterate through every joint
    for (uint64_t k = 0; k < numberOfJoints; k++) {
        vtkSmartPointer<vtkTable> table = vtkSmartPointer<vtkTable>::New();

        vtkSmartPointer<vtkFloatArray> arrX = vtkSmartPointer<vtkFloatArray>::New();
        arrX->SetName("Iterations (n) (at control loop frequency)");
        table->AddColumn(arrX);

        vtkSmartPointer<vtkFloatArray> pos = vtkSmartPointer<vtkFloatArray>::New();
        pos->SetName("Joint Position");
        table->AddColumn(pos);

        vtkSmartPointer<vtkFloatArray> Vel = vtkSmartPointer<vtkFloatArray>::New();
        Vel->SetName("Joint Velocity");
        table->AddColumn(Vel);

        vtkSmartPointer<vtkFloatArray> acc = vtkSmartPointer<vtkFloatArray>::New();
        acc->SetName("Joint Acceleration");
        table->AddColumn(acc);

        vtkSmartPointer<vtkFloatArray> tqe = vtkSmartPointer<vtkFloatArray>::New();
        tqe->SetName("Joint Torque");
        table->AddColumn(tqe);

        // Make a row for every sample
        table->SetNumberOfRows(signals.size());
        for (uint64_t i = 0; i < signals.size(); i++) {
            table->SetValue(i, 0, i);
            if (signals.at(i).positions)
                table->SetValue(i, 1, signals.at(i).positions.value()[k]);
            if (signals.at(i).velocities)
                table->SetValue(i, 2, signals.at(i).velocities.value()[k]);
            if (signals.at(i).accelerations)
                table->SetValue(i, 3, signals.at(i).accelerations.value()[k]);
            if (signals.at(i).forceTorques)
                table->SetValue(i, 4, signals.at(i).forceTorques.value()[k]);
        }

        vtkSmartPointer<vtkChartXY> chart = vtkSmartPointer<vtkChartXY>::New();
        vtkSmartPointer<vtkContextView> view = vtkSmartPointer<vtkContextView>::New();
        view->GetRenderer()->SetBackground(1.0, 1.0, 1.0);

        // Add chart
        view->GetScene()->AddItem(chart);
        vtkPlot *line = chart->AddPlot(vtkChart::LINE);
        line->SetInputData(table, 0, 1);
        line->SetColor(0, 255, 0, 255);  // Green
        line->SetWidth(2.0);

        line = chart->AddPlot(vtkChart::LINE);
        line->SetInputData(table, 0, 2);
        line->SetColor(255, 0, 0, 255);  // Red
        line->SetWidth(2.0);

        line = chart->AddPlot(vtkChart::LINE);
        line->SetInputData(table, 0, 3);
        line->SetColor(0, 0, 255, 255);  // Blue
        line->SetWidth(2.0);

        line = chart->AddPlot(vtkChart::LINE);
        line->SetInputData(table, 0, 4);
        line->SetColor(255, 0, 255, 255);  // Purple
        line->SetWidth(2.0);

        // View
        view->GetRenderWindow()->SetSize(WINDOW_WIDTH, WINDOW_HEIGHT);
        view->GetRenderWindow()->Render();
        view->GetInteractor()->Initialize();
        view->GetInteractor()->Start();
    }
}

void PlotJointPositionsOverTime(
    std::vector<std::vector<crf::utility::types::JointPositions>> positions,
    std::vector<std::vector<crf::utility::types::JointVelocities>> velocities) {
        crf::utility::logger::EventLogger logger_("PlotJointPositionsOverTime");
        if (positions.size() != velocities.size()) {
            logger_->warn("Poistion and velocities of parameters do not match");
            return;
        }
        // Run through all the vectors of positions
        if (positions.size()%2 == 1) {
            logger_->warn("There is one reference or position missing");
            logger_->warn("The individual joint will be represented alone");
        }
        // Check for an odd number or not represent
        for (uint64_t n = 0; n < positions.size(); n=n+2) {
            // Run through all the joints in the vector
            bool final = false;
            if (n+1 == positions.size()) {
                final = true;
            }

            for (unsigned int k = 0; k < positions.at(n).at(0).size(); k++) {
                // Vector to store all the positions of joint k in the vector
                std::vector<float> jointNPos;
                std::vector<float> targeNPos;
                std::vector<float> jointNVel;
                std::vector<float> targeNVel;

                // Run through all the positions and save the joint k
                // check n and n+1 size
                for (uint64_t  i = 0; i < positions.at(n).size(); i++) {
                    jointNPos.push_back(positions.at(n).at(i)[k]);
                    jointNVel.push_back(velocities.at(n).at(i)[k]);
                    if (!final) {
                        targeNPos.push_back(positions.at(n+1).at(i)[k]);
                        targeNVel.push_back(velocities.at(n+1).at(i)[k]);
                    }
                }

                vtkSmartPointer<vtkTable> table = vtkSmartPointer<vtkTable>::New();
                vtkSmartPointer<vtkFloatArray> arrX = vtkSmartPointer<vtkFloatArray>::New();
                arrX->SetName("Iterations (n) (at control loop frequency)");
                table->AddColumn(arrX);

                vtkSmartPointer<vtkFloatArray> pos = vtkSmartPointer<vtkFloatArray>::New();
                pos->SetName("Joint Position");
                table->AddColumn(pos);

                vtkSmartPointer<vtkFloatArray> Vel = vtkSmartPointer<vtkFloatArray>::New();
                Vel->SetName("Joint Velocity");
                table->AddColumn(Vel);

                if (!final) {
                    vtkSmartPointer<vtkFloatArray> PosRef = vtkSmartPointer<vtkFloatArray>::New();
                    PosRef->SetName("Joint Position Ref");
                    table->AddColumn(PosRef);

                    vtkSmartPointer<vtkFloatArray> VelRef = vtkSmartPointer<vtkFloatArray>::New();
                    VelRef->SetName("Joint Velocity Ref");
                    table->AddColumn(VelRef);
                }

                table->SetNumberOfRows(jointNPos.size());
                for (uint64_t i = 0; i < jointNPos.size(); ++i) {
                    table->SetValue(i, 0, i);
                    table->SetValue(i, 1, jointNPos.at(i));
                    table->SetValue(i, 2, jointNVel.at(i));
                    if (!final) {
                        table->SetValue(i, 3, targeNPos.at(i));
                        table->SetValue(i, 4, targeNVel.at(i));
                    }
                }

                vtkSmartPointer<vtkChartXY> chart = vtkSmartPointer<vtkChartXY>::New();
                vtkSmartPointer<vtkContextView> view = vtkSmartPointer<vtkContextView>::New();
                view->GetRenderer()->SetBackground(1.0, 1.0, 1.0);

                // Add chart
                view->GetScene()->AddItem(chart);
                vtkPlot *line = chart->AddPlot(vtkChart::LINE);
                line->SetInputData(table, 0, 1);
                line->SetColor(0, 255, 0, 255);
                line->SetWidth(2.0);

                line = chart->AddPlot(vtkChart::LINE);
                line->SetInputData(table, 0, 2);
                line->SetColor(255, 0, 0, 255);
                line->SetWidth(2.0);


                if (!final) {
                    line = chart->AddPlot(vtkChart::LINE);
                    line->SetInputData(table, 0, 3);
                    line->SetColor(0, 255, 255, 255);
                    line->SetWidth(2.0);

                    line = chart->AddPlot(vtkChart::LINE);
                    line->SetInputData(table, 0, 4);
                    line->SetColor(0, 0, 255, 255);
                    line->SetWidth(2.0);
                }

                // View
                view->GetRenderWindow()->SetSize(WINDOW_WIDTH, WINDOW_HEIGHT);
                view->GetRenderWindow()->Render();
                view->GetInteractor()->Initialize();
                view->GetInteractor()->Start();
            }
        }
}

void PlotTaskPoseOverTime(
    std::vector<std::vector<crf::utility::types::TaskPose>> positions) {
    crf::utility::logger::EventLogger logger_("PlotTaskPoseOverTime");

    // Set up a 3D scene and add an XYZ chart to it.
    vtkSmartPointer<vtkContextView> view = vtkSmartPointer<vtkContextView>::New();
    view->GetRenderWindow()->SetSize(640, 480);
    vtkSmartPointer<vtkNamedColors> colors = vtkSmartPointer<vtkNamedColors>::New();

    if (positions.size()%2 == 1) {
        logger_->warn("There is one reference or position missing");
        logger_->warn("The individual joint will be represented alone");
    }


    for (uint64_t n = 0; n < positions.size(); n=n+2) {   // CHECK FOR ODD NUMBER OR NOT REPRESENT
        vtkSmartPointer<vtkTable> table3d = vtkSmartPointer<vtkTable>::New();
        vtkSmartPointer<vtkTable> table3dRef = vtkSmartPointer<vtkTable>::New();

        vtkSmartPointer<vtkFloatArray> arrX3d = vtkSmartPointer<vtkFloatArray>::New();
        arrX3d->SetName("X");
        table3d->AddColumn(arrX3d);

        vtkSmartPointer<vtkFloatArray> arrY3d = vtkSmartPointer<vtkFloatArray>::New();
        arrY3d->SetName("Y");
        table3d->AddColumn(arrY3d);

        vtkSmartPointer<vtkFloatArray> arrZ3d = vtkSmartPointer<vtkFloatArray>::New();
        arrZ3d->SetName("Z");
        table3d->AddColumn(arrZ3d);

        if (!positions.at(n+1).empty()) {
            vtkSmartPointer<vtkFloatArray> arrX3dRef = vtkSmartPointer<vtkFloatArray>::New();
            arrX3dRef->SetName("X Ref");
            table3dRef->AddColumn(arrX3dRef);

            vtkSmartPointer<vtkFloatArray> arrY3dRef = vtkSmartPointer<vtkFloatArray>::New();
            arrY3dRef->SetName("Y Ref");
            table3dRef->AddColumn(arrY3dRef);

            vtkSmartPointer<vtkFloatArray> arrZ3dRef = vtkSmartPointer<vtkFloatArray>::New();
            arrZ3dRef->SetName("Z Ref");
            table3dRef->AddColumn(arrZ3dRef);
        }

        table3d->SetNumberOfRows(positions.at(n).size());
        table3dRef->SetNumberOfRows(positions.at(n+1).size());
        for (uint64_t i = 0; i < positions.at(n).size(); i++) {
            table3d->SetValue(i, 0, positions.at(n).at(i).getPosition()(0));
            table3d->SetValue(i, 1, positions.at(n).at(i).getPosition()(1));
            table3d->SetValue(i, 2, positions.at(n).at(i).getPosition()(2));
            if (!positions.at(n+1).empty()) {
                table3dRef->SetValue(i, 0, positions.at(n+1).at(i).getPosition()(0));
                table3dRef->SetValue(i, 1, positions.at(n+1).at(i).getPosition()(1));
                table3dRef->SetValue(i, 2, positions.at(n+1).at(i).getPosition()(2));
            }
        }
        vtkSmartPointer<vtkChartXYZ> chart = vtkSmartPointer<vtkChartXYZ>::New();
        chart->SetGeometry(vtkRectf(5.0, 5.0, 635.0, 475.0));
        view->GetScene()->AddItem(chart);

        // Add a line plot.
        vtkSmartPointer<vtkPlotLine3D> plot = vtkSmartPointer<vtkPlotLine3D>::New();
        plot->SetInputData(table3d);
        plot->GetPen()->SetColorF(colors->GetColor3d("Tomato").GetData());
        view->GetRenderWindow()->SetMultiSamples(0);
        plot->GetPen()->SetWidth(6.0);
        chart->AddPlot(plot);

        // Add a line plot.
        vtkSmartPointer<vtkPlotLine3D> plotRef = vtkSmartPointer<vtkPlotLine3D>::New();
        plotRef->SetInputData(table3dRef);
        plotRef->GetPen()->SetColorF(colors->GetColor3d("Gray").GetData());
        view->GetRenderWindow()->SetMultiSamples(0);
        plotRef->GetPen()->SetWidth(6.0);
        chart->AddPlot(plotRef);

        // Finally render the scene.
        view->GetRenderer()->SetBackground(colors->GetColor3d("Mint").GetData());
        view->GetRenderWindow()->Render();
        view->GetInteractor()->Initialize();
        view->GetInteractor()->Start();

        // Represent RPY orientation
        vtkSmartPointer<vtkTable> tableRPY = vtkSmartPointer<vtkTable>::New();

        vtkSmartPointer<vtkFloatArray> arrX = vtkSmartPointer<vtkFloatArray>::New();
        arrX->SetName("Time");
        tableRPY->AddColumn(arrX);

        vtkSmartPointer<vtkFloatArray> roll = vtkSmartPointer<vtkFloatArray>::New();
        roll->SetName("Roll");
        tableRPY->AddColumn(roll);

        vtkSmartPointer<vtkFloatArray> pitch = vtkSmartPointer<vtkFloatArray>::New();
        pitch->SetName("Pitch");
        tableRPY->AddColumn(pitch);

        vtkSmartPointer<vtkFloatArray> yaw = vtkSmartPointer<vtkFloatArray>::New();
        yaw->SetName("Yaw");
        tableRPY->AddColumn(yaw);

        if (!positions.at(n+1).empty()) {
            vtkSmartPointer<vtkFloatArray> rollRef = vtkSmartPointer<vtkFloatArray>::New();
            rollRef->SetName("Roll Ref");
            tableRPY->AddColumn(rollRef);

            vtkSmartPointer<vtkFloatArray> pitchRef = vtkSmartPointer<vtkFloatArray>::New();
            pitchRef->SetName("Pitch Ref");
            tableRPY->AddColumn(pitchRef);

            vtkSmartPointer<vtkFloatArray> yawRef = vtkSmartPointer<vtkFloatArray>::New();
            yawRef->SetName("Yaw Ref");
            tableRPY->AddColumn(yawRef);
        }

        tableRPY->SetNumberOfRows(positions.at(n).size());
        for (uint64_t i = 0; i < positions.at(n).size(); ++i) {
            tableRPY->SetValue(i, 0, i);
            tableRPY->SetValue(i, 1, positions.at(n).at(i).getCardanXYZ()[0]);
            tableRPY->SetValue(i, 2, positions.at(n).at(i).getCardanXYZ()[1]);
            tableRPY->SetValue(i, 3, positions.at(n).at(i).getCardanXYZ()[2]);

            if (!positions.at(n+1).empty()) {
                tableRPY->SetValue(i, 4, positions.at(n+1).at(i).getCardanXYZ()[0]);
                tableRPY->SetValue(i, 5, positions.at(n+1).at(i).getCardanXYZ()[1]);
                tableRPY->SetValue(i, 6, positions.at(n+1).at(i).getCardanXYZ()[2]);
            }
        }

        vtkSmartPointer<vtkContextView> viewRPY = vtkSmartPointer<vtkContextView>::New();
        viewRPY->GetRenderer()->SetBackground(1.0, 1.0, 1.0);

        vtkSmartPointer<vtkChartXY> chartRPY = vtkSmartPointer<vtkChartXY>::New();

        // Add chart
        viewRPY->GetScene()->AddItem(chartRPY);
        vtkPlot *line = chartRPY->AddPlot(vtkChart::LINE);
        line->SetInputData(tableRPY, 0, 1);
        line->SetColor(0, 0, 255, 255);
        line->SetWidth(2.0);

        line = chartRPY->AddPlot(vtkChart::LINE);
        line->SetInputData(tableRPY, 0, 2);
        line->SetColor(0, 255, 0, 255);
        line->SetWidth(2.0);

        line = chartRPY->AddPlot(vtkChart::LINE);
        line->SetInputData(tableRPY, 0, 3);
        line->SetColor(0, 255, 255, 255);
        line->SetWidth(2.0);

        if (!positions.at(n+1).empty()) {
            line = chartRPY->AddPlot(vtkChart::LINE);
            line->SetInputData(tableRPY, 0, 4);
            line->SetColor(255, 0, 0, 255);
            line->SetWidth(2.0);

            line = chartRPY->AddPlot(vtkChart::LINE);
            line->SetInputData(tableRPY, 0, 5);
            line->SetColor(255, 0, 255, 255);
            line->SetWidth(2.0);

            line = chartRPY->AddPlot(vtkChart::LINE);
            line->SetInputData(tableRPY, 0, 6);
            line->SetColor(255, 255, 0, 255);
            line->SetWidth(2.0);
        }

        viewRPY->GetRenderWindow()->Render();
        viewRPY->GetInteractor()->Initialize();
        viewRPY->GetInteractor()->Start();
    }
}

void PlotData(
    const std::vector<std::vector<float>>& data,
    const std::vector<float>& limits) {
    crf::utility::logger::EventLogger logger_("PlotData");

    if (limits.size() != 6) {
        logger_->error("Incorrect limits specified");
        return;
    }

    vtkSmartPointer<vtkTable> tableX = vtkSmartPointer<vtkTable>::New();
    vtkSmartPointer<vtkTable> tableY = vtkSmartPointer<vtkTable>::New();
    vtkSmartPointer<vtkTable> tableZ = vtkSmartPointer<vtkTable>::New();
    vtkSmartPointer<vtkContextView> view = vtkSmartPointer<vtkContextView>::New();

    view->GetRenderer()->SetBackground(0.65, 0.65, 0.65);

    vtkSmartPointer<vtkFloatArray> arrX = vtkSmartPointer<vtkFloatArray>::New();

    arrX->SetName("Time (s)");
    tableX->AddColumn(arrX);
    tableY->AddColumn(arrX);
    tableZ->AddColumn(arrX);

    vtkSmartPointer<vtkFloatArray> dataX = vtkSmartPointer<vtkFloatArray>::New();
    dataX->SetName("X accel Data");
    tableX->AddColumn(dataX);

    vtkSmartPointer<vtkFloatArray> dataY = vtkSmartPointer<vtkFloatArray>::New();
    dataY->SetName("Y accel Data");
    tableY->AddColumn(dataY);

    vtkSmartPointer<vtkFloatArray> dataZ = vtkSmartPointer<vtkFloatArray>::New();
    dataZ->SetName("Z accel Data");
    tableZ->AddColumn(dataZ);

    tableX->SetNumberOfRows(data.at(0).size());
    tableY->SetNumberOfRows(data.at(0).size());
    tableZ->SetNumberOfRows(data.at(0).size());

    for (uint64_t i = 0; i < data.at(0).size(); i++) {
        tableX->SetValue(i, 0, data.at(0).at(i));       // X-axis values for X-acceleration
        tableX->SetValue(i, 1, data.at(1).at(i));       // Y-axis values for X-acceleration
        tableY->SetValue(i, 0, data.at(0).at(i));       // X-axis values for Y-acceleration
        tableY->SetValue(i, 1, data.at(2).at(i));       // Y-axis values for Y-acceleration
        tableZ->SetValue(i, 0, data.at(0).at(i));       // X-axis values for Z-acceleration
        tableZ->SetValue(i, 1, data.at(3).at(i));       // Y-axis values for Z-acceleration
    }

    vtkSmartPointer<vtkChartXY> chart = vtkSmartPointer<vtkChartXY>::New();

    // Add chart
    view->GetScene()->AddItem(chart);
    vtkPlot *lineX = chart->AddPlot(vtkChart::LINE);
    lineX->SetInputData(tableX, 0, 1);
    lineX->SetColor(45, 154, 33, 255);  // X-accel Green
    lineX->SetWidth(2.0);
    lineX->GetYAxis()->SetTitle("Acceleration (g)");
    lineX->GetXAxis()->SetTitle("Time (s)");
    lineX->GetYAxis()->SetMaximum(limits[0]);
    lineX->GetYAxis()->SetMinimum(limits[1]);
    lineX->GetYAxis()->SetBehavior(1);  // Sets fixed limits for X-accel plot
    lineX->GetYAxis()->AutoScale();

    vtkPlot *lineY = chart->AddPlot(vtkChart::LINE);
    lineY->SetInputData(tableY, 0, 1);
    lineY->SetColor(83, 72, 236, 255);  // Y-accel Blue
    lineY->SetWidth(2.0);
    lineY->GetYAxis()->SetMaximum(limits[2]);
    lineY->GetYAxis()->SetMinimum(limits[3]);
    lineY->GetYAxis()->SetBehavior(1);  // Sets fixed limits for X-accel plot
    lineY->GetYAxis()->AutoScale();

    vtkPlot *lineZ = chart->AddPlot(vtkChart::LINE);
    lineZ->SetInputData(tableZ, 0, 1);
    lineZ->SetColor(239, 35, 35, 255);  // Z-accel Red
    lineZ->SetWidth(2.0);
    lineZ->GetYAxis()->SetMaximum(limits[4]);
    lineZ->GetYAxis()->SetMinimum(limits[5]);
    lineZ->GetYAxis()->SetBehavior(1);  // Sets fixed limits for X-accel plot
    lineZ->GetYAxis()->AutoScale();

    chart->SetShowLegend(true);  // Activate legend
    chart->GetLegend()->SetInline(true);  // Shows legend over plot
    chart->GetLegend()->SetHorizontalAlignment(vtkChartLegend::RIGHT);  // Aligns legend on right
    chart->GetLegend()->SetVerticalAlignment(vtkChartLegend::TOP);  // Aligns legend on top

    view->GetRenderWindow()->SetSize(WINDOW_WIDTH, WINDOW_HEIGHT);  // Sets plot window size
    view->GetRenderWindow()->Render();

    view->GetInteractor()->Initialize();
    view->GetInteractor()->Start();
}

}  // namespace graphplot
}  // namespace utility
}  // namespace crf
