/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Julia Kabalar CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#include <memory>
#include <vector>
#include <chrono>
#include <thread>

#include "TrajectoryPointGenerator/ITrajectoryPointGenerator.hpp"
#include "TrajectoryPointGenerator/ReflexxesTrajectoryPointGenerator.hpp"


int main(int argc, char* argv[]) {
    int dofs = 6;
    // Select all degrees of freedom in TASK_SPACE
    std::vector<bool> selectedDim;
    for (int i = 0; i < dofs; i++) {
        selectedDim.push_back(true);
    }
    float cycleTimeInS = 0.001;
    float defaultOffsetAllowed = 0.001;

    crf::control::trajectorypointgenerator::ReflexxesTrajectoryPointGenerator generator(
        crf::control::trajectorypointgenerator::ControlMode::POSITION, selectedDim, cycleTimeInS);
    crf::utility::types::TaskTrajectoryData initialState;
    crf::utility::types::TaskPose startPosition(Eigen::Vector3d({10.0, 2.0, 2.0}),
                                                crf::math::rotation::CardanXYZ({2.0, 2.0, 2.0}));
    initialState.pose = startPosition;

    crf::utility::types::TaskTrajectoryData maximalPoint;
    maximalPoint.velocity = {2.0, 2.0, 2.0, 2.0, 2.0, 2.0};
    maximalPoint.acceleration = {2.0, 2.0, 2.0, 2.0, 2.0, 2.0};
    generator.updateMotionConstraints(maximalPoint);

    crf::utility::types::TaskVelocity targetVelocity({1.0, 1.0, 1.0, 1.0, 1.0, 1.0});
    generator.updateVelocityTarget(targetVelocity);

    crf::utility::types::TaskPose targetPose(Eigen::Vector3d({20.0, 2.0, 2.0}),
                                             crf::math::rotation::CardanXYZ({2.0, 2.0, 2.0}));
    generator.updatePositionTarget(targetPose);

    crf::utility::types::TaskTrajectoryData result;

    generator.updateCurrentState(initialState);

    while (std::abs(targetPose.getPosition()(0) - result.pose.getPosition()(0)) >
        defaultOffsetAllowed) {
        result = generator.getTaskTrajectoryPoint().get();
        // here fooling the generator by passing planned trajectory point as current one
        generator.updateCurrentState(result);
        std::cout << result.pose << std::endl;
        std::this_thread::sleep_for(std::chrono::duration<float>(cycleTimeInS));
    }
    return 0;
}
