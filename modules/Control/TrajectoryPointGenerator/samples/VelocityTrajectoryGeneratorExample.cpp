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
    float allowedOffsetValue = 0.001;

    crf::control::trajectorypointgenerator::ReflexxesTrajectoryPointGenerator generator(
        crf::control::trajectorypointgenerator::ControlMode::VELOCITY, selectedDim, cycleTimeInS);
    crf::utility::types::TaskTrajectoryData initialState;
    crf::utility::types::TaskVelocity startVelocity({2.0, 2.0, 2.0, 2.0, 2.0, 2.0});
    initialState.velocity = startVelocity;

    crf::utility::types::TaskVelocity targetVelocity({5.0, 5.0, 5.0, 5.0, 5.0, 5.0});
    generator.updateVelocityTarget(targetVelocity);

    crf::utility::types::TaskTrajectoryData maximalPoint;
    maximalPoint.velocity = {2.0, 2.0, 2.0, 2.0, 2.0, 2.0};
    maximalPoint.acceleration = {2.0, 2.0, 2.0, 2.0, 2.0, 2.0};
    generator.updateMotionConstraints(maximalPoint);

    crf::utility::types::TaskTrajectoryData result;

    generator.updateCurrentState(initialState);

    while (std::abs(targetVelocity[0] - result.velocity[0]) > allowedOffsetValue) {
        result = generator.getTaskTrajectoryPoint().get();
        // here fooling the generator by passing planned trajectory point as current one
        generator.updateCurrentState(result);
        std::cout << result.velocity << std::endl;
        std::this_thread::sleep_for(std::chrono::duration<float>(cycleTimeInS));
    }
    return 0;
}
