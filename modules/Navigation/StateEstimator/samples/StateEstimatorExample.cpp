/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license. 
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 * 
 * Author: Julia Kabalar CERN EN/SMM/MRO
 * 
 *  ==================================================================================================
 */

#include <vector>
#include <memory>

#include "StateEstimator/DefaultSystemModel.hpp"
#include "StateEstimator/DefaultMeasurementModel.hpp"
#include "StateEstimator/StateEstimator.hpp"

#define STATEVECTORSIZE 6
#define INPUTVECTORSIZE 3

int main() {
    auto measurementModel =
        std::make_shared<crf::algorithms::stateestimator::
        DefaultMeasurementModel<STATEVECTORSIZE, INPUTVECTORSIZE>>();
    auto systemModel =
        std::make_shared<crf::algorithms::stateestimator::
        DefaultSystemModel<STATEVECTORSIZE>>();
    auto type =
        crf::algorithms::stateestimator::StateEstimatorFilterType::UNSCENTED_KF;
    std::array<float, STATEVECTORSIZE> initialState{.0, .0, .0, .0, .0, .0};
    crf::algorithms::stateestimator::StateEstimator<STATEVECTORSIZE,
        INPUTVECTORSIZE> stateEstimator(initialState, type, measurementModel, systemModel);

    auto initialEstimate = stateEstimator.getEstimate();
    for (const auto& s : initialEstimate) {
        std::cout << s << ' ';
    }
    std::cout << std::endl;

    std::vector<float> measurementVector{1.0, 1.0, 1.0};
    // Adding some measurements (here constant measurement)
    stateEstimator.addMeasurement(measurementVector);
    stateEstimator.addMeasurement(measurementVector);
    stateEstimator.addMeasurement(measurementVector);
    stateEstimator.addMeasurement(measurementVector);
    auto estimatedState = stateEstimator.getEstimate();
    for (const auto& s : estimatedState) {
        std::cout << s << ' ';
    }
    std::cout << std::endl;
    return 0;
}
