# State Estimator {#state_estimator}

This module uses the [kalman](https://github.com/mherb/kalman) (header only) library from Markus Herb (affiliated with Technical University Munich Copyright (c) 2015 mherb).

## Usage of kalman filter library

The *state vector* defines the state variables of your system that should be estimated. You can use the parameters in the template interface which will be later used to define the Kalman::Vector size.

The *system model* defines how the system state evolves over time, i.e. from one time-step to the next given some control input. The transition function is in general non-linear. Any system model must derive from the base SystemModel class template. In case a linearized filter such as the Extended Kalman Filter should be used, then the system model must be given as linearized model by deriving from LinearizedSystemModel and defining the corresponding jacobians. Note that linearized models can of course also be used with fully non-linear filters such as the Unscented Kalman Filter.

In case your system has some control input, a *control vector* can be defined analogously to the state vector.

The *measurement vector* represents the measurement taken by some sensors and has to be defined analogously to the state and control vectors.

The *measurement model* defines how a measurement is related to the system state, i.e. it maps a system state to the expected sensor measurement. Measurement models must derive from the class template MeasurementModel or, in case of linearized models for EKFs, from LinearizedMeasurementModel.

## Known Issues

 * Eigen Memory Allignment issue requires to add the memory alignment macros. Until next release targeted using [this summary](https://eigen.tuxfamily.org/dox/group__TopicStructHavingEigenMembers.html)
