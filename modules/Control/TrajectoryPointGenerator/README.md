# Trajectory Point Generator {#trajectory_point_generator}

Hereby, we use the **Reflexxes Motion Libraries (RML)** in order to generate smooth motion based on the current state of the system, the target state and motion constraints. Reflexxes ensures that the generated motion command never violates the given motion constraints (e.g. position limits, max. velocity, acceleration or jerk). Generally, there are two possible ways to use the generated output from this implementation:

* As a low-level (safety) interpolator, which ensures that the commands sent to your actuators are always smooth and dynamically feasible
* As a motion profile generator, which creates reference trajectories that obey the given motion constraints

For details on the implementation of Reflexxes, please check the documentation on their website [Reflexxes Motion Libraries](http://reflexxes.ws/software/typeiirml/v1.2.6/docs/page__reflexxes_motion_libraries.html). Please note that there are two different Reflexxes libraries, that provide different features. Within this scope we use the following:

*  Reflexxes Type II: Fully open source (LGPL V3.0). It allows arbitrary initial and target states of motion, as well as limitation of velocity and acceleration

The basic concept is presented in [Opening the Door to New Sensor-Based RobotApplications — The Reflexxes Motion Libraries](http://reflexxes.ws/papers/ReflexxesICRA2011.pdf) and examples are demonstrated [here](http://reflexxes.ws/software/typeiirml/v1.2.6/docs/page__getting_started.html)

## Examples
 *  **VelocityTrajectoryGenerator**: Velocity based implementation in Task space
    * Inputs:
        * Current Task state
        * Target Task (X, Y, Z, Roll, Pitch, Yaw) velocities
        * (optional) MinimumSynchronizationTime
    * Outputs:
        * Smooth motion in form of TaskTrajectoryPointData (position/velocity/acceleration)
    * NOTE: by default: mode is  NO_SYNCHRONIZATION (linear approach of target velocity) and for the moment controlled by setting the maximum acceleration           accordingly. Instead, if we want to have a rather smooth approach, ONLY_TIME_SYNCHRONIZATION should be selected and MinimumSynchronizationTime has to be approximated

 *  **PositionTrajectoryGenerator**: Position based implementation in Task space
     * Inputs:
        * Current Task state
        * Target Task X, Y, Z, Roll, Pitch, Yaw values (position/orientation) and (optionally) target Task (X, Y, Z, Roll, Pitch, Yaw) velocities. For ZYX-euler angles limit the target/maximum orientation accordingly
    * Outputs:
        * Smooth motion in form of TaskTrajectoryPointData (position/velocity/acceleration)
    * NOTE: We use PHASE_SYNCHRONIZATION_IF_POSSIBLE as Synchronization behavior

## Relexxes API

The constructor determines the control mode (velocity/position), the control cycle time and the number of degrees of freedom (using a boolean vector to select the degrees of freedom to be computed).
 
 *  Cycle Time is in seconds. This value should match the period of the component.
 *  Motion constraints that define the properties of the output trajectory. These includes the maximum speed, maximum acceleration and maximum jerk (derivative of acceleration - here we have a default).

NOTE:
 * We use the build-in state validation (CheckValidity) before computing the trajectory points
 * Synchronization behavior for the different degrees of freedom. Can be one of PHASE_SYNCHRONIZATION_IF_POSSIBLE, ONLY_TIME_SYNCHRONIZATION, ONLY_PHASE_SYNCHRONIZATION and NO_SYNCHRONIZATION. See reflexxes/RMLFlags.h for details.
 *  **Keep current velocity when target reached is ON by default.**
