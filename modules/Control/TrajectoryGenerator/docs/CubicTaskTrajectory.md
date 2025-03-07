@ingroup group_cubic_task_trajectory

### Class overview
This class generates a time-dependent interpolation between a given set of appended task space points. The generated trajectory starts at an initial pose which is set by the user and is defined such that the given velocity and acceleration limits are satisfied. As such, the following variables much be passed through the constructor.

**Inputs:**
* `TaskVelocity maxVelocity` - The maximum task velocity allowed in the generated trajectory
* `TaskAcceleration maxAcceleration` - The maximum task acceleration allowed in the generated trajectory

**Required calls:**  
To begin the trajectory after the object has been constructed, the initial pose must be set using:  
`void setInitialPose(TaskPose initialPose)`  

**Main usage:**  
Once the initial pose has been set, points can be appended to the trajectory using:  
`void append(const std::vector<TaskPose>& path)`  
This append function can be used to add more points any time after the initial pose has been set, even while the trajectory is running.

Once the initial pose has been set, the trajectory can be evaluated at a given time instance, Tp, using:  
`TaskSignals getTrajectoryPoint(double Tp)`  
Note:
* Going backwards in a trajectory is not permitted, so each evaluation time (Tp) must be larger than the previous evaluation time.
* If no points have been appended, `getTrajectoryPoint(double Tp)` will simply return the initial pose with zero velocity and acceleration.
* If the trajectory is evaluated outside of the defined range of time values, it will return the endpoint pose and acceleration with zero velocity (return start point if evaluated before start time or last point if evaluated after the last time instance).

**Additional functionality:**  
The profile velocity and acceleration can also be updated using the following functions:  
`void setProfileVelocity(const TaskVelocity& newProfileVelocity)`  
`void setProfileAcceleration(const TaskAcceleration& newProfileAcceleration)`

Once a significant number of points have been appended and evaluated, the memory storing the previous trajectories that are no longer being used can be cleared using:  
`void clearMemory()`  
These 'previous trajectories' mentioned above will be explained in the following section.

It is also possible to reset the task trajectory and clear all stored data:  
`void reset()`  
Since the initial pose cannot be re-set after points have been appended, to set it again, first call `reset()` then
`setInitialPose(TaskPose initialPose)`.

To see if the trajectory is still running (if there are still points left to evaluate in the trajectory), call:  
`bool isTrajectoryRunning()`  

<div style="text-align: center;">
<table>
<caption id="multi_row">Summary of contributors</caption>
<tr>    <th>@Name      <th>Role     <th>Company             <th>Year
<tr><td>@Chelsea Davidson     <td>Author  <td>CERN - BE/CEM/MRO       <td>2024
</table>
</div>  

### Algorithm Explanation
This algorithm works by combining a collection of trajectories to create one smooth trajectory. Each trajectory is defined as a TaskTrajectory object which has a linear component and an angular component. The linear component is a shared pointer to an array of 3 CubicPolynomial objects, which store the linear translation trajectories for each axis (x, y, z) (see Table 1). The angular component is a shared pointer to a CubicOrientationSpline object which stores the orientation trajectory. These 4 trajectories are calculated individually but all require a set of path points, a set of time instances when the trajectory reaches these path points, a starting velocity, and a finishing velocity. Please see the documentation for CubicPolynomial and CubicOrientationSpline for further information. The combination of these TaskTrajectory objects can be seen in the diagram below where two trajectories have been combined to create one smooth trajectory.

<div style="text-align: center;">
<img src="https://cernbox.cern.ch/remote.php/dav/public-files/N2N3GkhnlJn5p3e/CombinedTaskTrajectory.png?scalingup=0&preview=1&a=1&c=136116185149210624%3A892bf8ff&x=1920&y=1920" alt="Image" style="width:auto;height:auto;" />
</div>

The combination of task trajectories is done by defining a map of these smaller trajectories with the start time of that trajectory as the key (see Table 1). Thus, to find the trajectory point at a certain time, the individual trajectory defined for that time is found and each component (3 CubicPolynomials and 1 CubicOrientationSpline) is evaluated. This trajectory that was evaluated is then stored as the currentTrajectory_ and the evaluation time is stored as the lastEvaluationPoint_ so that the program knows where in the overall trajectory the user is evaluating (it is assumed and enforced that the user evaluates at successive times).

<div style="text-align: center;">
<table>
<caption id="multi_row">Table 1: Map Elements</caption>
<tr>    <th>Key      <th>Value
<div style="text-align: left;">
<tr><td>double startTime     <td>TaskTrajectory:
                                  * linear:
                                    * [0]: CubicPolynomial xTrajectory
                                    * [1]: CubicPolynomial yTrajectory
                                    * [2]: CubicPolynomial zTrajectory
                                  * angular:
                                    * CubicOrientationSpline orientationTrajectory
</div>
</table>  
</div>

**Important class variables:**
* `std::vector<TaskPose> pathPoints_` - set of task poses that the trajectory needs to pass through
* `std::vector<double> ranges_` - list of times at which the trajectory reaches the path points
* `std::atomic<double> lastEvaluationPoint_` - time at which the trajectory was last evaluated
* `TaskTrajectory currentTrajectory_` - trajectory where lastEvaluationPoint_ is defined
* `TaskTrajectory lastTrajectory_` - the most recent trajectory added (trajectory defined over the end times in ranges_)
* `std::map<double, TaskTrajectory> trajectories_` - map storing each trajectory and its start time  
<br>
<br>

### Append algorithm
When `void append(const std::vector<TaskPose>& path)` is called, a new TaskTrajectory is created and added to the map of trajectories with the relevant start time. To make this new TaskTrajectory, the points in the existing trajectory (global path points) are used as the starting points, and combined with the new path points entered into the append() path parameter (new path points) to make the new combined trajectory as smooth as possible. Since this start time will be the cross-over point in the overall trajectory, the smoothness is created by setting the pose and velocity at this start point to be the same as it was defined in the existing overall trajectory. The selection of these starting points depends on where in the existing trajectory the user is evaluating. The different appending conditions and the corresponding starting points are outlined below. The example plots show the state of the trajectory when it satisfies each append type condition (the input points are depicted on the plots).  
<br>

**1. First trajectory append:**
<div style="display: flex; justify-content: center; width: auto;">
  <img src="https://cernbox.cern.ch/remote.php/dav/public-files/s4LCNW9y5hVcbso/InitialPose.png?scalingup=0&preview=1&a=1&c=136116185417646080%3Aa10e1b6e&x=1920&y=1920" alt="Image" style="width: 45%; height: auto; margin-right: 0px;" />
  <img src="https://cernbox.cern.ch/remote.php/dav/public-files/MhPOiEoVud8UyMX/FirstTrajectoryAppend.jpg?scalingup=0&preview=1&a=1&c=136116184075468800%3Abeaf4323&x=1920&y=1920" alt="Image" style="width: 45%; height: auto; margin-left: 0px;" />
</div>
<div style="margin-top: 15px;">
  <ul>
    <li><em>Condition:</em><br>&emsp;Only the initial pose has been specified, no other points have been appended</li>
    <li><em>Start pose:</em><br>&emsp;Initial pose</li>
    <li><em>Start velocity:</em><br>&emsp;Zero (start stationary)</li>
    <li><em>Start time:</em><br>&emsp;Last evaluation point. Since the initial pose can be evaluated before other points have been appended, the first trajectory must start from the last time the initial pose was evaluated (seen in Example on the left). However, if the trajectory hasn't been evaluated, the last evaluation time should be 0 (seen in Example on the right)</li>
  </ul>
</div>  

**2. Smooth trajectory append:**
<div style="display: flex; align-items: flex-start; width: 100%;">
  <div style="flex: 1; margin-right: 0px;">
    <img src="https://cernbox.cern.ch/remote.php/dav/public-files/VEVW1qU341OMq13/SmoothTrajectoryAppend.png?scalingup=0&preview=1&a=1&c=136116185954516992%3A14be2a14&x=1920&y=1920" alt="Image" style="width: 100%; height: auto;" />
  </div>
  <div style="flex: 1;">
    <ul>
      <li><em>Condition:</em><br>&emsp;The last evaluation point is at or before the second last global path point (see Example on the left)</li>
      <li><em>Start pose:</em><br>&emsp;Pose at second last global path point</li>
      <li><em>Start velocity:</em><br>&emsp;Velocity evaluated at the second last global path point</li>
      <li><em>Start time:</em><br>&emsp;Time instance of the second last global path point</li>
      <li><em>Next start pose:</em><br>&emsp;Pose at last global path point</li>
    </ul>
  </div>
</div>  

**3. End trajectory append:**
<div style="display: flex; align-items: flex-start; width: 100%;">
  <div style="flex: 1; margin-right: 0px;">
    <img src="https://cernbox.cern.ch/remote.php/dav/public-files/8c2DTupphAHMu3E/EndTrajectoryAppend.png?scalingup=0&preview=1&a=1&c=136116184612339712%3Ac0ff38dd&x=1920&y=1920" alt="Image" style="width: 100%; height: auto;" />
  </div>
  <div style="flex: 1;">
    <ul>
      <li><em>Condition:</em><br>&emsp;The last evaluation point is before the last global path point but after the second last global path point (see Example on the left)</li>
      <li><em>Start pose:</em><br>&emsp;Pose at last evaluation point</li>
      <li><em>Start velocity:</em><br>&emsp;Velocity evaluated at last evaluation point</li>
      <li><em>Start time:</em><br>&emsp;Last evaluation point</li>
      <li><em>Next start pose:</em><br>&emsp;Pose at last global path point</li>
    </ul>
  </div>
</div>  

**4. Finished trajectory append:**
<div style="display: flex; align-items: flex-start; width: 100%;">
  <div style="flex: 1; margin-right: 0px;">
    <img src="https://cernbox.cern.ch/remote.php/dav/public-files/25FSMM5QPY4ENjP/FinishedTrajectoryAppend.png?scalingup=0&preview=1&a=1&c=136116184880775168%3A0e5ad93e&x=1920&y=1920" alt="Image" style="width: 100%; height: auto;" />
  </div>
  <div style="flex: 1;">
    <ul>
      <li><em>Condition:</em><br>&emsp;The last evaluation point is at or after the last global path point</li>
      <li><em>Start pose:</em><br>&emsp;Pose at last evaluation point (should be same as last global path point - see Note below)</li>
      <li><em>Start velocity:</em><br>&emsp;Velocity evaluated at last evaluation point (should be zero)</li>
      <li><em>Start time:</em><br>&emsp;Last evaluation point</li>
    </ul>
  </div>
</div>
Note:  
When creating a new TaskTrajectory during the append, the final velocity is always set to zero. Likewise, when creating a first trajectory append, the initial velocity is set to zero. CubicPolynomial and CubicOrientationSpline are defined such that they will return the endpoint when the evaluated time is beyond the defined range and the corresponding endpoint velocity is zero (eg. if end velocity is zero and the last time instance is 10s, it would return the last point when the trajectory is evaluated at time 15s). Thus, these TaskTrajectories will remain constant at the endpoints, leading to the Note in the 'Main usage' section above.  
<br>
These starting poses are combined with the new path points to generate a complete path for this new trajectory. Since the x, y, z, and orientation trajectories of the TaskTrajectory are all independent of each other, they are computed separately. Thus, these path points need to be decomposed into x, y, z, and orientation points and used to generate the individual component trajectories. However, it is important that the TaskTrajectory hits the desired TaskPoses at the same time. To achieve this, the most restrictive movement (either x, y, z translation or orientation rotation) is found and the time taken to achieve this movement while satisfying the velocity and acceleration constraints are computed. This is done in the two `getMovementTime()` functions. This is then set as the time instance of that pose in all component trajectories.  
<br>

### Get trajectory point algorithm
When `TaskSignals getTrajectoryPoint(double Tp)` is called, the TaskTrajectory defined for that evaluation time, Tp, is determined, and its component trajectories (x, y, z, and orientation) and their derivative trajectories are evaluated at that given evaluation time. This can be seen below where the Tp value is defined within the 2nd TaskTrajectory. Thus, TaskTrajectory 2 is the trajectory that should be evaluated. The evaluated position and orientation can be combined to get the evaluated TaskPose, the evaluated linear and angular velocity can be combined to get the evaluated TaskVelocity, and the evaluated linear and angular acceleration can be combined to get the evaluated TaskAcceleration. 
<div style="text-align: center;">
<img src="https://cernbox.cern.ch/remote.php/dav/public-files/wda46u3cGu7xNLU/Evaluate.png?scalingup=0&preview=1&a=1&c=136116184343904256%3A97cc46dd&x=1920&y=1920" alt="Image" style="width: auto;height:auto;" />
</div>
