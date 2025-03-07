@ingroup group_cernbot_sps_three

### DOCS AND MORE THINGIES

TODO: Dimensions of the robot and reference frame, for now it's written

Axis:

- X is forward
- Y is to the side
- Yaw is the turn

Distance from the front wheel to the back wheel is 0.366m
Distance from the left wheel to the right wheel is 0.274m

Diameter of the wheels is 0.156m

### Configuration file

The configuration file is divided into several parts that will be explained in the following parts:

- "MaxCurrent" : 50000 -> WIP legacy value that might be wrong
- "MaxTorque" : 50000 -> WIP legacy value that might be wrong

- "RadToCountRatio": This value refers to the conversion between the increments of the encoder to radians. In this case, the motor has an encoder of 4096 increments with a gearbox reduction of 2:35

$$  4096\frac{counts}{rev_{internal}} . \frac{35}{2}\frac{rev_{internal}}{rev_{external}} . \frac{1}{2\pi}\frac{rev_{external}}{radians} = 11408.5627 \frac{Counts}{Radian}  $$

- "JointSpaceDegreeOfFreedom": Four joints as we treat each wheel as a joint
- "TaskSpaceDegreeOfFreedom": Three as we can move through a plane (X, Y) and rotate over Z (Yaw)
- "ControllerLoopTimeMs": Loop time for the control loop of this robot. Now set to 4ms as to be consistent with the others but it can be faster.
- "JointLimits": The driver is set without position limits so values are set to infinite. The maximum velocity in each wheel's driver is set to $2700deg/sec$ and the acceleration is set to $1000deg/sec^2$. However, this velocity can also be expanded if needed (up to 8000 rpm or aproximaetly 164000deg/s). Probably a common approach should be used to fix these velocities for all robots. This values can be set into $rad/s$ and $rad/s^2$ following the next transformations.

$$ 2700\frac{deg}{s} . \frac{2\pi}{360}\frac{rad}{deg} = 47.1239\frac{rad}{s}  $$

$$ 1000\frac{deg}{s^2} . \frac{2\pi}{360}\frac{rad}{deg} = 17.4532\frac{rad}{s^2}  $$

- "TaskLimits": As for the task limits, the maximum velocity of the robot come from the forward kinematics equations for mecanum wheels. The variables $l_{x}$ and $l_{y}$ represent the length from the center of the robot to the wheel in x and y respectively (half of the total measure).

$$  V_{x} = (\omega_{fl} + \omega_{fr} + \omega_{bl} + \omega_{br}) . \frac{r}{4} $$
$$  V_{y} = (-\omega_{fl} + \omega_{fr} + \omega_{bl} - \omega_{br}) . \frac{r}{4} $$
$$  \omega_{z} = (-\omega_{fl} + \omega_{fr} - \omega_{bl} + \omega_{br}) . \frac{r}{4 . (l_{x} + l_{y})} $$

Subtituing the maximum positive or negative angular velocity, and since all wheels have the same maximum velocities, we can simplify the equations to:

$$  V_{xmax} = \omega_{max} . r = 47.1239 * 0.076 = 3.58 \frac{m}{s} $$
$$  V_{ymax} = \omega_{max} . r = 47.1239 * 0.076 = 3.58 \frac{m}{s} $$
$$  \omega_{zmax} = \frac{\omega_{max} . r}{l_{x} + l_{y}} = \frac{47.1239 * 0.076}{0.183 + 0.137} = 11.19 \frac{rad}{s} $$ 

And the same for the accelerations

$$  A_{xmax} = \alpha_{max} . r = 17.4532 * 0.076 = 1.32 \frac{m}{s^2} $$
$$  A_{ymax} = \alpha_{max} . r = 17.4532 * 0.076 = 1.32 \frac{m}{s^2} $$
$$  \alpha_{zmax} = \frac{\alpha_{max} . r}{l_{x} + l_{y}} = \frac{17.4532 * 0.076}{0.183 + 0.137} = 4.14 \frac{rad}{s}$$

As for the profile parameters we set some starting values that don't represent the maximum velocities. Usually around half of it.
