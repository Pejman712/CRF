@ingroup group_motion_controller

The motion controller is a class that contains the main control loop of our robots. It keeps a control loop that keeps track of the robot signals and can be used for trajectory execution among other features.

The main design philosophy behind this class is the idea that it could be reused regardless of the controller that we want to use and should be flexible enough to adapt to any needs.
