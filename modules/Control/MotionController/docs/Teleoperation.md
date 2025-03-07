@ingroup group_teleoperation

The implementation teleoperation is mainly thought for users to teleoperate with it. It includes an input shaper to smoothen the input from the user and has a timer to ensure that the robot is moving with user control.

If the user stops giving signals to the controller then the velocity reference will slowly move to 0 to ensure a safe stop of the robot.
