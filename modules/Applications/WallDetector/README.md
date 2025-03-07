# Wall Detector {#wall_detector}

- There is a known bug with PCL library (not solved yet in v1.9.1.1):

- in order to have RANSAC algorithm work in 2D space, it is necessary to add Z axis, and it should not be set to 0. My approach is to randomize Z values within a negligible distance from 0 using C++ rand() function.
