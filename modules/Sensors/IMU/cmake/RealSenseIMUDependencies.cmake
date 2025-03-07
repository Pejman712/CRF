# Check for the libraries dependencies
check_module_status(REQUESTER
RealSenseIMU
                    CHECK
                        VisionUtility
)

check_library_status(REQUESTER
                         RealSenseIMU
                     CHECK
                         realsense2
)