# Check for the modules dependencies
check_module_status(REQUESTER
                        FCLCollisionDetector
                    CHECK
                        EventLogger
                        RobotBase
)

# Check for the libraries dependencies
check_library_status(REQUESTER
                         FCLCollisionDetector
                     CHECK
                         fcl
)
