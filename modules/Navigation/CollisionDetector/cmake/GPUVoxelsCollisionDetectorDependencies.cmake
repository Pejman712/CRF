# Check for the modules dependencies
check_module_status(REQUESTER
                        GPUVoxelsCollisionDetector
                    CHECK
                        EventLogger
                        RobotArm
)

# Check for the libraries dependencies
check_library_status(REQUESTER
                         GPUVoxelsCollisionDetector
                     CHECK
                         CUDA
                         icl_core
                         gpu_voxels
                         PCL
)
