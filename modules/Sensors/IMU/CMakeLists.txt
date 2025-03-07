#######################################################################################################################
##                                                                                                                   ##
## © Copyright CERN 2023. All rights reserved. This software is released under a CERN proprietary software license.  ##
## Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch ##
##                                                                                                                   ##
## Authors: Julia Kabalar CERN EN/SMM/MRO                                                                            ##
##          Alejandro Diaz Rosales CERN BE/CEM/MRO                                                                   ##
##                                                                                                                   ##
#######################################################################################################################

set(MODULE_NAME "IMU")

if(DISABLE_${MODULE_NAME})
    return()
endif()

set(MODULE_LIBS CommonInterfaces
                ErrorHandler
                EventLogger
)
set(INCLUDE_DIRS ./include
)
set(SRC_FILES
)

if(NOT DISABLE_VMU931)
    set(MODULE_LIBS ${MODULE_LIBS}
                    EventLogger
                    SerialCommunication
    )
    set(INCLUDE_DIRS ${INCLUDE_DIRS}
    )
    set(SRC_FILES ${SRC_FILES}
                  ./src/VMU931/VMU931.cpp
    )
endif()

if(NOT DISABLE_XsensMT)
    set(MODULE_LIBS ${MODULE_LIBS}
                    EventLogger
                    ${XsensApi_LIBRARIES}
    )
    set(INCLUDE_DIRS ${INCLUDE_DIRS}
                     ${XsensApi_INCLUDE_DIRS}
    )
    set(SRC_FILES ${SRC_FILES}
                  ./src/XsensMT/XsensMT.cpp
    )
endif()

if(NOT DISABLE_IMUCommunicationPoint)
    set(MODULE_LIBS ${MODULE_LIBS}
                    EventLogger
                    Sockets
                    DataPackets
                    DataPacketSocket
                    DeviceManager
    )
    set(INCLUDE_DIRS ${INCLUDE_DIRS}
    )
    set(SRC_FILES ${SRC_FILES}
                  ./src/IMUCommunicationPoint/IMUManager.cpp
                  ./src/IMUCommunicationPoint/IMUCommunicationPoint.cpp
                  ./src/IMUCommunicationPoint/IMUCommunicationPointFactory.cpp
    )
endif()

if(NOT DISABLE_Gable)
    set(MODULE_LIBS ${MODULE_LIBS}
                    EventLogger
                    EtherCATDrivers
    )
    set(SRC_FILES ${SRC_FILES}
                  ./src/Gable/GableAbstract.cpp
                  ./src/Gable/SE1/GableSE1.cpp
                  ./src/Gable/SE2SE3/GableSE2SE3.cpp
    )
endif()


if(NOT DISABLE_UnitreeL1IMUSample)
    set(MODULE_LIBS ${MODULE_LIBS}
                    VisionUtility
                    ${UnitreeL1SDK_LIBRARIES}
                    SerialCommunication
    )
    set(INCLUDE_DIRS ${INCLUDE_DIRS}
                     ${UnitreeL1SDK_INCLUDE_DIRS}
    )
    set(SRC_FILES ${SRC_FILES}
                  ./src/UnitreeL1IMU/UnitreeL1IMU.cpp    
    )
endif()


if(NOT DISABLE_RealSenseIMUSample)
    set(MODULE_LIBS ${MODULE_LIBS}
                    VisionUtility
                    ${realsense2_LIBRARY}
                    SerialCommunication
    )
    set(INCLUDE_DIRS ${INCLUDE_DIRS}
                    ${realsense2_INCLUDE_DIR}
    )
    set(SRC_FILES ${SRC_FILES}
                  ./src/RealSenseIMU/RealSenseIMU.cpp    
    )
endif()




if(DISABLE_VMU931 AND DISABLE_XsensMT AND DISABLE_IMUCommunicationPoint AND DISABLE_Gable)
    add_library(${MODULE_NAME} INTERFACE)
    target_include_directories(${MODULE_NAME} INTERFACE ${INCLUDE_DIRS})
    target_link_libraries(${MODULE_NAME} INTERFACE)
else()
    add_library(${MODULE_NAME} SHARED ${SRC_FILES})
    target_include_directories(${MODULE_NAME} PUBLIC ${INCLUDE_DIRS})
    target_link_libraries(${MODULE_NAME} ${MODULE_LIBS})

    target_compile_options(${MODULE_NAME} PRIVATE -Wall -Wpedantic -pthread)
endif()

if(ENABLE_UnitTests)
    add_subdirectory(tests)
endif()

# Optional packages for the different samples
find_package(Boost COMPONENTS program_options)

if (NOT DISABLE_VMU931)
    add_executable(VMU931Sample ./samples/VMU931Sample.cpp)
    target_link_libraries(VMU931Sample ${MODULE_NAME})
endif()

if (NOT DISABLE_XsensMT)
    add_executable(XsensMti200 ./samples/XsensMti200.cpp)
    target_link_libraries(XsensMti200 ${MODULE_NAME})
endif()

if(NOT DISABLE_IMUCommunicationPoint)
    add_executable(MockIMUPoint ./samples/MockIMUPoint.cpp)
    target_link_libraries(MockIMUPoint gmock ${Boost_PROGRAM_OPTIONS_LIBRARY} ${MODULE_NAME})
endif()


if(NOT DISABLE_UnitreeL1IMU)
    add_executable(UnitreeL1IMUSample ./samples/UnitreeL1IMUSample.cpp)
    target_link_libraries(UnitreeL1IMUSample  ${Boost_PROGRAM_OPTIONS_LIBRARY} ${MODULE_NAME})
endif()


if(NOT DISABLE_RealSenseIMU)
    add_executable(RealSenseIMUSample ./samples/RealSenseIMUSample.cpp)
    target_link_libraries(RealSenseIMUSample  ${Boost_PROGRAM_OPTIONS_LIBRARY} ${MODULE_NAME})
endif()



if (NOT DISABLE_Gable)
    add_executable(GableSE1Sample ./samples/GableSE1Sample.cpp)
    target_link_libraries(GableSE1Sample ${MODULE_NAME} ${Boost_PROGRAM_OPTIONS_LIBRARY})

    add_executable(GableSE2SE3Sample ./samples/GableSE2SE3Sample.cpp)
    target_link_libraries(GableSE2SE3Sample ${MODULE_NAME} ${Boost_PROGRAM_OPTIONS_LIBRARY})

    if(NOT DISABLE_IMUCommunicationPoint)
        add_executable(GableSE1Point ./samples/GableSE1Point.cpp)
        target_link_libraries(GableSE1Point gmock ${Boost_PROGRAM_OPTIONS_LIBRARY} ${MODULE_NAME})

        add_executable(GableSE2SE3Point ./samples/GableSE2SE3Point.cpp)
        target_link_libraries(GableSE2SE3Point gmock ${Boost_PROGRAM_OPTIONS_LIBRARY} ${MODULE_NAME})
    endif()
endif()
