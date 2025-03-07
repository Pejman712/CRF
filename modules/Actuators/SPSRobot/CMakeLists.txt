set (MODULE_NAME "SPSRobot")

set(MODULE_LIBS EventLogger
                EtherCATDevices
                EtherCATDrivers
                RobotBase
)
set(INCLUDE_DIRS ./include
)
set(SRC_FILES ./src/SPSRobot.cpp
)

add_library(${MODULE_NAME} SHARED ${SRC_FILES})
target_include_directories(${MODULE_NAME} PUBLIC ${INCLUDE_DIRS})
target_link_libraries(${MODULE_NAME} ${MODULE_LIBS})



find_package(Boost 1.56 QUIET COMPONENTS program_options)
if (Boost_FOUND)
    add_executable(SPSRobotCommunicationPoint ./samples/SPSRobotCommunicationPoint.cpp)
    target_link_libraries(SPSRobotCommunicationPoint ${MODULE_NAME} ${Boost_LIBRARIES} RobotBaseCommunicationPoint)

    add_executable(SPSRobotBasePointwithLocalization ./samples/SPSRobotBasePointwithLocalization.cpp)
    target_link_libraries(SPSRobotBasePointwithLocalization ${MODULE_NAME}
    	                                                    ${Boost_LIBRARIES}
    	                                                    RobotBaseCommunicationPoint
    	                                                    RobotPoseEstimators)
endif()


add_executable(SPSRobotSample ./samples/SPSRobotSample.cpp)
target_link_libraries(SPSRobotSample ${MODULE_NAME} ${Boost_LIBRARIES})