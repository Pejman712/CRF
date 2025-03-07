#######################################################################################################################
##                                                                                                                   ##
## © Copyright CERN 2023. All rights reserved. This software is released under a CERN proprietary software license.  ##
## Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch ##
##                                                                                                                   ##
## Author: Alejandro Diaz Rosales CERN BE/CEM/MRO 2023                                                               ##
##                                                                                                                   ##
#######################################################################################################################

set(MODULE_NAME "Tool")

if(DISABLE_${MODULE_NAME})
    return()
endif()

set(INCLUDE_DIRS ./include
                 ${urdfdom_INCLUDE_DIRS}
)
set(MODULE_LIBS CommonInterfaces
                ErrorHandler
                ${urdfdom_LIBRARIES}
)

if(NOT DISABLE_ActiveToolCommunicationPoint)
    set(MODULE_LIBS ${MODULE_LIBS}
                    EventLogger
                    DataPackets
                    DataPacketSocket
                    DeviceManager
    )
    set(INCLUDE_DIRS ${INCLUDE_DIRS}
    )
    set(SRC_FILES ${SRC_FILES}
                  ./src/ActiveToolCommunicationPoint/ActiveToolManager.cpp
                  ./src/ActiveToolCommunicationPoint/ActiveToolCommunicationPoint.cpp
                  ./src/ActiveToolCommunicationPoint/ActiveToolCommunicationPointFactory.cpp
    )
endif()

if (DISABLE_ActiveToolCommunicationPoint)
    add_library(${MODULE_NAME} INTERFACE)
    target_include_directories(${MODULE_NAME} INTERFACE ${INCLUDE_DIRS})
    target_link_libraries(${MODULE_NAME} INTERFACE)
else()
    add_library(${MODULE_NAME} SHARED ${SRC_FILES})
    target_include_directories(${MODULE_NAME} PUBLIC ${INCLUDE_DIRS})
    target_link_libraries(${MODULE_NAME} ${MODULE_LIBS})

    target_compile_options(${MODULE_NAME} PRIVATE -Wall)
endif()

if(ENABLE_UnitTests)
    add_subdirectory(tests)
endif()

# Optional packages for the different samples
find_package(Boost COMPONENTS program_options)

if (NOT DISABLE_Gable AND NOT DISABLE_CoE)
    add_executable(ScrewdriverERobIMUPoint ./samples/ScrewdriverERobIMUPoint.cpp)
    target_link_libraries(ScrewdriverERobIMUPoint ${MODULE_NAME} ${Boost_PROGRAM_OPTIONS_LIBRARY} IMU CANopenDrivers)
endif()
