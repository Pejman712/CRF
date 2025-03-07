#######################################################################################################################
##                                                                                                                   ##
## © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software license.  ##
## Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch ##
##                                                                                                                   ##
## Author: Alejandro Diaz Rosales CERN EN/SMM/MRO 2021                                                               ##
##                                                                                                                   ##
#######################################################################################################################

set(MODULE_NAME "CommunicationUtility")

if(DISABLE_${MODULE_NAME})
    return()
endif()

set(MODULE_LIBS ErrorHandler
)
set(INCLUDE_DIRS ./include
)
set(SRC_FILES ./src/SocketInterface.cpp
              ./src/StdJsonConverters.cpp
)

if (NOT DISABLE_StreamIO)
    set(MODULE_LIBS ${MODULE_LIBS}
                    ${OpenCV_LIBRARIES}
                    ${orocos_kdl_LIBRARIES}
    )
    set(INCLUDE_DIRS ${INCLUDE_DIRS}
                     ${OpenCV_INCLUDE_DIRS}
                     ${EIGEN3_INDCLUDE_DIRS}
    )
    set(SRC_FILES ${SRC_FILES}
                  ./src/StreamReader.cpp
                  ./src/StreamWriter.cpp
    )
endif()

add_library(${MODULE_NAME} SHARED ${SRC_FILES})
target_include_directories(${MODULE_NAME} PUBLIC ${INCLUDE_DIRS})
target_link_libraries(${MODULE_NAME} ${MODULE_LIBS})

target_compile_options(${MODULE_NAME} PRIVATE -Wall -Wpedantic -pthread)

if(ENABLE_UnitTests)
    add_subdirectory(tests)
endif()
