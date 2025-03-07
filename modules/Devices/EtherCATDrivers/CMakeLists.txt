#######################################################################################################################
##                                                                                                                   ##
## © Copyright CERN 2022. All rights reserved. This software is released under a CERN proprietary software license.  ##
## Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch ##
##                                                                                                                   ##
## Author: Natalija Topalovic CERN BE/CEM/MRO 2022                                                                   ##
##                                                                                                                   ##
#######################################################################################################################

set(MODULE_NAME "EtherCATDrivers")

if(DISABLE_${MODULE_NAME})
    return()
endif()

set(MODULE_LIBS EventLogger
                CommonInterfaces
                ErrorHandler
                SOEMAPI
)

set(INCLUDE_DIRS ./include
)

if (NOT DISABLE_EtherCATMaster)
    set(SRC_FILES ${SRC_FILES}
                  ./src/EtherCATMaster/EtherCATMaster.cpp
    )
endif()

if (NOT DISABLE_BasicEtherCATDriver)
    set(SRC_FILES ${SRC_FILES}
                  ./src/BasicEtherCATDriver/BasicEtherCATDriver.cpp
    )
endif()

if (DISABLE_EtherCATMaster AND DISABLE_BasicEtherCATDriver)
    add_library(${MODULE_NAME} INTERFACE)
    target_include_directories(${MODULE_NAME} INTERFACE ${INCLUDE_DIRS})
    target_link_libraries(${MODULE_NAME} INTERFACE ${MODULE_LIBS})
else()
    add_library(${MODULE_NAME} SHARED ${SRC_FILES})
    target_include_directories(${MODULE_NAME} PUBLIC ${INCLUDE_DIRS})
    target_link_libraries(${MODULE_NAME} ${MODULE_LIBS})

    target_compile_options(${MODULE_NAME} PRIVATE -Wall -Wpedantic -pthread)
endif()

if(ENABLE_UnitTests)
    add_subdirectory(tests)
endif()

if (NOT DISABLE_EtherCATMaster AND NOT DISABLE_BasicEtherCATDriver)
    add_executable(EtherCATExample ./samples/EtherCATExample.cpp)
    target_link_libraries(EtherCATExample ${MODULE_NAME})
endif()
