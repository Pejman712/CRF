#######################################################################################################################
##                                                                                                                   ##
## © Copyright CERN 2020. All rights reserved. This software is released under a CERN proprietary software license.  ##
## Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch ##
##                                                                                                                   ##
## Author: Alejandro Diaz Rosales CERN EN/SMM/MRO                                                                    ##
##                                                                                                                   ##
#######################################################################################################################

set(MODULE_NAME "MailSender")

if(DISABLE_${MODULE_NAME})
    return()
endif()

set(MODULE_LIBS EventLogger
                CommonInterfaces
)
set(INCLUDE_DIRS ./include
)
set(SRC_FILES ./src/Mail.cpp
              ./src/MailAddress.cpp
)

if (NOT DISABLE_CurlMailSender)
    set(MODULE_LIBS ${MODULE_LIBS}
                    curl
    )
    set(INCLUDE_DIRS ${INCLUDE_DIRS}
    )
    set(SRC_FILES ${SRC_FILES}
                  ./src/CurlMailSender.cpp
    )
endif()

if(DISABLE_CurlMailSender)
    add_library(${MODULE_NAME} INTERFACE)
    target_include_directories(${MODULE_NAME} INTERFACE ${INCLUDE_DIRS})
    target_link_libraries(${MODULE_NAME} INTERFACE)
else()
    add_library(${MODULE_NAME} SHARED ${SRC_FILES})
    target_include_directories(${MODULE_NAME} PUBLIC ${INCLUDE_DIRS})
    target_link_libraries(${MODULE_NAME} ${MODULE_LIBS})

    target_compile_options(${MODULE_NAME} PRIVATE -Wall -Wpedantic)
endif()

if(ENABLE_UnitTests)
    add_subdirectory(tests)
endif()

if (NOT DISABLE_CurlMailSender)
    add_executable(SendMailSample ./samples/SendMailSample.cpp)
    target_link_libraries(SendMailSample ${MODULE_NAME})
endif()
