#######################################################################################################################
##                                                                                                                   ##
## © Copyright CERN 2020.  All rights reserved. This software is released under a CERN proprietary software license. ##
## Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch ##
##                                                                                                                   ##
## Author: Jorge Playán Garai CERN EN/SMM/MRO                                                                        ##
##                                                                                                                   ##
#######################################################################################################################

set(MODULE_NAME "SensorValidationResults")

if(DISABLE_${MODULE_NAME})
    return()
endif()

set(MODULE_LIBS EventLogger
                BLMTriggeringAnalysis
)
set(INCLUDE_DIRS ./include
)
set(SRC_FILES
)

if (NOT DISABLE_BLMValidationJSONResults)
    set(MODULE_LIBS ${MODULE_LIBS}
                    MailSender
                    ${nlohmann_json_LIBRARIES}
    )
    set(INCLUDE_DIRS ${INCLUDE_DIRS}
                     ${nlohmann_json_INCLUDE_DIRS}
    )
    set(SRC_FILES ${SRC_FILES}
                  ./src/BLMValidationJSONResults/BLMValidationJSONResults.cpp
    )
endif()

if (NOT DISABLE_PMIValidationJSONResults)
    set(MODULE_LIBS ${MODULE_LIBS}
                    MailSender
                    ${nlohmann_json_LIBRARIES}
    )
    set(INCLUDE_DIRS ${INCLUDE_DIRS}
                     ${nlohmann_json_INCLUDE_DIRS}
    )
    set(SRC_FILES ${SRC_FILES}
                  ./src/PMIValidationJSONResults/PMIValidationJSONResults.cpp
    )
endif()

if(DISABLE_BLMValidationJSONResults AND DISABLE_PMIValidationJSONResults)
    add_library(${MODULE_NAME} INTERFACE)
    target_include_directories(${MODULE_NAME} INTERFACE ${INCLUDE_DIRS})
    target_link_libraries(${MODULE_NAME} INTERFACE)
else()
    add_library(${MODULE_NAME} SHARED ${SRC_FILES})
    target_include_directories(${MODULE_NAME} PUBLIC ${INCLUDE_DIRS})
    target_link_libraries(${MODULE_NAME} ${MODULE_LIBS})

    target_compile_options(${MODULE_NAME} PRIVATE -Wall -Wpedantic)
endif()

if (NOT DISABLE_BLMValidationJSONResults)
    add_executable(BLMValidationJSONResultsSample ./samples/BLMValidationJSONResultsSample.cpp)
    target_link_libraries(BLMValidationJSONResultsSample ${MODULE_NAME})
endif()

if (NOT DISABLE_PMIValidationJSONResults)
    add_executable(PMIValidationJSONResultsSample ./samples/PMIValidationJSONResultsSample.cpp)
    target_link_libraries(PMIValidationJSONResultsSample ${MODULE_NAME})
endif()
