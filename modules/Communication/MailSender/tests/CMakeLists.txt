#######################################################################################################################
##                                                                                                                   ##
## © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software license.  ##
## Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch ##
##                                                                                                                   ##
## Author: Jorge Playán Garai CERN BE/CEM/MRO 2022                                                                   ##
##                                                                                                                   ##
#######################################################################################################################

set(MAIL_SENDER_TESTS_FILES
    main.cpp
    MailAddressTests.cpp
    MailTests.cpp
)

if (NOT DISABLE_CurlMailSender)
    set(MISSION_MANAGER_TESTS_FILES 
        ${MISSION_MANAGER_TESTS_FILES}
        CurlMailSenderTests.cpp
    )
endif()

add_executable(ut-mailsender ${MAIL_SENDER_TESTS_FILES})
target_link_libraries(ut-mailsender gmock
                                    EventLogger
                                    MailSender
)

add_test(MailSenderUnitTests ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/ut-mailsender)
