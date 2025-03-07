#######################################################################################################################
##                                                                                                                   ##
## © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software licence. ##
## Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch ##
##                                                                                                                   ##
## Author: Alejandro Diaz Rosales CERN BE/CEM/MRO 2021                                                               ##
##                                                                                                                   ##
#######################################################################################################################

check_module_status(REQUESTER
                        CodeReader
                    CHECK
                        EventLogger
                        ErrorHandler
)


check_library_status(REQUESTER
                        CodeReader
                    CHECK
                        CURLPP
)

# Detectors
crf_implementation(IMPLEMENTATION QRCodeCVDetector               OF CodeReader        IS  ON)
if(OpenCV_VERSION VERSION_LESS "4.7.0")
    crf_implementation(IMPLEMENTATION ArUcoCVDetector            OF CodeReader        IS  OFF)
else()
    crf_implementation(IMPLEMENTATION ArUcoCVDetector            OF CodeReader        IS  ON)
endif()

# Decoders
crf_implementation(IMPLEMENTATION QRCodeCVDecoder                OF CodeReader        IS  ON)
crf_implementation(IMPLEMENTATION QRCodeZBarDecoder              OF CodeReader        IS  ON)

# Readers
crf_implementation(IMPLEMENTATION CodeBackgroundReader           OF CodeReader        IS  ON)

mark_module_or_library_as_checked(CodeReader)
