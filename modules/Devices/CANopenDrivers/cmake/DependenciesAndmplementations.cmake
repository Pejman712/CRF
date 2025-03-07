#######################################################################################################################
##                                                                                                                   ##
## © Copyright CERN 2022. All rights reserved. This software is released under a CERN proprietary software licence.  ##
## Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch ##
##                                                                                                                   ##
## Author: Natalija Topalovic CERN BE/CEM/MRO 2022                                                                   ##
##                                                                                                                   ##
#######################################################################################################################

check_module_status(REQUESTER
                        CANopenDrivers
                    CHECK
                        EventLogger
                        CommonInterfaces
                        ErrorHandler
)

crf_implementation(IMPLEMENTATION CiA402                                    OF CANopenDrivers        IS  ON)
crf_implementation(IMPLEMENTATION CANopen                                   OF CANopenDrivers        IS  ON)
crf_implementation(IMPLEMENTATION CoE                                       OF CANopenDrivers        IS  ON)
crf_implementation(IMPLEMENTATION CiA402CommunicationPoint                  OF CANopenDrivers        IS  ON)

mark_module_or_library_as_checked(CANopenDrivers)
