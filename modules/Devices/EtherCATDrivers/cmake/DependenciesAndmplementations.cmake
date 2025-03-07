#######################################################################################################################
##                                                                                                                   ##
## © Copyright CERN 2022. All rights reserved. This software is released under a CERN proprietary software licence.  ##
## Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch ##
##                                                                                                                   ##
## Author: Natalija Topalovic CERN BE/CEM/MRO 2022                                                                   ##
##                                                                                                                   ##
#######################################################################################################################

check_module_status(REQUESTER
                        EtherCATDrivers
                    CHECK
                        EventLogger
                        CommonInterfaces
                        ErrorHandler
                        SOEMAPI
)

check_library_status(REQUESTER
                        EtherCATDrivers
                    CHECK
                        nlohmann_json
)

crf_implementation(IMPLEMENTATION EtherCATMaster                                 OF EtherCATDrivers        IS  ON)
crf_implementation(IMPLEMENTATION BasicEtherCATDriver                            OF EtherCATDrivers        IS  ON)

mark_module_or_library_as_checked(EtherCATDrivers)
