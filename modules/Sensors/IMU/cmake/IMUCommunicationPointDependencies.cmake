#######################################################################################################################
##                                                                                                                   ##
## © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software licence.  ##
## Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch ##
##                                                                                                                   ##
## Author: Sebastien Collomb CERN BE/CEM/MRO 2023                                                                    ##
##                                                                                                                   ##
#######################################################################################################################

# Check for the modules dependencies
check_module_status(REQUESTER
                        IMUCommunicationPoint
                    CHECK
                        EventLogger
                        Sockets
                        DataPackets
                        DataPacketSocket
                        DeviceManager
)
