#######################################################################################################################
##                                                                                                                   ##
## © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software license. ##
## Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch ##
##                                                                                                                   ##
## Author: Alejandro Diaz Rosales CERN BE/CEM/MRO 2021                                                               ##
##                                                                                                                   ##
#######################################################################################################################

if(NOT WIN32)
  string(ASCII 27 Esc)
  set(COLOR_RESET  "${Esc}[m")
  set(COLOR_BOLD   "${Esc}[1m")
  set(RED          "${Esc}[31m")
  set(GREEN        "${Esc}[32m")
  set(YELLOW       "${Esc}[33m")
  set(BLUE         "${Esc}[34m")
  set(MAGENTA      "${Esc}[35m")
  set(CYAN         "${Esc}[36m")
  set(WHITE        "${Esc}[37m")
  set(BOLD_RED     "${Esc}[1;31m")
  set(BOLD_GREEN   "${Esc}[1;32m")
  set(BOLD_YELLOW  "${Esc}[1;33m")
  set(BOLD_BLUE    "${Esc}[1;34m")
  set(BOLD_MAGENTA "${Esc}[1;35m")
  set(BOLD_CYAN    "${Esc}[1;36m")
  set(BOLD_WHITE   "${Esc}[1;37m")
endif()
