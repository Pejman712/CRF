macro(XsensApi_REPORT_NOT_FOUND REASON_MSG)
    unset(XsensApi_FOUND)
    unset(XsensApi_INCLUDE_DIRS)
    unset(XsensApi_LIBRARIES)
    # Note <package>_FIND_[REQUIRED/QUIETLY] variables defined by FindPackage()
    # use the camelcase library name, not uppercase.
    if(XsensApi_FIND_QUIETLY)
        message(STATUS "Failed to find XsensApi - " ${REASON_MSG} ${ARGN})
    elseif(XsensApi_FIND_REQUIRED)
        message(FATAL_ERROR "Failed to find XsensApi - " ${REASON_MSG} ${ARGN})
    else()
        # Neither QUIETLY nor REQUIRED, use no priority which emits a message
        # but continues configuration and allows generation.
        message("-- Failed to find XsensApi - " ${REASON_MSG} ${ARGN})
    endif()
endmacro(XsensApi_REPORT_NOT_FOUND)

set(XsensApi_FOUND TRUE)

if(NOT EXISTS /usr/local/xsens/include)
    XsensApi_REPORT_NOT_FOUND("Directory /usr/local/xsens/include does not exist")
endif()

if(NOT EXISTS /usr/local/xsens/lib64/libxsensdeviceapi.so)
    XsensApi_REPORT_NOT_FOUND("Cannot find /usr/local/xsens/lib64/libxsensdeviceapi.so")
endif()

if(NOT EXISTS /usr/local/xsens/lib64/libxstypes.so)
    XsensApi_REPORT_NOT_FOUND("Cannot find /usr/local/xsens/lib64/libxstypes.so")
endif()

include(FindPackageHandleStandardArgs)
if (XsensApi_FOUND)
    set(XsensApi_INCLUDE_DIRS /usr/local/xsens/include)
    set(XsensApi_LIBRARIES
        /usr/local/xsens/lib64/libxsensdeviceapi.so
        /usr/local/xsens/lib64/libxstypes.so
    )
    FIND_PACKAGE_HANDLE_STANDARD_ARGS(XsensApi DEFAULT_MSG
        XsensApi_INCLUDE_DIRS XsensApi_LIBRARIES)
endif()
