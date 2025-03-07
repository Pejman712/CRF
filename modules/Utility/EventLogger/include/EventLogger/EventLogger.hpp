/* © Copyright CERN 2020. All rights reserved. This software is released under a CERN proprietary
 * software license. Any permission to use it shall be granted in writing. Requests shall be
 * addressed to CERN through mail-KT@cern.ch
 *
 * Author: Pawel Ptasznik CERN EN/SMM/MRO 2017
 * Contributors: Alejandro Diaz Rosales CERN EN/SMM/MRO 2020
 *               Carlos Prados Sesmero CERN EN/SMM/MRO 2020
 *               Adrien Luthi CERN EN/SMM/MRO 2023
 *
 *  ===============================================================================================
 */

#pragma once

#include <spdlog/fmt/ostr.h>
#include <spdlog/sinks/rotating_file_sink.h>
#include <spdlog/spdlog.h>
#include "spdlog/pattern_formatter.h"

#include <iostream>
#include <memory>
#include <string>

#define LOGGER_ENABLE_LOGGING "LOGGER_ENABLE_LOGGING"  // Set the env. variable to enable the logs.
#define LOGGER_PRINT_STDOUT "LOGGER_PRINT_STDOUT"  // Set the env. variable to print on STDOUT.
#define LOGGER_ENABLE_DEBUG "LOGGER_ENABLE_DEBUG"  // Set the env. variable to enable debug logs.
#define LOGGER_SET_FORMAT "LOGGER_SET_FORMAT"
#define LOGGER_FILENAME "RoboticFrameworkLog.log"
#define LOGGER_MAX_FILE_SIZE 1024 * 1024 * 64  // Using 64MB log files
#define LOGGER_MAX_NUMBER_OF_FILES 10  // Let's say that 10 files (64MB each) is enough

namespace crf::utility::logger {

enum class Format {
    Undefined = 0,
    Default = 1,
    UserDetails = 2,
};

/*
 * @brief Wrapper class of the spdlog library, a very fast C++ logging library. For instructions
 *        please see LoggerTests.cpp file in the Tests folders.
 */
class EventLogger {
 public:
    EventLogger() = delete;
    EventLogger(const EventLogger &l) = default;
    explicit EventLogger(const std::string &name);
    /*
     * @brief operator that prints a colored message into the screen with 5 different levels: info,
     *        debug, warning, error, critical.
     */
    std::shared_ptr<spdlog::logger> operator->() const;
    /*
     * @brief set the format of the logger.
     */
    static bool setFormat(Format format);
    /*
     * @brief Variable that store the userName to display on the logger line.
     */
    inline static std::string userName = "UserName";
    /*
     * @brief Variable that store the userRole to display on the logger line.
     */
    inline static std::string userRole = "UserRole";

 private:
    static const std::shared_ptr<spdlog::sinks::rotating_file_sink_mt> fileSink_;
    std::shared_ptr<spdlog::logger> logger_;
    inline static Format currentFormat_ = Format::Undefined;

    void LoggerParameterInitializer();
    /*
     * @brief Class which inherits from spdlog::custom_flag_formatter that allow to add the
     *              userName variable in logger line.
     */
    class UserNameFormatter : public spdlog::custom_flag_formatter {
     public:
        void format(const spdlog::details::log_msg &, const std::tm &,
                    spdlog::memory_buf_t &dest) override;
        std::unique_ptr<spdlog::custom_flag_formatter> clone() const override;
    };
    /*
     * @brief Class which inherits from spdlog::custom_flag_formatter that allow to add the
     *              userRole variable in logger line.
     */
    class UserRoleFormatter : public spdlog::custom_flag_formatter {
     public:
        void format(const spdlog::details::log_msg &, const std::tm &,
                    spdlog::memory_buf_t &dest) override;
        std::unique_ptr<spdlog::custom_flag_formatter> clone() const override;
    };
};

}  // namespace crf::utility::logger
