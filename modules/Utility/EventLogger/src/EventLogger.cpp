/* © Copyright CERN 2020. All rights reserved. This software is released under a CERN proprietary
 * software license. Any permission to use it shall be granted in writing. Requests shall be
 * addressed to CERN through mail-KT@cern.ch
 *
 * Author: Pawel Ptasznik CERN EN/SMM/MRO 2017
 * Contributors: Alejandro Diaz Rosales CERN EN/SMM/MRO 2020
 *               Carlos Prados Sesmero CERN EN/SMM/MRO 2020
 *               Adrien Luthi CERN EN/SMM/MRO 2023
 *
 *  ==================================================================================================
 */

#include "EventLogger/EventLogger.hpp"

#include <spdlog/fmt/ostr.h>
#include <spdlog/sinks/rotating_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>

#include <cstdlib>
#include <memory>
#include <string>
#include <thread>

namespace crf::utility::logger {

EventLogger::EventLogger(const std::string &name) {
    logger_ = spdlog::get(name);
    if (logger_) return;
    LoggerParameterInitializer();
    if (std::getenv(LOGGER_PRINT_STDOUT)) {
        logger_ = spdlog::stdout_color_mt(name);
    } else {
        logger_ = std::make_shared<spdlog::logger>(name, fileSink_);
        spdlog::initialize_logger(logger_);
    }
    spdlog::flush_every(std::chrono::seconds(1));
}

std::shared_ptr<spdlog::logger> EventLogger::operator->() const {
    return logger_;
}

bool EventLogger::setFormat(Format format) {
    if (format == currentFormat_) {
        return true;
    }
    if (format == Format::UserDetails) {
        auto format = std::make_unique<spdlog::pattern_formatter>();
        format->add_flag<EventLogger::UserNameFormatter>('*');
        format->add_flag<EventLogger::UserRoleFormatter>('#');
        format->set_pattern(
            "%^[%d-%m-%Y %H:%M:%S.%e] [PID:%P] [Thread:%t] [%l] [%n] [User:%*] [Role:%#] : %v%$");
        spdlog::set_formatter(std::move(format));
        currentFormat_ = Format::UserDetails;
        return true;
    } else if (format == Format::Default) {
        userName = "UserName";
        userRole = "UserRole";
        spdlog::set_pattern("%^[%Y-%m-%d %H:%M:%S.%e] [PID:%P] [Thread:%t] [%l] [%n] : %v%$");
        currentFormat_ = Format::Default;
        return true;
    }
    return false;
}

void EventLogger::LoggerParameterInitializer() {
    if (!std::getenv(LOGGER_ENABLE_LOGGING)) {
        spdlog::set_level(spdlog::level::off);
        return;
    }
    if (std::getenv(LOGGER_ENABLE_DEBUG)) {
        spdlog::set_level(spdlog::level::debug);
    } else {
        spdlog::set_level(spdlog::level::info);
    }
    char *format = getenv(LOGGER_SET_FORMAT);
    if (!format) {
        setFormat(Format::Default);
        return;
    }
    if (Format(std::stoi(format)) == Format::UserDetails) {
        setFormat(Format::UserDetails);
        return;
    }
    setFormat(Format::Default);
    return;
}

void EventLogger::UserNameFormatter::format(const spdlog::details::log_msg &, const std::tm &,
    spdlog::memory_buf_t &dest) {
    dest.append(userName.data(), userName.data() + userName.size());
}

std::unique_ptr<spdlog::custom_flag_formatter> EventLogger::UserNameFormatter::clone() const {
    return spdlog::details::make_unique<UserNameFormatter>();
}

void EventLogger::UserRoleFormatter::format(const spdlog::details::log_msg &, const std::tm &,
    spdlog::memory_buf_t &dest) {
    dest.append(userRole.data(), userRole.data() + userRole.size());
}

std::unique_ptr<spdlog::custom_flag_formatter> EventLogger::UserRoleFormatter::clone() const {
    return spdlog::details::make_unique<UserRoleFormatter>();
}

const std::shared_ptr<spdlog::sinks::rotating_file_sink_mt> EventLogger::fileSink_ =
    std::make_shared<spdlog::sinks::rotating_file_sink_mt>(LOGGER_FILENAME, LOGGER_MAX_FILE_SIZE,
                                                           LOGGER_MAX_NUMBER_OF_FILES);

}  // namespace crf::utility::logger
