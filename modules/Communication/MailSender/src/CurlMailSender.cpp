/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license. 
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 * 
 * Author: Giacomo Lunghi CERN EN/SMM/MRO 2019
 * 
 *  ==================================================================================================
 */

#include <curl/curl.h>

#include <ctime>
#include <cstring>
#include <memory>
#include <string>
#include <sstream>

#include "MailSender/CurlMailSender.hpp"

namespace crf {
namespace communication {
namespace mailsender {

CurlMailSender::CurlStringData::CurlStringData(const std::string& data):
    msg(data),
    bytesleft(msg.size()) {
}

CurlMailSender::CurlMailSender():
    smtpServer_("smtp://cernmx.cern.ch"),
    isInitialized_(false),
    logger_("CurlMailSender"),
    curl_() {
        logger_->debug("CTor");
}

bool CurlMailSender::initialize() {
    logger_->debug("initialize()");
    if (isInitialized_) {
        logger_->warn("Already initialized");
        return false;
    }
    curl_ = std::shared_ptr<CURL>(curl_easy_init(), curl_easy_cleanup);
    if (!curl_) {
        logger_->error("Failed initializing curl");
        return false;
    }
    isInitialized_ = true;
    return true;
}

bool CurlMailSender::deinitialize() {
    logger_->debug("deinitialize()");
    if (!isInitialized_) {
        logger_->warn("Not previously initialized");
        return false;
    }
    curl_.reset();
    isInitialized_ = false;
    return true;
}

bool CurlMailSender::send(const Mail& mail) {
    logger_->debug("send()");
    if (!isInitialized_) {
        return false;
    }
    if (mail.getSubject().empty()) {
        return false;
    }
    if (mail.getContent().empty()) {
        return false;
    }
    if (mail.getReceivers().empty()) {
        return false;
    }
    struct curl_slist *recipients = NULL;
    curl_easy_setopt(curl_.get(), CURLOPT_URL, smtpServer_.c_str());
    auto receivers = mail.getReceivers();
    for (unsigned int i=0; i < receivers.size(); i++) {
        recipients = curl_slist_append(recipients, receivers[i].getAddress().c_str());
    }
    curl_easy_setopt(curl_.get(), CURLOPT_MAIL_RCPT, recipients);
    curl_easy_setopt(curl_.get(), CURLOPT_MAIL_FROM, mail.getSender().getAddress().c_str());
    CurlStringData textdata(buildPayloadText(mail));
    curl_easy_setopt(curl_.get(), CURLOPT_READFUNCTION, read_callback);
    curl_easy_setopt(curl_.get(), CURLOPT_READDATA, &textdata);
    curl_easy_setopt(curl_.get(), CURLOPT_UPLOAD, 1L);
    CURLcode res = curl_easy_perform(curl_.get());
    if (res != CURLE_OK) {
        logger_->error("Could not send email: {0}", curl_easy_strerror(res));
        return false;
    }
    curl_slist_free_all(recipients);
    return true;
}

std::string CurlMailSender::buildPayloadText(const Mail& mail) {
    logger_->debug("buildPayloadText()");
    std::stringstream payloadText;
    payloadText << "Date: " << dateTimeNow() << "\r\n";
    payloadText << "To: ";
    auto receivers = mail.getReceivers();
    auto from =  mail.getSender().getAddress();
    for (unsigned int i=0; i < receivers.size(); i++) {
        if (i != 0) {
            payloadText << ", ";
        }
        payloadText << "<" << receivers[i].getAddress() << ">";
    }
    payloadText << "\r\n";
    payloadText << "From: <" << from << "> \r\n";
    payloadText << "Cc: \r\n";
    payloadText << "Message-ID: <" << generateMessageId() << "@" <<
        from.substr(from.find('@')+1) << ">\r\n";
    payloadText << "Subject: " << mail.getSubject() << "\r\n";
    payloadText << "\r\n";
    payloadText << mail.getContent() << "\r\n";
    payloadText << "\r\n";
    payloadText << "\r\n";
    payloadText << "\r\n";  // "Check RFC5322.\r\n";
    return payloadText.str();
}

std::string CurlMailSender::generateMessageId() {
    logger_->debug("generateMessageId()");
    const int MESSAGE_ID_LEN = 37;
    const char alphanum[] = "0123456789" "ABCDEFGHIJKLMNOPQRSTUVWXYZ" "abcdefghijklmnopqrstuvwxyz";
    time_t t;
    struct tm tm;
    std::string ret;
    ret.resize(15);
    time(&t);
    gmtime_r(&t, &tm);
    strftime(const_cast<char *>(ret.c_str()), MESSAGE_ID_LEN, "%Y%m%d%H%M%S.", &tm);
    ret.reserve(MESSAGE_ID_LEN);
    while (ret.size() < MESSAGE_ID_LEN) {
        ret += alphanum[rand() % (sizeof(alphanum) - 1)];
    }
    return ret;
}

std::string CurlMailSender::dateTimeNow() {
    logger_->debug("dateTimeNow()");
    const int RFC5322_TIME_LEN = 32;
    time_t t;
    struct tm *tm;
    std::string ret;
    ret.resize(RFC5322_TIME_LEN);
    time(&t);
    tm = localtime(&t);  // NOLINT
    strftime(&ret[0], RFC5322_TIME_LEN, "%a, %d %b %Y %H:%M:%S %z", tm);
    return ret;
}

size_t CurlMailSender::read_callback(char *buffer, size_t size,
    size_t nitems, void *userdata) {
    CurlStringData* data = reinterpret_cast<CurlStringData*>(userdata);
    size_t requestedNumBytes2Transfer = size*nitems;
    size_t numBytes2Transfer = (data->bytesleft >= requestedNumBytes2Transfer) ?
        requestedNumBytes2Transfer : data->bytesleft;
    std::memcpy(buffer, data->msg.c_str() + (data->msg.size() - data->bytesleft),
        numBytes2Transfer);
    data->bytesleft -= numBytes2Transfer;
    return numBytes2Transfer;
}

}  // namespace mailsender
}  // namespace communication
}  // namespace crf
