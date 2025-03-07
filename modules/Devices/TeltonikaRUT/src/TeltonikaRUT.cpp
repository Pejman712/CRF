/* © Copyright CERN 2018. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Giacomo Lunghi CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#include <vector>
#include <sstream>
#include <sys/time.h>

#include <TeltonikaRUT/TeltonikaRUT.hpp>

namespace crf {
namespace devices {

TeltonikaRUT::TeltonikaRUT(std::string ip_address, int port, std::string username,
    std::string password, std::shared_ptr<IPC> outputIPC) :
    ip_address_(ip_address),
    port_(port),
    username_(username),
    password_(password),
    outputIPC(outputIPC) {
    _stop = false;
    this->outputIPC = outputIPC;
}

size_t WriteCallback(char *contents, size_t size, size_t nmemb, void *userp) {
    ((std::string*)userp)->append(reinterpret_cast<char*>(contents), size * nmemb);
    return size * nmemb;
}


bool TeltonikaRUT::initialize() {
    curl_global_init(CURL_GLOBAL_ALL);
    atexit(curl_global_cleanup);

    curl = curl_easy_init();

    headers = NULL;  // Initialization to NULL is fundamental
    headers = curl_slist_append(headers, "Expect:");
    headers = curl_slist_append(headers, "Content-Type: application/json");
    curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);

    json params = json::array({
        "00000000000000000000000000000000",
        "session",
        "login",
        {
            {"username", username_ },
            {"password", password_}
        }
    });

    json j = {
        {"jsonrpc", "2.0"},
        {"id", 1},
        {"method", "call"},
        {"params",
            params
        }
    };

    std::string json_string = j.dump();
    std::string result;

    curl_easy_setopt(curl, CURLOPT_POSTFIELDS, json_string.c_str());
    curl_easy_setopt(curl, CURLOPT_POSTFIELDSIZE, -1L);

    curl_easy_setopt(curl, CURLOPT_URL, ip_address_.append("/ubus").c_str());

    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteCallback);
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, result);

    int res = curl_easy_perform(curl);
    if (res != CURLE_OK) {
        return false;
    } else {
        json jresult;
        try {
            jresult =   json::parse(result);
        } catch (std::exception &e) {
            return false;
        }

        if (jresult["result"][1].value("ubus_rpc_session", "") != "") {
            session_id_ = jresult["result"][1]["ubus_rpc_session"];
            printf("Session ID %s\n", session_id_.c_str());
        } else {
            return false;
        }
    }

    updateDataThread = std::thread(&TeltonikaRUT::updateSignalInfo, this);
    return true;
}

bool TeltonikaRUT::deinitialize() {
    curl_slist_free_all(headers);
    curl_easy_cleanup(curl);

    _stop = true;
    updateDataThread.join();
    return true;
}

void TeltonikaRUT::updateSignalInfo() {
    while (!_stop) {
        json params = json::array({
            session_id_,
            "file",
            "exec",
            {
                {"command", "gsmctl"},
                {"params", {
                    "--cellid",
                    "--sinr",
                    "-M",
                    "--rsrp",
                    "-E",       // WCDMA ec/io
                    "-X",       // WCDMA rscp
                    "-q",       // gsm signal
                    "-t",       //
                    "-r",
                    "wwan0",
                    "-e",
                    "wwan0",
                    "-n"
                }}
            }
        });

        json request = {
            {"jsonrpc", "2.0"},
            {"id", 1},
            {"method", "call"},
            {"params", params}
        };

        std::string request_string = request.dump();

        std::string result;

        curl_easy_setopt(curl, CURLOPT_POSTFIELDS, request_string.c_str());
        curl_easy_setopt(curl, CURLOPT_POSTFIELDSIZE, -1L);

        curl_easy_setopt(curl, CURLOPT_URL, ip_address_.append("/ubus").c_str());

        curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteCallback);
        curl_easy_setopt(curl, CURLOPT_WRITEDATA, result);

        int res = curl_easy_perform(curl);
        if (res != CURLE_OK) {
            continue;
        }

        json jresult =  json::parse(result);

        if (jresult["result"][1].value("code", -1) != 0) {
            continue;
        }

        std::string buffer = jresult["result"][1].value("stdout", "");

        if (buffer == "")
            continue;

        std::istringstream ss(buffer);
        std::getline(ss, statusPacket.cellName, '\n');

        std::string support_string;

        std::getline(ss, support_string, '\n');
        try {
            statusPacket.LTEsinr = std::stof(support_string);
        } catch (std::exception &e) {
            statusPacket.LTEsinr = 0;
        }

        std::getline(ss, support_string, '\n');
        try {
            statusPacket.LTErsrq = std::stof(support_string);
        } catch (std::exception &e) {
            statusPacket.LTErsrq = 0;
        }

        std::getline(ss, support_string, '\n');
        try {
            statusPacket.LTErsrp = std::stof(support_string);
        } catch (std::exception &e) {
            statusPacket.LTErsrp = 0;
        }

        std::getline(ss, support_string, '\n');
        try {
            statusPacket.WCDMAecic = std::stof(support_string);
        } catch (std::exception &e) {
            statusPacket.WCDMAecic = 0;
        }

        std::getline(ss, support_string, '\n');
        try {
            statusPacket.WCDMArscp = std::stof(support_string);
        } catch (std::exception &e) {
            statusPacket.WCDMArscp = 0;
        }

        std::getline(ss, support_string, '\n');
        try {
            statusPacket.GSMSignalStrength = std::stof(support_string);
        } catch (std::exception &e) {
            statusPacket.GSMSignalStrength = 0;
        }

        std::getline(ss, statusPacket.connectionType, '\n');

        std::getline(ss, support_string, '\n');
        try {
            statusPacket.GSMBytesReceived = std::stof(support_string);
        } catch (std::exception &e) {
            statusPacket.GSMBytesReceived = 0;
        }

        std::getline(ss, support_string, '\n');
        try {
            statusPacket.GSMBytesSent = std::stof(support_string);
        } catch (std::exception &e) {
            statusPacket.GSMBytesSent = 0;
        }


        outputIPC->write(statusPacket.serialize(), statusPacket.getHeader());
        usleep(100000);
    }
}

void TeltonikaRUT::joinThreads() {
    updateDataThread.join();
}

}  // namespace devices
}  // namespace crf
