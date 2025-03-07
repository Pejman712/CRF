/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Giacomo Lunghi CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#include <fstream>
#include <map>
#include <memory>
#include <nlohmann/json.hpp>
#include <string>
#include <vector>

#include "Cameras/AxisCamera/AxisCamera.hpp"
#include "WebServers/TIMWebServer.hpp"
#include "TIM/TIMS300.hpp"

namespace crf {
namespace communication {
namespace webservers {

TIMWebServer::TIMWebServer(const std::string& serverConfigFile) :
    logger_("TIMWebServer"),
    initialized_(false),
    serviceStarted_(false),
    closing_(false),
    port_(0),
    context_(),
    server_(),
    settings_(new restbed::Settings),
    tims_() {
        logger_->debug("CTor");
        if (!parseConfigFile(serverConfigFile)) {
            throw std::invalid_argument("The provided configuration file is not valid");
        }

        settings_->set_root(context_);
        settings_->set_port(port_);

        auto availableTimsStatusResource = std::make_shared<restbed::Resource>();
        availableTimsStatusResource->set_path("/all/status");
        availableTimsStatusResource->set_method_handler("GET",
            [this](const std::shared_ptr<restbed::Session> session) {
            nlohmann::json statusAll;
            for (auto it = tims_.begin(); it != tims_.end(); ++it) {
                statusAll[std::to_string(it->first)] =
                    nlohmann::json::parse(it->second->getStatus());
            }
            std::string statusString = statusAll.dump();
            session->close(200, statusString,
                { { "Content-Length", std::to_string(statusString.length())},
                { "Content-Type", "application/json"} });
        });
        server_.publish(availableTimsStatusResource);

        auto availableTimsListResource = std::make_shared<restbed::Resource>();
        availableTimsListResource->set_path("/list");
        availableTimsListResource->set_method_handler("GET",
            [this](const std::shared_ptr<restbed::Session> session) {
            std::vector<int> timIds;
            nlohmann::json list;
            for (auto it = tims_.begin(); it != tims_.end(); ++it) {
                timIds.push_back(it->first);
            }
            list["tims"] = timIds;
            std::string statusString = list.dump();
            session->close(200, statusString,
                { { "Content-Length", std::to_string(statusString.length())},
                { "Content-Type", "application/json"} });
        });
        server_.publish(availableTimsListResource);
}

TIMWebServer::~TIMWebServer() {
    logger_->debug("DTor");
    deinitialize();
}

bool TIMWebServer::initialize() {
    logger_->debug("initialize");
    if (initialized_) {
        logger_->warn("Already initialized");
        return false;
    }

    if (tims_.empty()) {
        logger_->warn("No tim webpoints were added");
        return false;
    }

    for (auto it = tims_.begin(); it != tims_.end(); ++it) {
        it->second->initialize();
    }

    closing_ = false;

    serverReturnValue_ = std::async(std::launch::async, [this](){
        try {
            server_.set_ready_handler(std::bind(&TIMWebServer::serviceReady,
                this, std::placeholders::_1));
            server_.start(settings_);
        } catch (const std::exception& e) {
            logger_->warn("Start exception {}", e.what());
            return closing_ ? true : false;
        }
        return true;
    });

    initialized_ = true;

    std::unique_lock<std::mutex> lock(mutex_);
    if (!condVar_.wait_for(lock,
        std::chrono::seconds(initalizeTimeoutSec_),
        [this]{return serviceStarted_.load();})) {
        logger_->error("Unable to initialize with the given timeout");
        deinitialize();
        return false;
    }

    return true;
}

bool TIMWebServer::deinitialize() {
    logger_->debug("deinitialize");
    if (!initialized_) {
        logger_->warn("Not initialized");
        return false;
    }

    closing_ = true;
    server_.stop();
    for (auto it = tims_.begin(); it != tims_.end(); ++it) {
        it->second->deinitialize();
    }
    initialized_ = false;
    return serverReturnValue_.get();
}

bool TIMWebServer::addTIM(int id, std::shared_ptr<TIMWebPoint> tim) {
    logger_->debug("addTIM");
    if (initialized_) {
        logger_->warn("Can't add tim on initialized server");
        return false;
    }

    if (tims_.find(id) != tims_.end()) {
        logger_->warn("TIM with ID {} already added", id);
        return false;
    }

    tims_.insert({id, tim});

    auto statusResource = std::make_shared<restbed::Resource>();
    statusResource->set_path("/" + std::to_string(id) + "/status/");
    statusResource->set_method_handler("GET",
        [this, tim](const std::shared_ptr<restbed::Session> session) {
        auto statusString = tim->getStatus();
        session->close(200, statusString,
            { { "Content-Length", std::to_string(statusString.length()) },
                { "Content-Type", "application/json"}});
    });
    server_.publish(statusResource);

    auto historyResource = std::make_shared<restbed::Resource>();
    historyResource->set_path("/" + std::to_string(id) + "/history/");
    historyResource->set_method_handler("GET",
        [this, tim](const std::shared_ptr<restbed::Session> session) {
        auto statusString = tim->getDataHistory();
        session->close(200, statusString,
            { { "Content-Length", std::to_string(statusString.length())},
                { "Content-Type", "application/json"}});
    });
    server_.publish(historyResource);

    auto cameras = tim->getCameraNames();
    for (size_t i=0; i < cameras.size(); i++) {
        std::string cameraName = cameras[i];
        auto cameraResource = std::make_shared<restbed::Resource>();
        cameraResource->set_path("/" + std::to_string(id) + "/cameras/"+cameraName);
        cameraResource->set_method_handler("GET",
            [this, tim, cameraName](const std::shared_ptr<restbed::Session> session) {
            auto jpegBytes = tim->getCameraFrame(cameraName);
            session->close(200, jpegBytes,
                { { "Content-Length", std::to_string(jpegBytes.length())},
                    { "Content-Type", "image/jpeg"}});
        });
        server_.publish(cameraResource);
    }

    return true;
}

bool TIMWebServer::parseConfigFile(const std::string& serverConfigFile) {
    logger_->debug("parseConfigFile");
    std::ifstream configData(serverConfigFile);
    if ((configData.rdstate() & std::ifstream::failbit) != 0) {
        return false;
    }

    nlohmann::json configJson;
    try {
        configData >> configJson;

        port_ = configJson.at("port").get<int>();
        initalizeTimeoutSec_ = configJson.at("initalizeTimeoutSec").get<int>();
        context_ = configJson.at("context").get<std::string>();

        auto timsConfig = configJson.at("tims").get<std::vector<nlohmann::json> > ();
        for (size_t i=0; i < timsConfig.size(); i++) {
            std::shared_ptr<robots::tim::ITIM> tim;
            if (timsConfig[i].at("type").get<std::string>() == "S300") {
                tim = std::make_shared<robots::tim::TIMS300>(
                    timsConfig[i].at("configFilename").get<std::string>());

                auto point = std::make_shared<TIMWebPoint>(tim);

                auto camerasConfig = timsConfig[i].at("cameras")
                    .get<std::vector<nlohmann::json> > ();
                for (size_t k=0; k < camerasConfig.size(); k++) {
                    auto camera = std::make_shared<sensors::cameras::AxisCamera>(
                        camerasConfig[k].at("address").get<std::string>());
                    if (!point->addCamera(camerasConfig[k].at("name").get<std::string>(), camera)) {
                        continue;
                    }
                }

                addTIM(timsConfig[i].at("id").get<int>(), point);
            }
        }
    } catch (const std::exception& e) {
        logger_->warn("Failed to parse because: {}", e.what());
        return false;
    }

    return true;
}

void TIMWebServer::serviceReady(restbed::Service&) {
    logger_->debug("serviceReady");
    serviceStarted_ = true;
    condVar_.notify_one();
}

}  // namespace webservers
}  // namespace communication
}  // namespace crf
