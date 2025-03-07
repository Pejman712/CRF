/* © Copyright CERN 2020. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Jorge Playán Garai CERN BE/CEM/MRO
 *
 *  ==================================================================================================
 */

#include <memory>
#include <string>
#include <sstream>
#include <vector>

#include "MissionManager/MissionManagerCommunicationPoint/MissionManagerCommunicationPoint.hpp"

namespace crf::applications::missionmanager {

MissionManagerCommunicationPoint::MissionManagerCommunicationPoint(
    std::shared_ptr<communication::datapacketsocket::PacketSocket> socket,
    std::shared_ptr<crf::applications::missionmanager::IMissionManager> mission):
    StatusStreamerCommunicationPoint(socket, mission),
    socket_(socket),
    mission_(mission),
    logger_("MissionManagerCommunicationPoint") {
    logger_->debug("CTor");
    jsonCommandHandlers_.insert({
        "start",
        std::bind(&MissionManagerCommunicationPoint::startHandler,
            this, std::placeholders::_1)});
    jsonCommandHandlers_.insert({
        "next",
        std::bind(&MissionManagerCommunicationPoint::nextHandler,
            this, std::placeholders::_1)});
    jsonCommandHandlers_.insert({
        "stop",
        std::bind(&MissionManagerCommunicationPoint::stopHandler,
            this, std::placeholders::_1)});
    jsonCommandHandlers_.insert({
        "pause",
        std::bind(&MissionManagerCommunicationPoint::pauseHandler,
            this, std::placeholders::_1)});
    jsonCommandHandlers_.insert({
        "resume",
        std::bind(&MissionManagerCommunicationPoint::resumeHandler,
            this, std::placeholders::_1)});
    jsonCommandHandlers_.insert({
        "goHome",
        std::bind(&MissionManagerCommunicationPoint::goHomeHandler,
            this, std::placeholders::_1)});
    jsonCommandHandlers_.insert({
        "recharge",
        std::bind(&MissionManagerCommunicationPoint::rechargeHandler,
            this, std::placeholders::_1)});
    jsonCommandHandlers_.insert({
        "setStatus",
        std::bind(&MissionManagerCommunicationPoint::setStatusHandler,
            this, std::placeholders::_1)});
    jsonCommandHandlers_.insert({
        "emergency",
        std::bind(&MissionManagerCommunicationPoint::emergencyHandler,
            this, std::placeholders::_1)});
    jsonCommandHandlers_.insert({
        "rearm",
        std::bind(&MissionManagerCommunicationPoint::rearmHandler,
            this, std::placeholders::_1)});
}

MissionManagerCommunicationPoint::~MissionManagerCommunicationPoint() {
}

void MissionManagerCommunicationPoint::startHandler(
    const communication::datapackets::JSONPacket& packet) {
    logger_->debug("startHandler");
    sendJSONReply<bool>(packet, mission_->start());
}

void MissionManagerCommunicationPoint::nextHandler(
    const communication::datapackets::JSONPacket& packet) {
    logger_->debug("nextHandler");
    sendJSONReply<bool>(packet, mission_->next());
}

void MissionManagerCommunicationPoint::stopHandler(
    const communication::datapackets::JSONPacket& packet) {
    logger_->debug("stopHandler");
    sendJSONReply<bool>(packet, mission_->stop());
}

void MissionManagerCommunicationPoint::pauseHandler(
    const communication::datapackets::JSONPacket& packet) {
    logger_->debug("pauseHandler");
    sendJSONReply<bool>(packet, mission_->pause());
}

void MissionManagerCommunicationPoint::resumeHandler(
    const communication::datapackets::JSONPacket& packet) {
    logger_->debug("resumeHandler");
    sendJSONReply<bool>(packet, mission_->resume());
}

void MissionManagerCommunicationPoint::goHomeHandler(
    const communication::datapackets::JSONPacket& packet) {
    logger_->debug("goHomeHandler");
    sendJSONReply<bool>(packet, mission_->goHome());
}

void MissionManagerCommunicationPoint::rechargeHandler(
    const communication::datapackets::JSONPacket& packet) {
    logger_->debug("rechargeHandler");
    sendJSONReply<bool>(packet, mission_->recharge());
}

void MissionManagerCommunicationPoint::setStatusHandler(
    const communication::datapackets::JSONPacket& packet) {
    logger_->debug("setStatusHandler");
    sendJSONReply<bool>(packet, mission_->setStatus(packet.data["message"]));
}

void MissionManagerCommunicationPoint::emergencyHandler(
    const communication::datapackets::JSONPacket& packet) {
    logger_->debug("emergencyHandler");
    sendJSONReply<bool>(packet, mission_->emergency());
}

void MissionManagerCommunicationPoint::rearmHandler(
    const communication::datapackets::JSONPacket& packet) {
    logger_->debug("rearmHandler");
    sendJSONReply<bool>(packet, mission_->rearm());
}

}  // namespace crf::applications::missionmanager
