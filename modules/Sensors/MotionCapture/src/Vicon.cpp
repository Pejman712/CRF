/* © Copyright CERN 2023. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Giancarlo D'Ago CERN BE/CEM/MRO 2023
 *
 * ====================================================================
*/

#include "MotionCapture/Vicon/Vicon.hpp"

namespace crf::sensors::motioncapture {

ViconMotionCapture::ViconMotionCapture(const std::string& hostName, const nlohmann::json& config,
    std::shared_ptr<crf::communication::viconapi::IViconApi> viconApiInterface) :
    client_(viconApiInterface),
    host_(hostName),
    logger_("ViconMotionCapture"),
    streamType_(StreamingType::ServerPush),
    axisMapping_(AxisMappingType::ZUp),
    clientBufferSize_(0),
    timeout_(1000),
    lightweight_(false),
    objectsOnly_(false),
    initialized_(false) {
    logger_->debug("CTor");
    if (!client_) {
        logger_->info("Creating a default Vicon API interface");
        client_ = std::make_shared<crf::communication::viconapi::ViconApi>();
    }
    parse(config);
    connect();
}

ViconMotionCapture::~ViconMotionCapture() {
    logger_->debug("DTor");
    if (initialized_) deinitialize();
    if (client_->isConnected().Connected) {
        client_->disconnect();
        logger_->debug("Disconnecting");
    }
}

bool ViconMotionCapture::parse(const nlohmann::json& motioncaptureJSON) {
    try {
        logger_->debug("Parsing Configuration File");
        clientBufferSize_ =
            motioncaptureJSON["SystemSetup"]["ClientBufferSize"].get<unsigned int>();
        streamType_ = motioncaptureJSON["SystemSetup"]["StreamingType"].get<StreamingType>();
        axisMapping_ = motioncaptureJSON["SystemSetup"]["AxisMapping"].get<AxisMappingType>();
        timeout_ = motioncaptureJSON["SystemSetup"]["ConnectionTimeout"].get<unsigned int>();
        lightweight_ = motioncaptureJSON["DataStreaming"]["Lightweight"].get<bool>();
        objectsOnly_ = motioncaptureJSON["DataStreaming"]["ObjectsOnly"].get<bool>();
        logger_->info("Parsing Successful");
        return true;
    } catch (const std::exception& e) {
        logger_->error("Failed to parse because: {}", e.what());
        return false;
    }
}

crf::Code ViconMotionCapture::evaluate(viconSDK::Result::Enum result) {
    switch (result) {
        case viconSDK::Result::Success:
            logger_->debug("Success");
            return crf::Code::Vicon_Success;
        case viconSDK::Result::Unknown:
            logger_->error("Unknown Error");
            return crf::Code::Vicon_Unknown;
        case viconSDK::Result::NotConnected:
            logger_->error("Not connected to the server");
            return crf::Code::Vicon_NotConnected;
        case viconSDK::Result::ClientAlreadyConnected:
            logger_->error("Client Already Connected");
            return crf::Code::Vicon_ClientAlreadyConnected;
        case viconSDK::Result::InvalidHostName:
            logger_->error("Invalid Host Name");
            return crf::Code::Vicon_InvalidHostName;
        case viconSDK::Result::ClientConnectionFailed:
            logger_->error("Client Connection Failed");
            return crf::Code::Vicon_ClientConnectionFailed;
        case viconSDK::Result::ArgumentOutOfRange:
            logger_->error("Argument Out Of Range");
            return crf::Code::Vicon_ArgumentOutOfRange;
        case viconSDK::Result::NoFrame:
            logger_->error("Couldn't find a frame to fetch from the server");
            return crf::Code::Vicon_NoFrame;
        case viconSDK::Result::InvalidSubjectName:
            logger_->error("Invalid Subject Name");
            return crf::Code::Vicon_InvalidSubjectName;
        case viconSDK::Result::InvalidSegmentName:
            logger_->error("Invalid Segment Name");
            return crf::Code::Vicon_InvalidSegmentName;
        case viconSDK::Result::InvalidMarkerName:
            logger_->error("Invalid Marker Name");
            return crf::Code::Vicon_InvalidMarkerName;
        default:
            logger_->error("General Error");
            return crf::Code::ThirdPartyQueryFailed;
    }
}

bool ViconMotionCapture::connect() {
    logger_->debug("Connecting to " + host_);
    while (!client_->isConnected().Connected) {
        client_->setConnectionTimeout(timeout_);
        if (evaluate(client_->connect(host_).Result) != crf::Code::Vicon_Success) {
            throw std::runtime_error("Connection Error");
            return false;
        }
    }
    logger_->info("Direct connection to " + host_ + " was successful");
    return true;
}

bool ViconMotionCapture::waitFrame() {
    if (client_->getFrame().Result != viconSDK::Result::Success) {
        logger_->error("Unable to fetch a frame from the server");
        return false;
    }
    return true;
}

bool ViconMotionCapture::initialize() {
    logger_->debug("Initialize");

    if (initialized_) {
        logger_->warn("Already initialized");
        return true;
    }

    // Set data streaming options
    client_->enableSegmentData();
    if (!objectsOnly_) {
        client_->enableMarkerData();
    }
    if (lightweight_) {
        if (client_->enableLightweightSegmentData().Result !=
            viconSDK::Result::Success) {
            logger_->debug("Server does not support lightweight segment data");
        }
    }

    // Set the streaming mode
    switch (streamType_) {
        case StreamingType::ClientPull:
            client_->setStreamMode(viconSDK::StreamMode::ClientPull);
            break;
        case StreamingType::ClientPullPreFetch:
            client_->setStreamMode(viconSDK::StreamMode::ClientPullPreFetch);
            break;
        case StreamingType::ServerPush:
            client_->setStreamMode(viconSDK::StreamMode::ServerPush);
    }

    // Set the global up axis
    switch (axisMapping_) {
        case AxisMappingType::XUp:
            client_->setAxisMapping(
                viconSDK::Direction::Up,
                viconSDK::Direction::Forward,
                viconSDK::Direction::Left);  // X-up
            break;
        case AxisMappingType::YUp:
            client_->setAxisMapping(
                viconSDK::Direction::Forward,
                viconSDK::Direction::Up,
                viconSDK::Direction::Right);  // Y-up
            break;
        default:
            client_->setAxisMapping(
                viconSDK::Direction::Forward,
                viconSDK::Direction::Left,
                viconSDK::Direction::Up);  // Z-up
    }

    // Set client buffer size
    if (clientBufferSize_ > 0) {
        client_->setBufferSize(clientBufferSize_);
        logger_->debug("Setting client buffer size to" + std::to_string(clientBufferSize_));
    }

    if (lightweight_) {
        logger_->warn("Lightweight transmission active. Precision might be decreased");
        logger_->warn("Lightweight transmission active. Marker data streaming is disabled");
    } else if ( objectsOnly_ ) {
        logger_->warn("ObjectsOnly option active. Marker data streaming is disabled");
    }

    initialized_ = true;
    return true;
}

bool ViconMotionCapture::deinitialize() {
    logger_->debug("Deinitialize");
    if (!initialized_) {
        logger_->warn("Already deinitialized");
        return true;
    }
    initialized_ = false;
    return true;
}

crf::expected<std::vector<std::string>> ViconMotionCapture::getObjectNames() {
    if (!client_->isConnected().Connected) return crf::Code::Disconnected;
    if (!initialized_) return crf::Code::NotInitialized;
    if (!waitFrame()) return crf::Code::RequestTimeout;

    int objectsCount = 0;
    std::vector<std::string> objectList;

    const viconSDK::Output_GetSubjectCount outputCount = client_->getSubjectCount();
    if (evaluate(outputCount.Result) == crf::Code::Vicon_Success) {
        objectsCount = outputCount.SubjectCount;
    } else {
        return evaluate(outputCount.Result);
    }

    objectList.resize(objectsCount);
    for (int objectIndex = 0; objectIndex < objectsCount; ++objectIndex) {
        const viconSDK::Output_GetSubjectName outputSubName =
            client_->getSubjectName(objectIndex);
        if (evaluate(outputSubName.Result) == crf::Code::Vicon_Success) {
            objectList[objectIndex] = outputSubName.SubjectName;
        } else {
            return evaluate(outputSubName.Result);
        }
    }
    return objectList;
}

crf::expected<crf::utility::types::TaskPose> ViconMotionCapture::getObjectPose(
    const std::string objectName) {
    if (!client_->isConnected().Connected) return crf::Code::Disconnected;
    if (!initialized_) return crf::Code::NotInitialized;
    if (!waitFrame()) return crf::Code::RequestTimeout;

    viconSDK::Output_GetSegmentGlobalTranslation outputTranslation =
        client_->getSegmentGlobalTranslation(objectName, objectName);
    viconSDK::Output_GetSegmentGlobalRotationQuaternion outputRotation =
        client_->getSegmentGlobalRotationQuaternion(objectName, objectName);

    if (evaluate(outputTranslation.Result) == crf::Code::Vicon_Success &&
            evaluate(outputRotation.Result) == crf::Code::Vicon_Success) {
        if (outputTranslation.Occluded || outputRotation.Occluded) {
            logger_->warn("Object occluded");
        }
        crf::utility::types::TaskPose Pose(
            {outputTranslation.Translation[0],
            outputTranslation.Translation[1],
            outputTranslation.Translation[2]},
            Eigen::Quaterniond(
            outputRotation.Rotation[1],
            outputRotation.Rotation[2],
            outputRotation.Rotation[3],
            outputRotation.Rotation[0]), 1e-7);
        return Pose;
    }
    return evaluate(outputTranslation.Result);
}

crf::expected<std::vector<MotionCaptureMarker>> ViconMotionCapture::getObjectMarkers(
    const std::string objectName) {
    if (!client_->isConnected().Connected) return crf::Code::Disconnected;
    if (!initialized_) return crf::Code::NotInitialized;
    if (!waitFrame()) return crf::Code::RequestTimeout;

    if (lightweight_ || objectsOnly_) {
        return std::vector<MotionCaptureMarker>();
    }

    viconSDK::Output_GetMarkerCount count = client_->getMarkerCount(objectName);
    if (evaluate(count.Result) != crf::Code::Vicon_Success) {
        return evaluate(count.Result);
    }

    std::vector<MotionCaptureMarker> markerList(count.MarkerCount);
    for (unsigned int markerIndex = 0 ; markerIndex < count.MarkerCount ; markerIndex++) {
        viconSDK::Output_GetMarkerName outputMarkerName =
            client_->getMarkerName(objectName, markerIndex);
        if (evaluate(outputMarkerName.Result) != crf::Code::Vicon_Success) {
            continue;
        }
        markerList[markerIndex].markerName = outputMarkerName.MarkerName;
        viconSDK::Output_GetMarkerGlobalTranslation markerPosition =
            client_->getMarkerGlobalTranslation(
                objectName, markerList[markerIndex].markerName);
        for (int positionIndex = 0; positionIndex < 3; positionIndex++) {
            markerList[markerIndex].markerTranslation[positionIndex] =
                markerPosition.Translation[positionIndex];
        }
        if (markerPosition.Occluded) {
            logger_->warn("Marker occluded");
            markerList[markerIndex].markerOccluded = true;
        }
    }
    return markerList;
}

}  // namespace crf::sensors::motioncapture
