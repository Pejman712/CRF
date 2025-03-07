/* © Copyright CERN 2022. All rights reserved. This software is released under a
 * CERN proprietary software license. Any permission to use it shall be granted
 * in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Authors: Ante Marić CERN BE/CEM/MRO 2022
 *          Bartosz Sójka CERN BE/CEM/MRO 2023
 *
 *  ===============================================================================================================
 */

#include "KinematicChain/URDFKinematicChain/URDFKinematicChain.hpp"

namespace crf::math::kinematicchain {

URDFKinematicChain::URDFKinematicChain(
    const std::string& pathToRobotURDF,
    const std::string& endEffectorName,
    const std::string& pathToToolURDF) :
    logger_("URDFKinematicChain"),
    model_(),
    endEffectorName_(endEffectorName),
    descriptionType_(DescriptionType::NotDefined),
    joints_(),
    wheelDrives_(),
    noOfWheels_(0),
    wheelRadius_(0),
    platformLength_(0),
    platformWidth_(0),
    chainSize_(0),
    initialisedGeometricData_(false),
    In_W_(),
    Wn_W_(),
    Ir_IW_(),
    IR_IWZeroPosition_(),
    Jn_J_(),
    Pr_PJZeroPosition_(),
    PR_PJZeroPosition_(),
    Lr_LE_(0.0, 0.0, 0.0),
    LR_LE_(1.0, 0.0, 0.0, 0.0),
    Ir_PJ_(),
    Pr_PJ_(),
    PR_PJ_(),
    Ir_IJ_(),
    IR_IJ_(),
    In_J_(),
    Jr_JE_(),
    Ir_JE_(),
    Ir_IE_(0.0, 0.0, 0.0),
    IR_IE_(1.0, 0.0, 0.0, 0.0),
    jointPositions_(1) {
    logger_->debug("Ctor");

    std::string xmlString;

    logger_->info("Loading XML\"{}\"...", pathToRobotURDF);
    std::fstream xmlFile(pathToRobotURDF, std::fstream::in);

    if (!xmlFile) {
        throw std::invalid_argument(
            "Specified URDF file doesn't exist"
            "There is no file such as " +
            pathToRobotURDF + "!");
    }

    bool foundEe = false;
    while (xmlFile.good()) {
        std::string line, stripped;
        std::getline(xmlFile, line);
        stripped = line;
        remove_if(stripped.begin(), stripped.end(), isspace);
        if (stripped.find("<linkname=\"" + endEffectorName + "\"") != std::string::npos) {
            foundEe = true;
        }
        xmlString += (line + "\n");
    }

    if (!foundEe) {
        throw std::invalid_argument(
            "Specified end effector doesn't exist. There were no link named " + endEffectorName +
            "!");
    }

    logger_->info("Parsing URDF...");
    model_ = urdf::parseURDF(xmlString);

    logger_->info("Parsing for mobile platform");

    bool foundPlatform = false;
    int correctlyNamedWheelsFound = 0;

    wheelDrives_.resize(4, std::shared_ptr<urdf::Joint>());
    In_W_.resize(4, Eigen::Vector3d());
    Wn_W_.resize(4, Eigen::Vector3d());
    Ir_IW_.resize(4, Eigen::Vector3d());
    IR_IWZeroPosition_.resize(4, Eigen::Quaterniond());

    int wheelN = 0;

    bool foundRadius = false;
    double wheelRadius = 0;

    std::shared_ptr<urdf::Link> link;
    for (auto const& [key, link] : model_->links_) {
        logger_->debug(link->name);
        std::string linkName = link->name;
        std::transform(linkName.begin(), linkName.end(), linkName.begin(), [](unsigned char c) {
            return std::tolower(c);
        });
        if (link->child_links.empty() && link->name.find("Wheel") != std::string::npos) {
            logger_->info("Detected wheel: {}", link->name);

            std::shared_ptr<urdf::Cylinder> cylinder =
                std::dynamic_pointer_cast<urdf::Cylinder>(link->collision->geometry);
            wheelRadius_ = cylinder->radius;
            if (!foundRadius) {
                foundRadius = true;
                wheelRadius = wheelRadius_;
            } else {
                if (wheelRadius != wheelRadius_)
                    throw std::invalid_argument("Not all radius of the wheels are the same.");
            }
            if (wheelRadius_ == 0) {
                throw std::invalid_argument("Wheel radius can not be 0!");
            }
            logger_->info("Radius: {}", wheelRadius_);

            std::shared_ptr<urdf::Joint> joint = link->parent_joint;
            Eigen::Vector3d Wn_W;
            Eigen::Vector3d In_W;
            Wn_W << joint->axis.x, joint->axis.y, joint->axis.z;
            if (Wn_W == Eigen::Vector3d::Zero()) {
                throw std::invalid_argument(
                    "Wheel: " + joint->name + "have zero axis!" +
                    "Wheel must have a non-zero axis.");
            }
            Wn_W.normalize();
            wheelN = 0;
            if (link->name.find("frontRightWheel") != std::string::npos) {
                wheelN = 0;
                correctlyNamedWheelsFound++;
            }
            if (link->name.find("frontLeftWheel") != std::string::npos) {
                wheelN = 1;
                correctlyNamedWheelsFound++;
            }
            if (link->name.find("rearLeftWheel") != std::string::npos) {
                wheelN = 2;
                correctlyNamedWheelsFound++;
            }
            if (link->name.find("rearRightWheel") != std::string::npos) {
                wheelN = 3;
                correctlyNamedWheelsFound++;
            }

            wheelDrives_[wheelN] = joint;
            Wn_W_[wheelN] = Wn_W;

            Eigen::Quaterniond IR_IWZeroPosition;
            urdf::Pose parentToJointOriginTransform = joint->parent_to_joint_origin_transform;
            urdf::Rotation IR_IWZeroPositionUrdfRotation = parentToJointOriginTransform.rotation;
            IR_IWZeroPosition.w() = IR_IWZeroPositionUrdfRotation.w;
            IR_IWZeroPosition.x() = IR_IWZeroPositionUrdfRotation.x;
            IR_IWZeroPosition.y() = IR_IWZeroPositionUrdfRotation.y;
            IR_IWZeroPosition.z() = IR_IWZeroPositionUrdfRotation.z;

            IR_IWZeroPosition_[wheelN] = IR_IWZeroPosition;

            In_W = IR_IWZeroPosition * Wn_W;

            if ((In_W - Eigen::Vector3d(0, 1, 0)).norm() > 10e-11) {
                throw std::invalid_argument(
                    "Wheel axis specified in the configuration is not compatible with the standard."
                    "Wheel number" + std::to_string(wheelN) + "have an axis"
                    + std::to_string(In_W[0]) + std::to_string(In_W[1]) + std::to_string(In_W[2]) +
                    "that is not in the y nor -y direction in the inertial frame of reference!"
                    "Wheel must have an axis in the y direction in the "
                    "inertial frame of reference.");
            } else {
                In_W = Eigen::Vector3d(0, 1, 0);
            }

            In_W_[wheelN] = In_W;
        }
    }
    noOfWheels_ = correctlyNamedWheelsFound;
    if (noOfWheels_ == 4) {
        foundPlatform = true;
    }

    if (noOfWheels_ != 4 && noOfWheels_ != 0) {
        throw std::invalid_argument(
            "There weren't 4 correctl named wheels."
            "Found only " +
            std::to_string(noOfWheels_) + " correctly named wheels!");
    }

    if (foundPlatform) {
        parsePlatform();
    } else {
        logger_->info("No platform detected.");
    }

    model_->getLink(endEffectorName, link);
    chainSize_ = 0;
    while (link != model_->getRoot()) {
        logger_->info("\t Link: {}", link->name);
        if (link->parent_joint->type != urdf::Joint::FIXED &&
            link->parent_joint->type != urdf::Joint::UNKNOWN) {
            chainSize_++;
        }
        link = link->getParent();
    }

    joints_.reserve(chainSize_);
    Jn_J_.reserve(chainSize_);
    Pr_PJZeroPosition_.reserve(chainSize_);
    PR_PJZeroPosition_.reserve(chainSize_);

    Ir_PJ_.resize(chainSize_);
    Pr_PJ_.resize(chainSize_);
    PR_PJ_.resize(chainSize_);
    Ir_IJ_.resize(chainSize_);
    IR_IJ_.resize(chainSize_);
    In_J_.resize(chainSize_);
    Jr_JE_.resize(chainSize_);
    Ir_JE_.resize(chainSize_);

    model_->getLink(endEffectorName, link);
    logger_->info("Constructing chain for \"{}\":", endEffectorName);

    int noOfFixedJoints = 0;

    bool lastNonFixedJointParsed = false;
    int lastParsedJointIndex = -1;

    while (link != model_->getRoot()) {
        logger_->info("\t Link: {}", link->name);
        std::shared_ptr<urdf::Joint> joint = link->parent_joint;
        Eigen::Quaterniond PR_PJZeroPosition;
        Eigen::Vector3d Pr_PJZeroPosition;
        Eigen::Vector3d Jn_J;
        if (joint->type != urdf::Joint::UNKNOWN) {
            logger_->info("\t Joint: {} of the type: {}", joint->name, joint->type);
            urdf::Pose parentToJointOriginTransform = joint->parent_to_joint_origin_transform;
            urdf::Vector3 Pr_PJZeroPositionUrdfVector3 = parentToJointOriginTransform.position;
            urdf::Rotation PR_PJZeroPositionUrdfRotation = parentToJointOriginTransform.rotation;

            Pr_PJZeroPosition << Pr_PJZeroPositionUrdfVector3.x, Pr_PJZeroPositionUrdfVector3.y,
                Pr_PJZeroPositionUrdfVector3.z;

            PR_PJZeroPosition.w() = PR_PJZeroPositionUrdfRotation.w;
            PR_PJZeroPosition.x() = PR_PJZeroPositionUrdfRotation.x;
            PR_PJZeroPosition.y() = PR_PJZeroPositionUrdfRotation.y;
            PR_PJZeroPosition.z() = PR_PJZeroPositionUrdfRotation.z;

            if (joint->type != urdf::Joint::FIXED) {
                Pr_PJZeroPosition_.push_back(Pr_PJZeroPosition);
                PR_PJZeroPosition_.push_back(PR_PJZeroPosition);

                Jn_J << joint->axis.x, joint->axis.y, joint->axis.z;
                if (Jn_J == Eigen::Vector3d::Zero()) {
                    throw std::invalid_argument(
                        "Joint: " + joint->name +
                        " have zero axis! Joint must have a non-zero axis");
                }
                Jn_J.normalize();
                Jn_J_.push_back(Jn_J);

                joints_.push_back(joint);
                lastParsedJointIndex++;

                lastNonFixedJointParsed = true;
            } else {
                noOfFixedJoints++;
                if (lastNonFixedJointParsed) {
                    Pr_PJZeroPosition_[lastParsedJointIndex] = Pr_PJZeroPosition +
                        PR_PJZeroPosition * Pr_PJZeroPosition_[lastParsedJointIndex];
                    PR_PJZeroPosition_[lastParsedJointIndex] =
                        PR_PJZeroPosition * PR_PJZeroPosition_[lastParsedJointIndex];
                } else {
                    Lr_LE_ = Pr_PJZeroPosition + PR_PJZeroPosition * Lr_LE_;
                    LR_LE_ = PR_PJZeroPosition * LR_LE_;
                }
            }
        } else {
            throw std::invalid_argument(
                "Joint: " + joint->name + " is of the type 'unknown'! "
                "Joints of unknown type are not allowed.");
        }

        link = link->getParent();
    }
    std::reverse(joints_.begin(), joints_.end());
    std::reverse(Pr_PJZeroPosition_.begin(), Pr_PJZeroPosition_.end());
    std::reverse(PR_PJZeroPosition_.begin(), PR_PJZeroPosition_.end());
    std::reverse(Jn_J_.begin(), Jn_J_.end());

    if (chainSize_ != joints_.size()) {
        throw std::logic_error(
            "Error of kinematic chain structure construction, "
            "chainSize_ != joints_.size()\nchainSize_ is equal to " +
            std::to_string(chainSize_) + ", but joints_.size() is equal to " +
            std::to_string(joints_.size()) + "!");
    }

    if (chainSize_ == 0 && !foundPlatform) {
        logger_->info("Found no moveable joints or wheels - assuming Tool description");
        descriptionType_ = DescriptionType::Tool;
        return;
    } else if (chainSize_ == 0) {
        logger_->info("Found no moveable joints, but found wheels - assuming "
                      "Platform description");
        descriptionType_ = DescriptionType::Platform;
        return;
    } else if (!foundPlatform) {
        logger_->info("Found moveable joints, but no wheels - assuming _arm description");
        descriptionType_ = DescriptionType::Arm;
    } else {
        logger_->info("Found moveable joints and wheels - assuming Combined description");
        descriptionType_ = DescriptionType::Combined;
    }

    if (pathToToolURDF != "") {
        parseToolURDF(pathToToolURDF);
    }

    int DoF = static_cast<int>(noOfWheels_) + static_cast<int>(chainSize_);
    std::vector<double> jointPositions0StdVector;
    jointPositions0StdVector.resize(static_cast<size_t>(DoF));

    for (int i = 0; i < DoF; i++) {
        jointPositions0StdVector[i] = 0.0;
    }

    crf::utility::types::JointPositions jointPositions0(jointPositions0StdVector);
    jointPositions_ = jointPositions0;
    if (descriptionType_ == DescriptionType::Arm || descriptionType_ == DescriptionType::Combined) {
        setJointPositions(jointPositions0);
    }
}

URDFKinematicChain::~URDFKinematicChain() {
    logger_->debug("DTor");
}

void URDFKinematicChain::parseToolURDF(const std::string& pathToToolURDF) {
    std::string xmlString;

    logger_->info("Loading tool XML\"{}\"...", pathToToolURDF);
    std::fstream xmlFile(pathToToolURDF, std::fstream::in);

    if (!xmlFile) {
        throw std::invalid_argument(
            "Specified tool URDF file doesn't exist. There is no file such as " +
            pathToToolURDF + "!");
    }

    while (xmlFile.good()) {
        std::string line, stripped;
        std::getline(xmlFile, line);
        xmlString += (line + "\n");
    }

    logger_->info("Parsing tool URDF...");
    model_ = urdf::parseURDF(xmlString);

    std::shared_ptr<urdf::Link> link;
    model_->getLink("leafLink", link);

    std::shared_ptr<urdf::Joint> joint = link->parent_joint;

    Eigen::Vector3d Er_ET;
    Eigen::Quaterniond ER_ET;

    urdf::Pose parentToJointOriginTransform = joint->parent_to_joint_origin_transform;
    urdf::Vector3 Er_ETUrdfVector3 = parentToJointOriginTransform.position;
    urdf::Rotation ER_ETUrdfRotation = parentToJointOriginTransform.rotation;

    Er_ET << Er_ETUrdfVector3.x, Er_ETUrdfVector3.y, Er_ETUrdfVector3.z;

    ER_ET.w() = ER_ETUrdfRotation.w;
    ER_ET.x() = ER_ETUrdfRotation.x;
    ER_ET.y() = ER_ETUrdfRotation.y;
    ER_ET.z() = ER_ETUrdfRotation.z;

    Lr_LE_ = Lr_LE_ + LR_LE_ * Er_ET;
    LR_LE_ = LR_LE_ * ER_ET;
}

void URDFKinematicChain::parsePlatform() {
    logger_->info("Parse platform");

    std::shared_ptr<urdf::Joint> drive;
    std::shared_ptr<urdf::Link> link;

    for (int wheelN = 0; wheelN < noOfWheels_; wheelN++) {
        drive = wheelDrives_[wheelN];
        logger_->debug("Drive: {}", drive->name);
        model_->getLink(drive->parent_link_name, link);

        Eigen::Vector3d Pr_PW;
        Eigen::Vector3d Ir_IW(0, 0, 0);
        Eigen::Quaterniond PR_PWZeroPosition;
        Eigen::Quaterniond IR_IWZeroPosition(1, 0, 0, 0);

        urdf::Pose parentToJointOriginTransform = drive->parent_to_joint_origin_transform;
        urdf::Vector3 Pr_PWUrdfVector3 = parentToJointOriginTransform.position;

        urdf::Rotation PR_PWZeroPositionUrdfRotation = parentToJointOriginTransform.rotation;

        Pr_PW << Pr_PWUrdfVector3.x, Pr_PWUrdfVector3.y, Pr_PWUrdfVector3.z;

        PR_PWZeroPosition.w() = PR_PWZeroPositionUrdfRotation.w;
        PR_PWZeroPosition.x() = PR_PWZeroPositionUrdfRotation.x;
        PR_PWZeroPosition.y() = PR_PWZeroPositionUrdfRotation.y;
        PR_PWZeroPosition.z() = PR_PWZeroPositionUrdfRotation.z;

        Ir_IW += Pr_PW;

        IR_IWZeroPosition = PR_PWZeroPosition;

        Ir_IW_[wheelN] = Ir_IW;
        IR_IWZeroPosition_[wheelN] = PR_PWZeroPosition;
    }

    double widthDistance1 = (Ir_IW_[0] - Ir_IW_[1]).norm();
    double widthDistance2 = (Ir_IW_[2] - Ir_IW_[3]).norm();

    double lengthDistance1 = (Ir_IW_[0] - Ir_IW_[3]).norm();
    double lengthDistance2 = (Ir_IW_[1] - Ir_IW_[2]).norm();

    if (abs(widthDistance1 - widthDistance2) > 10e-11) {
        throw std::invalid_argument(
            "Width distance between 0th and 1st wheel --" + std::to_string(widthDistance1) +
            "is different than the width distance between 2nd and "
            "3rd wheel --" + std::to_string(widthDistance2) + "."
            "Platform should have the same width distance between each pair of wheels!");
    }

    if (abs(lengthDistance1 - lengthDistance2) > 10e-11) {
        throw std::invalid_argument(
            "Length distance between 0th and 3rd wheel --" + std::to_string(lengthDistance1) +
            "is different than the length distance between 1st and "
            "2nd wheel -- " + std::to_string(lengthDistance2) + "."
            "Platform should have the same length distance between each pair of wheels!");
    }

    platformWidth_ = widthDistance1;
    platformLength_ = lengthDistance1;
}

DescriptionType URDFKinematicChain::getType() {
    return descriptionType_;
}

int URDFKinematicChain::getJointType(const int& jointIndex) {
    return joints_[jointIndex]->type;
}

size_t URDFKinematicChain::getChainSize() const {
    return chainSize_;
}

size_t URDFKinematicChain::getNumWheels() const {
    return noOfWheels_;
}

double URDFKinematicChain::getWheelRadius() {
    if (descriptionType_ != DescriptionType::Platform &&
        descriptionType_ != DescriptionType::Combined) {
        throw std::runtime_error("Called getWheelRadius on a description type without platform!");
    }
    return wheelRadius_;
}

double URDFKinematicChain::getPlatformL() {
    if (descriptionType_ != DescriptionType::Platform &&
        descriptionType_ != DescriptionType::Combined) {
        throw std::runtime_error("Called getPlatformL on a description type without platform!");
    }
    return platformLength_;
}

double URDFKinematicChain::getPlatformW() {
    if (descriptionType_ != DescriptionType::Platform &&
        descriptionType_ != DescriptionType::Combined) {
        throw std::runtime_error("Called getPlatformW on a description type without platform!");
    }
    return platformWidth_;
}

// MODIFYING THE DYNAMIC DATA

void URDFKinematicChain::setJointPositions(const utility::types::JointPositions& jointPositions) {
    logger_->debug("setJointPositions");

    if (descriptionType_ == DescriptionType::Platform ||
        descriptionType_ == DescriptionType::Tool) {
        logger_->info("Called setJointPositions of a {} description type!", descriptionType_);
        return;
    }

    size_t numWheels = getNumWheels();

    if (jointPositions.size() != static_cast<size_t>(chainSize_ + numWheels)) {
        throw std::invalid_argument(
            "Invalid size of configuration vector! JointPositions size was " +
            std::to_string(jointPositions.size()) + ", but chainSize_ (" +
            std::to_string(chainSize_) + ") + noOfWheels_ (" + std::to_string(getNumWheels()) +
            ") = " + std::to_string(chainSize_ + numWheels) + ".");
    }

    utility::types::JointPositions jointPositions_armOnly(chainSize_);
    for (size_t i = 0; i < chainSize_; i++) {
        jointPositions_armOnly[i] = jointPositions[i + numWheels];
    }

    utility::types::JointPositions current_jointPositions_armOnly(chainSize_);
    for (size_t i = 0; i < chainSize_; i++) {
        current_jointPositions_armOnly[i] = jointPositions_[i + numWheels];
    }

    jointPositions_ = jointPositions;
    std::vector<bool> jointPositionIsChanged;
    jointPositionIsChanged.assign(chainSize_, true);
    int firstChangedJoint = chainSize_;
    int lastChangedJoint = chainSize_ - 1;

    for (int i = 0; i < static_cast<int>(chainSize_); i++) {
        if (jointPositions_armOnly[i] == current_jointPositions_armOnly[i]) {
            jointPositionIsChanged[i] = false;
        } else {
            if (firstChangedJoint == static_cast<int>(chainSize_)) {
                firstChangedJoint = i;
            }
            lastChangedJoint = i;
        }
    }

    if (initialisedGeometricData_ == false) {
        firstChangedJoint = 0;
        lastChangedJoint = chainSize_ - 1;
        for (int i = 0; i < static_cast<int>(chainSize_); i++) {
            jointPositionIsChanged[i] = true;
        }
    }

    if (firstChangedJoint == static_cast<int>(chainSize_)) {
        return;
    }

    std::shared_ptr<urdf::Joint> joint;

    Eigen::Vector3d Pr_PJ;
    Eigen::Vector3d Ir_PJ;
    Eigen::Vector3d Ir_IJ(0.0, 0.0, 0.0);
    if (firstChangedJoint > 0) {
        Ir_IJ = Ir_IJ_[firstChangedJoint - 1];
    }
    Eigen::Quaterniond PR_PJ;
    Eigen::Quaterniond IR_IJ(1.0, 0.0, 0.0, 0.0);
    if (firstChangedJoint > 0) {
        IR_IJ = IR_IJ_[firstChangedJoint - 1];
    }
    Eigen::Vector3d Jn_J;
    Eigen::Vector3d In_J;
    Eigen::Vector3d Jr_JE;
    Eigen::Vector3d Ir_JE;

    for (int i = firstChangedJoint; i < static_cast<int>(chainSize_); i++) {
        Jn_J = Jn_J_[i];

        if (jointPositionIsChanged[i]) {
            Pr_PJ = Pr_PJZeroPosition_[i];
            PR_PJ = PR_PJZeroPosition_[i];
            joint = joints_[i];
            logger_->debug(joint->name);
            double jointPosition = jointPositions_armOnly[i];

            if (joint->type != urdf::Joint::PRISMATIC) {
                Eigen::AngleAxisd PR_PJJointMovementAngleAxis(jointPosition, Jn_J);
                Eigen::Quaterniond PR_PJJointMovement(PR_PJJointMovementAngleAxis);
                PR_PJ = PR_PJ * PR_PJJointMovement;
            } else {
                Pr_PJ += PR_PJ * (Jn_J * jointPosition);
            }

            Pr_PJ_[i] = Pr_PJ;
            PR_PJ_[i] = PR_PJ;
        } else {
            Pr_PJ = Pr_PJ_[i];
            PR_PJ = PR_PJ_[i];
        }

        IR_IJ = IR_IJ * PR_PJ;
        In_J = IR_IJ * Jn_J;

        IR_IJ_[i] = IR_IJ;
        In_J_[i] = In_J;
    }

    for (int i = firstChangedJoint; i < static_cast<int>(chainSize_); i++) {
        if (i == 0) {
            Ir_PJ = Pr_PJ_[0];
        } else {
            Ir_PJ = IR_IJ_[i - 1] * Pr_PJ_[i];
        }
        Ir_PJ_[i] = Ir_PJ;
        Ir_IJ += Ir_PJ;
        Ir_IJ_[i] = Ir_IJ;
    }

    IR_IE_ = IR_IJ_[chainSize_ - 1] * LR_LE_;

    Jr_JE = Lr_LE_;

    for (int i = chainSize_ - 1; i > lastChangedJoint; i--) {
        Jr_JE = Jr_JE_[i];
        Ir_JE = IR_IJ_[i] * Jr_JE;
        Ir_JE_[i] = Ir_JE;
    }

    Jr_JE = Jr_JE_[lastChangedJoint];

    if (initialisedGeometricData_ == false) {
        Jr_JE = Lr_LE_;
    }

    for (int i = lastChangedJoint; i >= 0; i--) {
        Ir_JE = IR_IJ_[i] * Jr_JE;
        Jr_JE_[i] = Jr_JE;
        Ir_JE_[i] = Ir_JE;

        Jr_JE = PR_PJ_[i] * Jr_JE;
        Jr_JE += Pr_PJ_[i];
    }
    Ir_IE_ = Jr_JE;

    if (initialisedGeometricData_ == false) {
        initialisedGeometricData_ = true;
    }
}

// ACCESSING DYNAMIC DATA

Eigen::Vector3d URDFKinematicChain::getAxis(const Axes axis, const int& wheelOrJointN) {
    logger_->debug("getAxis");
    if (axis == Axes::IW || axis == Axes::WW) {
        if (descriptionType_ != DescriptionType::Platform &&
            descriptionType_ != DescriptionType::Combined) {
            throw std::runtime_error("Called getAxis IW on a description type without platform!");
        }
        if (wheelOrJointN < 0 || 4 <= wheelOrJointN) {
            throw std::invalid_argument(
                "Invalid wheel number! Called getAxis IW with the wheel index " +
                std::to_string(wheelOrJointN) + "!");
        }
    }
    if (axis == Axes::IJ || axis == Axes::JJ) {
        if (descriptionType_ != DescriptionType::Arm &&
            descriptionType_ != DescriptionType::Combined) {
            throw std::runtime_error("Called getAxis IJ or JJ on a description type without arm!");
        }
        if (wheelOrJointN < 0 || wheelOrJointN >= static_cast<int>(chainSize_)) {
            throw std::invalid_argument(
                "Invalid joint index. Joint number " + std::to_string(wheelOrJointN) +
                " was requested in getAxis IJ or JJ in a kinematic chain of " +
                std::to_string(chainSize_) + " chainSize_!");
        }
    }
    switch (axis) {
        case Axes::IW:
            return In_W_[wheelOrJointN];
            break;
        case Axes::WW:
            return Wn_W_[wheelOrJointN];
            break;
        case Axes::IJ:
            return In_J_[wheelOrJointN];
            break;
        case Axes::JJ:
            return Jn_J_[wheelOrJointN];
            break;
    }
    throw std::logic_error("Implemented axis is missing in the switch case in getAxis.");
}

Eigen::Vector3d URDFKinematicChain::getTranslation(
    const Translations translation,
    const int& wheelOrJointN) {
    logger_->debug("getTranslation");
    if (translation == Translations::IIW) {
        if (descriptionType_ != DescriptionType::Platform &&
            descriptionType_ != DescriptionType::Combined) {
            throw std::runtime_error(
                "Called getTranslation IIW on a description type without platform!");
        }
        if (wheelOrJointN < 0 || 4 <= wheelOrJointN) {
            throw std::invalid_argument(
                "Invalid wheel index!"
                "Called getTranslation IIW with the wheel index " +
                std::to_string(wheelOrJointN) + "!");
        }
    }
    if (translation == Translations::IIJ || translation == Translations::IIE ||
        translation == Translations::IPJ || translation == Translations::IJE ||
        translation == Translations::PPJ || translation == Translations::JJE ||
        translation == Translations::LLE) {
        if (descriptionType_ != DescriptionType::Arm &&
            descriptionType_ != DescriptionType::Combined) {
            throw std::runtime_error("Called getTranslation IIJ, IPJ, IJE, PPJ, JJE, or LLE "
                "on a description type without arm!");
        }
        if (wheelOrJointN < 0 || wheelOrJointN >= static_cast<int>(chainSize_)) {
            throw std::invalid_argument(
                "Invalid joint index! Called getTranslation IIJ, IPJ, IJE, PPJ, JJE, or LLE "
                "with the wheel index " + std::to_string(wheelOrJointN) + "!");
        }
    }
    switch (translation) {
        case Translations::IIW:
            return Ir_IW_[wheelOrJointN];
            break;
        case Translations::IIJ:
            return Ir_IJ_[wheelOrJointN];
            break;
        case Translations::IIE:
            return Ir_IE_;
            break;
        case Translations::IPJ:
            return Ir_PJ_[wheelOrJointN];
            break;
        case Translations::IJE:
            return Ir_JE_[wheelOrJointN];
            break;
        case Translations::PPJ:
            return Pr_PJ_[wheelOrJointN];
            break;
        case Translations::JJE:
            return Jr_JE_[wheelOrJointN];
            break;
        case Translations::LLE:
            return Lr_LE_;
            break;
    }
    throw std::logic_error("Implemented translation is missing in the switch case in "
        "getTranslation.");
}

Eigen::Quaterniond URDFKinematicChain::getRotation(
    const Rotations rotation,
    const int& wheelOrJointN) {
    logger_->debug("getRotation");
    if (rotation == Rotations::IIW) {
        if (descriptionType_ != DescriptionType::Platform &&
            descriptionType_ != DescriptionType::Combined) {
            throw std::runtime_error(
                "Called getRotation IIW on a description type without platform!");
        }
        if (wheelOrJointN < 0 || 4 <= wheelOrJointN) {
            throw std::invalid_argument(
                "Invalid wheel index! Called getRotation IIW with the wheel index " +
                std::to_string(wheelOrJointN) + "!");
        }
    }
    if (rotation == Rotations::IIJ || rotation == Rotations::PPJ || rotation == Rotations::IIE ||
        rotation == Rotations::LLE) {
        if (descriptionType_ != DescriptionType::Arm &&
            descriptionType_ != DescriptionType::Combined) {
            throw std::runtime_error("Called getRotation IIJ, PPJ or LLE "
                "on a description type without arm!");
        }
        if (wheelOrJointN < 0 || wheelOrJointN >= static_cast<int>(chainSize_)) {
            throw std::invalid_argument(
                "Invalid joint index! Called getTranslation IIJ, PPJ or LLE "
                "with the wheel index " + std::to_string(wheelOrJointN) + "!");
        }
    }
    switch (rotation) {
        case Rotations::IIW:
            return IR_IWZeroPosition_[wheelOrJointN];
            break;
        case Rotations::IIJ:
            return IR_IJ_[wheelOrJointN];
            break;
        case Rotations::IIE:
            return IR_IE_;
            break;
        case Rotations::PPJ:
            return PR_PJ_[wheelOrJointN];
            break;
        case Rotations::LLE:
            return LR_LE_;
            break;
    }
    throw std::logic_error("Implemented rotation is missing in the switch case in "
        "getRotation.");
}

Eigen::Vector3d URDFKinematicChain::computeAxis(
    const Axes axis,
    const int& wheelOrJointN,
    const crf::utility::types::JointPositions& jointPositions) {
    logger_->debug("computeAxis");
    setJointPositions(jointPositions);
    return getAxis(axis, wheelOrJointN);
}

Eigen::Vector3d URDFKinematicChain::computeTranslation(
    const Translations translation,
    const int& wheelOrJointN,
    const crf::utility::types::JointPositions& jointPositions) {
    logger_->debug("computeTranslaton");
    setJointPositions(jointPositions);
    return getTranslation(translation, wheelOrJointN);
}

Eigen::Quaterniond URDFKinematicChain::computeRotation(
    const Rotations rotation,
    const int& wheelOrJointN,
    const crf::utility::types::JointPositions& jointPositions) {
    logger_->debug("computeRotation");
    setJointPositions(jointPositions);
    return getRotation(rotation, wheelOrJointN);
}

}  // namespace crf::math::kinematicchain
