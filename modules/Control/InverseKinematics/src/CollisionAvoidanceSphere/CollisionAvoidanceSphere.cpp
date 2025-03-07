/* © Copyright CERN 2022. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Laura Rodrigo Pérez CERN BE/CEM/MRO 2022
 *
 *  ================================================================================================================
*/

#include <math.h>

#include "InverseKinematics/CollisionAvoidanceSphere/CollisionAvoidanceSphere.hpp"

using crf::utility::types::JointPositions;

namespace crf::control::inversekinematics {

CollisionAvoidanceSphere::CollisionAvoidanceSphere(double rangeSinusoid, double cycleTime,
    int curveType, double c, double p, Eigen::Vector3d center, double radius, int robot):
    logger_("CollisionAvoidanceSphere"),
    rangeSinusoid_(rangeSinusoid),
    cycleTime_(cycleTime),
    curveType_(curveType),
    c_(c),
    p_(p),
    center_(center),
    radius_(radius),
    robot_(robot),
    dq_(1e-4) {
    logger_->debug("CTor");
    if (rangeSinusoid <= 0 || cycleTime <= 0 || c <= 0 || radius <= 0) {
        throw std::invalid_argument("Wrong value in one or some input arguments");
    }
    if (curveType < 0 || curveType > 1) {
        throw std::invalid_argument("Wrong value in curve type input");
    }
    if (curveType == 1 && p <= 0) {
        throw std::invalid_argument("In exponential curve type, the parameter p is used and it "
            "needs to be bigger than 0");
    }
    if (robot < 0 || robot > 3) {
        throw std::invalid_argument("Wrong value in robot input");
    }
    increasingSinusoid_.reset(new crf::math::geometricmethods::Sinusoid(0, 1, rangeSinusoid_,
        crf::math::geometricmethods::ComputationMethod::SetRange));
    decreasingSinusoid_.reset(new crf::math::geometricmethods::Sinusoid(1, 0, rangeSinusoid_,
        crf::math::geometricmethods::ComputationMethod::SetRange));
    inTransition_ = false;
    transtionEvaluationTime_ = -1.0;
    goToNextIteration_ = true;
    enabled_ = false;
    startTransition_ = false;
}

CollisionAvoidanceSphere::~CollisionAvoidanceSphere() {
    logger_->debug("DTor");
}

Eigen::MatrixXd CollisionAvoidanceSphere::getGradient(
    const crf::utility::types::JointPositions& q,
    const crf::utility::types::JointPositions& qAttr) {
    logger_->debug("getGradient");
    unsigned int qSize = q.size();
    // We cannot check if the size of q is correct.
    Eigen::MatrixXd gradientPenalty(static_cast<int>(qSize), 1);
    transitionFactor_ = getTransitionFactor();

    // Distance calculation
    // The size of d corresponds to the number of links that can collide with the collision object.
    Eigen::VectorXd distance = calcMinDistRobot(q, center_);
    // Artificial potential
    Eigen::VectorXd V(distance.size());  // dim(V) = n x 1 w/ n number of links
    for (unsigned int i = 0; i < distance.size(); i++) {
        if (curveType_ == 0) {  // Quadratic curve
            if (distance(i) > radius_) {
                V(i) = 0.0;
            } else {
                V(i) = 1.0/2.0 * -c_ * ((distance(i) - radius_) * (distance(i) - radius_));
            }
        } else {  // Exponential curve
            V(i) = -p_ * exp(-c_ * (distance(i) - radius_));
        }
    }
    // Numeric derivative
    crf::utility::types::JointPositions qDq(q.size());
    Eigen::VectorXd dQi(distance.size());
    Eigen::VectorXd dV(distance.size());
    Eigen::MatrixXd p(q.size(), distance.size());
    for (unsigned int i = 0; i < q.size(); i++) {
        qDq = q;
        qDq[i] = qDq[i] + dq_;
        dQi = calcMinDistRobot(qDq, center_);
        for (unsigned int j = 0; j < distance.size(); j++) {
            if (curveType_ == 0) {  // Quadratic curve
                if (dQi(j) > radius_) {
                    dV(j) = 0.0;
                } else {
                    dV(j) = 1.0/2.0 * -c_ * ((dQi(j) - radius_) * (dQi(j) - radius_));
                }
            } else {  // Exponential curve
                dV(j) = -p_ * exp(-c_ * (dQi(j) - radius_));
            }
            dV(j) = dV(j) - V(j);
        }
        // Derivative for every link p = dV/dq_
        p.row(i) = dV.transpose()/dq_;
        gradientPenalty(i, 0) = p.row(i).sum() * transitionFactor_;
    }

    return gradientPenalty;
}

std::vector<double> CollisionAvoidanceSphere::getTimeDerivative(
    const crf::utility::types::JointPositions& q,
    const crf::utility::types::JointVelocities& qd,
    const crf::utility::types::JointPositions& qAttr) {
    logger_->debug("getTimeDerivative");
    logger_->error("This method is still not implemented for this objective function");
    return std::vector<double>(0);
}

bool CollisionAvoidanceSphere::enable(bool state) {
    logger_->debug("setEnable");
    if (inTransition_ || startTransition_) {
        return false;
    }
    if (state != enabled_) {
        startTransition_ = true;
    }
    enabled_ = state;
    return true;
}

bool CollisionAvoidanceSphere::enable() const {
    logger_->debug("getEnable");
    return enabled_;
}

void CollisionAvoidanceSphere::goToNextIteration(const bool& next) {
    logger_->debug("goToNextIteration");
    goToNextIteration_ = next;
}

double CollisionAvoidanceSphere::getTransitionFactor() {
    logger_->debug("getTransitionFactor");
    if (goToNextIteration_ && inTransition_) {
        transtionEvaluationTime_ = transtionEvaluationTime_ + cycleTime_;
    }
    if (startTransition_) {
        startTransition_ = false;
        inTransition_ = true;
        transtionEvaluationTime_ = 0.0;
    }
    if (transtionEvaluationTime_ < 0.0 || transtionEvaluationTime_ > rangeSinusoid_) {
        inTransition_ = false;
        if (enabled_) {
            return 1.0;
        }
        // !enabled_
        return 0.0;
    }
    if (enabled_) {
        return increasingSinusoid_->evaluate(transtionEvaluationTime_, 0).value();
    }
    // !enabled_
    return decreasingSinusoid_->evaluate(transtionEvaluationTime_, 0).value();
}

bool CollisionAvoidanceSphere::setParam(std::string objFuncName, std::string value) {
    logger_->debug("setParam");
    if (std::stod(value) <= 0) {
        logger_->error("Wrong imput in value");
        return false;
    }
    if (enabled_ || inTransition_ || startTransition_) {
        logger_->error("Parameters cannot change if the objective function is enabled or "
            "in transition to be enabled/disabled");
        return false;
    }
    if (objFuncName == "c") {
        c_ = std::stod(value);
        return true;
    }
    logger_->error("Wrong imput in objFuncName");
    return false;
}

double CollisionAvoidanceSphere::calcMinDistLink(
    Eigen::VectorXd frame,
    Eigen::VectorXd nextFrame,
    Eigen::VectorXd obstacleCentre,
    double length) {

    Eigen::VectorXd unitary = (nextFrame - frame) / length;
    double dotProduct = unitary.transpose() * (obstacleCentre - frame);

    double distance = 0.0;
    if (dotProduct < 0.0) {
        // Angle of dotProduct bigger than 90 degrees and smaller than 270 degrees. Frame is the
        // nearest point of the link to the obstacleCentre.
        distance = (frame - obstacleCentre).norm();
    } else if (dotProduct > length) {
        // Angle of dotProduct small. NextFrame is the nearest point of the link to the
        // obstacleCentre.
        distance = (nextFrame - obstacleCentre).norm();
    } else {
        // The nearest point of the link to the obstacleCentre is neither of the two ends of the
        // link.
        distance = ((frame + dotProduct * unitary) - obstacleCentre).norm();
    }

    return distance;
}

Eigen::VectorXd CollisionAvoidanceSphere::calcMinDistRobot(
    crf::utility::types::JointPositions q,
    Eigen::VectorXd obstacleCentre) {

    unsigned int k = 3;
    Eigen::MatrixXd IrIx(3, k);
    unsigned int n = 2;  // Number of collision objects
    Eigen::VectorXd lengthCollisionObject;
    Eigen::VectorXd d;
    if (robot_ == 0) {  // 2DOF robot
        // Load geometric parameters
        double l1x = 1.0, l2x = 1.0;
        n = 2;  // Number of collision objects
        lengthCollisionObject = Eigen::VectorXd(n);
        lengthCollisionObject << l1x, l2x;
        // Set the kinematics
        Eigen::Vector3d IrI(0.0, 0.0, 0.0);
        Eigen::Vector3d IrI1(l1x*cos(q[0]),
                             l1x*sin(q[0]),
                             0.0);
        Eigen::Vector3d IrIE(l1x*cos(q[0]) + l2x*cos(q[0] + q[1]),
                             l1x*sin(q[0]) + l2x*sin(q[0] + q[1]),
                             0.0);
        k = 3;
        IrIx = Eigen::MatrixXd(3, k);
        IrIx << IrI, IrI1, IrIE;

    } else if (robot_ == 1) {  // 3DOF robot
        // Load geometric parameters
        double l1x = 1.0, l2x = 1.0, l3x = 1.0;
        n = 3;  // Number of collision objects
        lengthCollisionObject = Eigen::VectorXd(n);
        lengthCollisionObject << l1x, l2x, l3x;
        // Set the kinematics
        Eigen::Vector3d IrI(0.0, 0.0, 0.0);
        Eigen::Vector3d IrI1(l1x*cos(q[0]),
                             l1x*sin(q[0]),
                             0.0);
        Eigen::Vector3d IrI2(l1x*cos(q[0]) + l2x*cos(q[0] + q[1]),
                             l1x*sin(q[0]) + l2x*sin(q[0] + q[1]),
                             0.0);
        Eigen::Vector3d IrIE(l1x*cos(q[0]) + l2x*cos(q[0] + q[1]) + l3x*cos(q[0] + q[1] + q[2]),
                             l1x*sin(q[0]) + l2x*sin(q[0] + q[1]) + l3x*sin(q[0] + q[1] + q[2]),
                             0.0);
        k = 4;
        IrIx = Eigen::MatrixXd(3, k);
        IrIx << IrI, IrI1, IrI2, IrIE;

    } else if (robot_ == 2) {  // UR10e - 6DOF robot
        // Load geometric parameters
        double l2z = 0.1807, l3x = 0.6127, l4x = 0.57155, l5z = 0.17415, l6y = 0.11655,
            l6z = 0.11985;
        n = 6;  // Number of collision objects
        lengthCollisionObject = Eigen::VectorXd(n);
        lengthCollisionObject << l2z, l3x, l4x, l5z, l6y, l6z;
        // Set the kinematics
        Eigen::Vector3d IrI1(0.0, 0.0, 0.0);
        Eigen::Vector3d IrI2(0.0, 0.0, l2z);
        Eigen::Vector3d IrI3(-cos(q[0]) * cos(q[1]) * l3x,
                             -sin(q[0]) * cos(q[1]) * l3x,
                             l2z - sin(q[1]) * l3x);
        Eigen::Vector3d IrI4(((-l4x * cos(q[2]) - l3x) * cos(q[1]) + sin(q[2]) * sin(q[1]) * l4x) * cos(q[0]),  // NOLINT
                             ((-l4x * cos(q[2]) - l3x) * cos(q[1]) + sin(q[2]) * sin(q[1]) * l4x) * sin(q[0]),  // NOLINT
                             sin(q[1]) * (-l4x * cos(q[2]) - l3x) - sin(q[2]) * cos(q[1]) * l4x + l2z);  // NOLINT
        Eigen::Vector3d IrI5(((-l4x * cos(q[2]) - l3x) * cos(q[1]) + sin(q[2]) * sin(q[1]) * l4x) * cos(q[0]) + sin(q[0]) * l5z,  // NOLINT
                             ((-l4x * cos(q[2]) - l3x) * cos(q[1]) + sin(q[2]) * sin(q[1]) * l4x) * sin(q[0]) - cos(q[0]) * l5z,  // NOLINT
                             sin(q[1]) * (-l4x * cos(q[2]) - l3x) - sin(q[2]) * cos(q[1]) * l4x + l2z);  // NOLINT
        Eigen::Vector3d IrIE((((-cos(q[3]) * sin(q[4]) * l6y + l6z * sin(q[3]) - l4x) * cos(q[2]) + (sin(q[3]) * sin(q[4]) * l6y + l6z * cos(q[3])) * sin(q[2]) - l3x) * cos(q[1]) + sin(q[1]) * ((sin(q[3]) * sin(q[4]) * l6y + l6z * cos(q[3])) * cos(q[2]) - sin(q[2]) * (-cos(q[3]) * sin(q[4]) * l6y + l6z * sin(q[3]) - l4x))) * cos(q[0]) + sin(q[0]) * (cos(q[4]) * l6y + l5z),  // NOLINT
                             (((-cos(q[3]) * sin(q[4]) * l6y + l6z * sin(q[3]) - l4x) * cos(q[2]) + (sin(q[3]) * sin(q[4]) * l6y + l6z * cos(q[3])) * sin(q[2]) - l3x) * cos(q[1]) + sin(q[1]) * ((sin(q[3]) * sin(q[4]) * l6y + l6z * cos(q[3])) * cos(q[2]) - sin(q[2]) * (-cos(q[3]) * sin(q[4]) * l6y + l6z * sin(q[3]) - l4x))) * sin(q[0]) - cos(q[0]) * (cos(q[4]) * l6y + l5z),  // NOLINT
                             ((-cos(q[3]) * sin(q[4]) * l6y + l6z * sin(q[3]) - l4x) * cos(q[2]) + (sin(q[3]) * sin(q[4]) * l6y + l6z * cos(q[3])) * sin(q[2]) - l3x) * sin(q[1]) + ((-sin(q[3]) * sin(q[4]) * l6y - l6z * cos(q[3])) * cos(q[2]) - sin(q[2]) * (cos(q[3]) * sin(q[4]) * l6y - l6z * sin(q[3]) + l4x)) * cos(q[1]) + l2z);  // NOLINT
        k = 6;
        IrIx = Eigen::MatrixXd(3, k);
        IrIx << IrI1, IrI2, IrI3, IrI4, IrI5, IrIE;

    } else {  // TIMArm - 9 DOF robot
        // Load geometric parameters
        double l2y = 0.0745, l2z = 0.109, l3x = 0.461, l3y = 0.0745, l4x = 0.2755, l6x = 0.41,
            l7x = 0.2073, l7y = 0.0133, l8x = 0.1038, l9x = 0.1038, lRail = 1.2817;
        unsigned int n = 5;  // Number of collision objects
        Eigen::VectorXd lCollObj(n);
        lCollObj << lRail, l4x, l6x, l7x+l8x, l9x;
        // Set the kinematics
        Eigen::Vector3d IrI2(sin(q[0]) * l2y,
                             -cos(q[0]) * l2y,
                             -l2z);
        Eigen::Vector3d IrIEndRail(cos(q[1]) * lRail * cos(q[0]) + sin(q[0]) * (l2y - l3y),
                                   cos(q[0]) * (-l2y + l3y) + cos(q[1]) * sin(q[0]) * lRail,
                                   lRail * sin(q[1]) - l2z);
        Eigen::Vector3d IrI3(cos(q[1]) * (q[2] + l3x) * cos(q[0]) + sin(q[0]) * (l2y - l3y),
                             cos(q[0]) * (-l2y + l3y) + cos(q[1]) * sin(q[0]) * (q[2] + l3x),
                             (q[2] + l3x) * sin(q[1]) - l2z);
        Eigen::Vector3d IrI4(cos(q[1]) * (l3x + l4x + q[2]) * cos(q[0]) + sin(q[0]) * (l2y - l3y),
                             sin(q[0]) * (l3x + l4x + q[2]) * cos(q[1]) - cos(q[0]) * (l2y - l3y),
                             (l3x + l4x + q[2]) * sin(q[1]) - l2z);
        Eigen::Vector3d IrI6(((l6x * cos(q[4]) + q[2] + l3x + l4x) * cos(q[1]) + l6x * cos(q[3]) * sin(q[1]) * sin(q[4])) * cos(q[0]) - sin(q[0]) * (l6x * sin(q[3]) * sin(q[4]) - l2y + l3y),  // NOLINT
                             ((l6x * cos(q[4]) + q[2] + l3x + l4x) * cos(q[1]) + l6x * cos(q[3]) * sin(q[1]) * sin(q[4])) * sin(q[0]) + cos(q[0]) * (l6x * sin(q[3]) * sin(q[4]) - l2y + l3y),  // NOLINT
                             (l6x * cos(q[4]) + q[2] + l3x + l4x) * sin(q[1]) - cos(q[1]) * sin(q[4]) * cos(q[3]) * l6x - l2z);  // NOLINT
        Eigen::Vector3d IrI8(((sin(q[5]) * (l7x + l8x) * sin(q[4]) + ((l7x + l8x) * cos(q[5]) + l6x) * cos(q[4]) + l4x + q[2] + l3x) * cos(q[1]) + sin(q[1]) * (((l7x + l8x) * cos(q[5]) + l6x) * cos(q[3]) * sin(q[4]) - cos(q[3]) * sin(q[5]) * (l7x + l8x) * cos(q[4]) + l7y * sin(q[3]))) * cos(q[0]) - (sin(q[3]) * ((l7x + l8x) * cos(q[5]) + l6x) * sin(q[4]) - sin(q[3]) * sin(q[5]) * (l7x + l8x) * cos(q[4]) - l7y * cos(q[3]) - l2y + l3y) * sin(q[0]),  // NOLINT
                             ((sin(q[5]) * (l7x + l8x) * sin(q[4]) + ((l7x + l8x) * cos(q[5]) + l6x) * cos(q[4]) + l4x + q[2] + l3x) * cos(q[1]) + sin(q[1]) * (((l7x + l8x) * cos(q[5]) + l6x) * cos(q[3]) * sin(q[4]) - cos(q[3]) * sin(q[5]) * (l7x + l8x) * cos(q[4]) + l7y * sin(q[3]))) * sin(q[0]) + cos(q[0]) * (sin(q[3]) * ((l7x + l8x) * cos(q[5]) + l6x) * sin(q[4]) - sin(q[3]) * sin(q[5]) * (l7x + l8x) * cos(q[4]) - l7y * cos(q[3]) - l2y + l3y),  // NOLINT
                             (sin(q[5]) * (l7x + l8x) * sin(q[4]) + ((l7x + l8x) * cos(q[5]) + l6x) * cos(q[4]) + l4x + q[2] + l3x) * sin(q[1]) + ((sin(q[5]) * (l7x + l8x) * cos(q[4]) - ((l7x + l8x) * cos(q[5]) + l6x) * sin(q[4])) * cos(q[3]) - l7y * sin(q[3])) * cos(q[1]) - l2z);  // NOLINT
        Eigen::Vector3d IrI9((((-l9x * cos(q[5]) * cos(q[6]) * sin(q[7]) + sin(q[5]) * (l9x * cos(q[7]) + l7x + l8x)) * cos(q[1]) + cos(q[3]) * ((l9x * cos(q[7]) + l7x + l8x) * cos(q[5]) + l9x * cos(q[6]) * sin(q[5]) * sin(q[7]) + l6x) * sin(q[1])) * sin(q[4]) + (((l9x * cos(q[7]) + l7x + l8x) * cos(q[5]) + l9x * cos(q[6]) * sin(q[5]) * sin(q[7]) + l6x) * cos(q[1]) - (-l9x * cos(q[5]) * cos(q[6]) * sin(q[7]) + sin(q[5]) * (l9x * cos(q[7]) + l7x + l8x)) * cos(q[3]) * sin(q[1])) * cos(q[4]) + (l3x + l4x + q[2]) * cos(q[1]) + sin(q[1]) * sin(q[3]) * (l9x * sin(q[6]) * sin(q[7]) + l7y)) * cos(q[0]) - sin(q[0]) * (sin(q[3]) * ((l9x * cos(q[7]) + l7x + l8x) * cos(q[5]) + l9x * cos(q[6]) * sin(q[5]) * sin(q[7]) + l6x) * sin(q[4]) - (-l9x * cos(q[5]) * cos(q[6]) * sin(q[7]) + sin(q[5]) * (l9x * cos(q[7]) + l7x + l8x)) * sin(q[3]) * cos(q[4]) + (-l9x * sin(q[6]) * sin(q[7]) - l7y) * cos(q[3]) + l3y - l2y),  // NOLINT
                             (((-l9x * cos(q[5]) * cos(q[6]) * sin(q[7]) + sin(q[5]) * (l9x * cos(q[7]) + l7x + l8x)) * cos(q[1]) + cos(q[3]) * ((l9x * cos(q[7]) + l7x + l8x) * cos(q[5]) + l9x * cos(q[6]) * sin(q[5]) * sin(q[7]) + l6x) * sin(q[1])) * sin(q[4]) + (((l9x * cos(q[7]) + l7x + l8x) * cos(q[5]) + l9x * cos(q[6]) * sin(q[5]) * sin(q[7]) + l6x) * cos(q[1]) - (-l9x * cos(q[5]) * cos(q[6]) * sin(q[7]) + sin(q[5]) * (l9x * cos(q[7]) + l7x + l8x)) * cos(q[3]) * sin(q[1])) * cos(q[4]) + (l3x + l4x + q[2]) * cos(q[1]) + sin(q[1]) * sin(q[3]) * (l9x * sin(q[6]) * sin(q[7]) + l7y)) * sin(q[0]) + cos(q[0]) * (sin(q[3]) * ((l9x * cos(q[7]) + l7x + l8x) * cos(q[5]) + l9x * cos(q[6]) * sin(q[5]) * sin(q[7]) + l6x) * sin(q[4]) - (-l9x * cos(q[5]) * cos(q[6]) * sin(q[7]) + sin(q[5]) * (l9x * cos(q[7]) + l7x + l8x)) * sin(q[3]) * cos(q[4]) + (-l9x * sin(q[6]) * sin(q[7]) - l7y) * cos(q[3]) + l3y - l2y),  // NOLINT
                             (((l9x * cos(q[7]) + l7x + l8x) * cos(q[5]) + l9x * cos(q[6]) * sin(q[5]) * sin(q[7]) + l6x) * cos(q[4]) + (-l9x * cos(q[5]) * cos(q[6]) * sin(q[7]) + sin(q[5]) * (l9x * cos(q[7]) + l7x + l8x)) * sin(q[4]) + l4x + q[2] + l3x) * sin(q[1]) + (((-l9x * cos(q[5]) * cos(q[6]) * sin(q[7]) + sin(q[5]) * (l9x * cos(q[7]) + l7x + l8x)) * cos(q[4]) - sin(q[4]) * ((l9x * cos(q[7]) + l7x + l8x) * cos(q[5]) + l9x * cos(q[6]) * sin(q[5]) * sin(q[7]) + l6x)) * cos(q[3]) - sin(q[3]) * (l9x * sin(q[6]) * sin(q[7]) + l7y)) * cos(q[1]) - l2z);  // NOLINT
        k = 7;
        IrIx = Eigen::MatrixXd(3, k);
        IrIx << IrI2, IrIEndRail, IrI3, IrI4, IrI6, IrI8, IrI9;
        // Considering that there is a coll object between every point we have k-1 coll obj.
        // Since we also skip one (there is no object between IrIEndRail and IrI3) we have k-2 coll
        // obj.
        d = Eigen::VectorXd::Zero(k-2);
        d(0) = calcMinDistLink(IrIx.col(0), IrIx.col(1), obstacleCentre, lCollObj(0));
        for (unsigned int i = 2; i < k-1; i++) {  // skipping the object from endOfRail to point 3
            d(i-1) = calcMinDistLink(IrIx.col(i), IrIx.col(i+1), obstacleCentre, lCollObj(i-1));
        }
    }

    // Part common for 2DOF robot, 3DOF robot, and UR10e
    if (robot_ == 0 || robot_ == 1 || robot_ == 2) {
        // Considering that there is a coll object between every point we have k-1 coll obj.
        d = Eigen::VectorXd::Zero(k-1);
        for (unsigned int i = 0; i < k-1; i++) {
            d(i) = calcMinDistLink(IrIx.col(i), IrIx.col(i+1), obstacleCentre,
                lengthCollisionObject(i));
        }
    }

    // The size of d corresponds to the number of links that can collide with the collision object.
    // ATTENTION: It can be different from the number of degrees of freedom.
    return d;
}

}  // namespace crf::control::inversekinematics
