/*
 * Copyright (c) 2011, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author: Tobias Kunz <tobias@gatech.edu>
 * Date: 05/2012
 *
 * Humanoid Robotics Lab      Georgia Institute of Technology
 * Director: Mike Stilman     http://www.golems.org
 *
 * Algorithm details and publications:
 * http://www.golems.org/node/1570
 *
 * This file is provided under the following "BSD-style" License:
 *   Redistribution and use in source and binary forms, with or
 *   without modification, are permitted provided that the following
 *   conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 */
#define EIGEN_DONT_VECTORIZE
#define EIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT

#include "TimeOptimalTrajectoryGenerator/Path.hpp"
#include <fstream>
#include <iostream>
#include <vector>
#include <algorithm>
#include <cmath>
#include "eigen3/Eigen/Geometry"

using namespace std; //NOLINT
using namespace Eigen; //NOLINT


class LinearPathSegment : public PathSegment {
 public:
    LinearPathSegment(const Eigen::VectorXd &start, const Eigen::VectorXd &end) :
        start(start),
        end(end),
        PathSegment((end-start).norm()) {
        }

    Eigen::VectorXd getConfig(double s) const {
        s /= length;
        s = std::max(0.0, std::min(1.0, s));
        return (1.0 - s) * start + s * end;
    }

    Eigen::VectorXd getTangent(double /* s */) const {
        return (end - start) / length;
    }

    Eigen::VectorXd getCurvature(double /* s */) const {
        return Eigen::VectorXd::Zero(start.size());
    }

    list<double> getSwitchingPoints() const {
        return list<double>();
    }

    LinearPathSegment* clone() const {
        return new LinearPathSegment(*this);
    }

 private:
    Eigen::VectorXd start;
    Eigen::VectorXd end;
};


class CircularPathSegment : public PathSegment {
 public:
    CircularPathSegment(const Eigen::VectorXd &start,
        const Eigen::VectorXd &intersection,
        const Eigen::VectorXd &end,
        double maxDeviation) {
        if ((intersection - start).norm() < 0.000001 || (end - intersection).norm() < 0.000001) {
            length = 0.0;
            radius = 1.0;
            center = intersection;
            x = Eigen::VectorXd::Zero(start.size());
            y = Eigen::VectorXd::Zero(start.size());
            return;
        }

        const Eigen::VectorXd startDirection = (intersection - start).normalized();
        const Eigen::VectorXd endDirection = (end - intersection).normalized();

        if ((startDirection - endDirection).norm() < 0.000001) {
            length = 0.0;
            radius = 1.0;
            center = intersection;
            x = Eigen::VectorXd::Zero(start.size());
            y = Eigen::VectorXd::Zero(start.size());
            return;
        }

        const double startDistance = (start - intersection).norm();
        const double endDistance = (end - intersection).norm();

        double distance = std::min((start - intersection).norm(), (end - intersection).norm());
        const double angle = acos(startDirection.dot(endDirection));

        distance = std::min(distance,
            maxDeviation * sin(0.5 * angle) / (1.0 - cos(0.5 * angle)));  // enforce max deviation

        radius = distance / tan(0.5 * angle);
        length = angle * radius;

        center = intersection + (endDirection - startDirection).normalized() * radius / cos(0.5 * angle); //NOLINT

        if ((intersection - distance * startDirection - center).norm() < 0.000001) {
            length = 0.0;
            radius = 1.0;
            center = intersection;
            x = Eigen::VectorXd::Zero(start.size());
            y = Eigen::VectorXd::Zero(start.size());
            return;
        }
        x = (intersection - distance * startDirection - center).normalized();
        y = startDirection;
    }

    Eigen::VectorXd getConfig(double s) const {
        const double angle = s / radius;
        return center + radius * (x * cos(angle) + y * sin(angle));
    }

    Eigen::VectorXd getTangent(double s) const {
        const double angle = s / radius;
        return - x * sin(angle) + y * cos(angle);
    }

    Eigen::VectorXd getCurvature(double s) const {
        const double angle = s / radius;
        return - 1.0 / radius * (x * cos(angle) + y * sin(angle));
    }

    list<double> getSwitchingPoints() const {
        list<double> switchingPoints;
        const double dim = x.size();
        for (unsigned int i = 0; i < dim; i++) {
            double switchingAngle = atan2(y[i], x[i]);
            if (switchingAngle < 0.0) {
                switchingAngle += M_PI;
            }
            const double switchingPoint = switchingAngle * radius;
            if (switchingPoint < length) {
                switchingPoints.push_back(switchingPoint);
            }
        }
        switchingPoints.sort();
        return switchingPoints;
    }

    CircularPathSegment* clone() const {
        return new CircularPathSegment(*this);
    }

 private:
    double radius;
    Eigen::VectorXd center;
    Eigen::VectorXd x;
    Eigen::VectorXd y;
};



Path::Path(const list<VectorXd> &path, double maxDeviation) : length(0.0) {
    if (path.size() < 2)
        return;
    list<VectorXd>::const_iterator config1 = path.begin();
    list<VectorXd>::const_iterator config2 = config1;
    config2++;
    list<VectorXd>::const_iterator config3;
    VectorXd startConfig = *config1;

    while (config2 != path.end()) {
        config3 = config2;
        config3++;
        if (maxDeviation > 0.0 && config3 != path.end()) {
            CircularPathSegment* blendSegment = new CircularPathSegment(0.5 * (*config1 + *config2), *config2, 0.5 * (*config2 + *config3), maxDeviation); //NOLINT
            VectorXd endConfig = blendSegment->getConfig(0.0);
            if ((endConfig - startConfig).norm() > 0.000001) {
                pathSegments.push_back(new LinearPathSegment(startConfig, endConfig));
            }
            pathSegments.push_back(blendSegment);
            startConfig = blendSegment->getConfig(blendSegment->getLength());
        } else {
            pathSegments.push_back(new LinearPathSegment(startConfig, *config2));
            startConfig = *config2;
        }
        config1 = config2;
        config2++;
    }


    // create list of switching point candidates, calculate total path length and absolute positions of path segments //NOLINT
    for (list<PathSegment*>::iterator segment = pathSegments.begin(); segment != pathSegments.end(); segment++) { //NOLINT
        (*segment)->position = length;
        list<double> localSwitchingPoints = (*segment)->getSwitchingPoints();
        for (list<double>::const_iterator point = localSwitchingPoints.begin(); point != localSwitchingPoints.end(); point++) { //NOLINT
            switchingPoints.push_back(make_pair(length + *point, false));
        }
        length += (*segment)->getLength();
        while (!switchingPoints.empty() && switchingPoints.back().first >= length)
            switchingPoints.pop_back();
        switchingPoints.push_back(make_pair(length, true));
    }
    switchingPoints.pop_back();
}

Path::Path(const Path &path) :
    length(path.length),
    switchingPoints(path.switchingPoints) {
    for (list<PathSegment*>::const_iterator it = path.pathSegments.begin(); it != path.pathSegments.end(); it++) { //NOLINT
        pathSegments.push_back((*it)->clone());
    }
}

Path::~Path() {
    for (list<PathSegment*>::iterator it = pathSegments.begin(); it != pathSegments.end(); it++) {
        delete *it;
    }
}

double Path::getLength() const {
    return length;
}

PathSegment* Path::getPathSegment(double &s) const {
    list<PathSegment*>::const_iterator it = pathSegments.begin();
    list<PathSegment*>::const_iterator next = it;
    next++;
    while (next != pathSegments.end() && s >= (*next)->position) {
        it = next;
        next++;
    }
    s -= (*it)->position;
    return *it;
}

VectorXd Path::getConfig(double s) const {
    const PathSegment* pathSegment = getPathSegment(s);
    return pathSegment->getConfig(s);
}

VectorXd Path::getTangent(double s) const {
    const PathSegment* pathSegment = getPathSegment(s);
    return pathSegment->getTangent(s);
}

VectorXd Path::getCurvature(double s) const {
    const PathSegment* pathSegment = getPathSegment(s);
    return pathSegment->getCurvature(s);
}

double Path::getNextSwitchingPoint(double s, bool &discontinuity) const {
    list<pair<double, bool> >::const_iterator it = switchingPoints.begin();
    while (it != switchingPoints.end() && it->first <= s) {
        it++;
    }
    if (it == switchingPoints.end()) {
        discontinuity = true;
        return length;
    } else {
        discontinuity = it->second;
        return it->first;
    }
}

list<pair<double, bool> > Path::getSwitchingPoints() const { //NOLINT
    return switchingPoints;
}
