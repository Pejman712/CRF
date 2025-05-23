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

#pragma once

#define EIGEN_DONT_VECTORIZE
#define EIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT

#include <list>
#include "eigen3/Eigen/Core"

class PathSegment {
 public:
    PathSegment(double length = 0.0) : //NOLINT
        length(length) {}

    virtual ~PathSegment() {}

    double getLength() const {
        return length;
    }
    virtual Eigen::VectorXd getConfig(double s) const = 0;
    virtual Eigen::VectorXd getTangent(double s) const = 0;
    virtual Eigen::VectorXd getCurvature(double s) const = 0;
    virtual std::list<double> getSwitchingPoints() const = 0;
    virtual PathSegment* clone() const = 0;

    double position;
 protected:
    double length;
};



class Path {
 public:
    Path(const std::list<Eigen::VectorXd> &path, double maxDeviation = 0.0); //NOLINT
    Path(const Path &path);
    ~Path();
    double getLength() const;
    Eigen::VectorXd getConfig(double s) const;
    Eigen::VectorXd getTangent(double s) const;
    Eigen::VectorXd getCurvature(double s) const;
    double getNextSwitchingPoint(double s, bool &discontinuity) const; //NOLINT
    std::list<std::pair<double, bool> > getSwitchingPoints() const;
 private:
    PathSegment* getPathSegment(double &s) const; //NOLINT
    double length;
    std::list<std::pair<double, bool> > switchingPoints; //NOLINT
    std::list<PathSegment*> pathSegments;
};
