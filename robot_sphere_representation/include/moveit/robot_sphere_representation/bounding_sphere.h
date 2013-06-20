/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Acorn Pooley */

#ifndef MOVEIT_ROBOT_SPHERE_REPRESENTATION__BOUNDING_SPHERE
#define MOVEIT_ROBOT_SPHERE_REPRESENTATION__BOUNDING_SPHERE

#include <eigen_stl_containers/eigen_stl_containers.h>

namespace robot_sphere_representation
{
  // generate a sphere that tightly bounds all the points.
  void generateBoundingSphere(
        const EigenSTL::vector_Vector3d& points,
        Eigen::Vector3d& center,
        double &radius);

  // generate a sphere whose diameter is the 2 points
  void robot_sphere_representation::findSphereTouching2Points(
        Eigen::Vector3d& center,
        double& radius,
        const Eigen::Vector3d& a,
        const Eigen::Vector3d& b);

  // Generate a sphere touching 3 points.  The center of the sphere will be on
  // the same plane as the 3 points.  If the points are colinear then one of
  // the 3 input points may be inside the sphere and/or the radius may be very large.
  void robot_sphere_representation::findSphereTouching3Points(
        Eigen::Vector3d& center,
        double& radius,
        const Eigen::Vector3d& a,
        const Eigen::Vector3d& b,
        const Eigen::Vector3d& c);

  // Generate the unique sphere that touches the 4 points.
  // If the 4 points are coplanar then the sphere may contain some of the input
  // points and/or the radius may be very large.
  void robot_sphere_representation::findSphereTouching3Points(
  void robot_sphere_representation::findSphereTouching4Points(
        Eigen::Vector3d& center,
        double& radius,
        const Eigen::Vector3d& a,
        const Eigen::Vector3d& b,
        const Eigen::Vector3d& c,
        const Eigen::Vector3d& d);

}


#endif

