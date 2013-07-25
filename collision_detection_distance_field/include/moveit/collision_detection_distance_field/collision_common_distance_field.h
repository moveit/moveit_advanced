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

#ifndef MOVEIT_COLLISION_DETECTION_DISTANCE_FIELD__COLLISION_COMMON_DISTANCE_FIELD
#define MOVEIT_COLLISION_DETECTION_DISTANCE_FIELD__COLLISION_COMMON_DISTANCE_FIELD

#include <moveit/collision_detection/world.h>
#include <moveit/collision_detection/collision_world.h>

namespace collision_detection
{


class StaticDistanceField;

// information about a contact detected by distance field
struct DFContact : public Contact
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  // clear all fields, then copy Contact part from <contact>
  void copyFrom(const Contact& contact);

  // clear all fields
  void clear();


  // sphere_centers are in planning frame coordinate system
  double sphere_radius_1;             // 0 if no sphere
  Eigen::Vector3d sphere_center_1;    // valid if sphere_radius_1 != 0
  const StaticDistanceField *sdf_1;   // valid if non-NULL

  double sphere_radius_2;             // 0 if no sphere
  Eigen::Vector3d sphere_center_2;    // valid if sphere_radius_1 != 0
  const StaticDistanceField *sdf_2;   // valid if non-NULL

  bool eliminated_by_acm_function;
};


/** \brief Representation of a collision checking request for distance field collision detection. */
struct CollisionDistanceFieldRequest : public CollisionRequest
{
  CollisionDistanceFieldRequest()
    : use_sphere_sphere_for_self_collision(false)
  {}
  CollisionDistanceFieldRequest(const CollisionRequest& req);

  /** \brief If true, self collision checking is done with sphere-sphere checks (not with distance fields).
   * This will be slightly slower and the results will be different. */
  bool use_sphere_sphere_for_self_collision;
};

extern const AllowedCollisionMatrix empty_acm;

}

#endif
