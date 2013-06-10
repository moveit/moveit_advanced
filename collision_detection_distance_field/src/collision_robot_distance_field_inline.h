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

#ifndef MOVEIT_COLLISION_DETECTION_DISTANCE_FIELD_COLLISION_ROBOT_DISTANCE_FIELD_INLINE
#define MOVEIT_COLLISION_DETECTION_DISTANCE_FIELD_COLLISION_ROBOT_DISTANCE_FIELD_INLINE

#include <moveit/collision_detection_distance_field/collision_robot_distance_field.h>

inline void collision_detection::CollisionRobotDistanceField::WorkArea::initQuery(
    const char *descrip,
    const CollisionRequest *req,
    CollisionResult *res,
    const robot_state::RobotState *state1,
    const robot_state::RobotState *state2,
    const CollisionRobot *other_robot,
    const robot_state::RobotState *other_state1,
    const robot_state::RobotState *other_state2,
    const AllowedCollisionMatrix *acm)
{
  req_ = req;
  res_ = res;
  state1_ = state1;
  state2_ = state2;
  other_robot_ = other_robot;
  other_state1_ = other_state1;
  other_state2_ = other_state2;
  acm_ = acm;

  // debug
  if (1)
  {
    logInform("CollisionRobotDistanceField Query %s", descrip);

    std::stringstream ss_cost;
    ss_cost << ", cost(max=" << req->max_cost_sources << ", dens=" << req->min_cost_density << ")";
    std::stringstream ss_contacts;
    ss_contacts << ", contact(max=" << req->max_contacts << ", cpp=" << req->max_contacts_per_pair << ")";
    std::stringstream ss_acm;
    if (acm)
    {
      std::vector<std::string> names;
      acm->getAllEntryNames(names);
      ss_acm << ", acm(nnames=" << names.size() << ", sz=" << acm->getSize() << ")";
    }
    logInform("   request: result%s%s%s%s%s%s",
      req->distance ? ", distance" : "",
      req->cost ? ss_cost.str().c_str() : "",
      req->contacts ? ss_contacts.str().c_str() : "",
      req->is_done ? ", is_done-func" : "",
      req->verbose ? ", VERBOSE" : "",
      ss_acm.str().c_str());
  }
}


#endif
