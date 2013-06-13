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
#include <algorithm>

inline void collision_detection::CollisionRobotDistanceField::initQuery(
    WorkArea& work,
    const char *descrip,
    const CollisionRequest *req,
    CollisionResult *res,
    const robot_state::RobotState *state1,
    const robot_state::RobotState *state2,
    const CollisionRobot *other_robot,
    const robot_state::RobotState *other_state1,
    const robot_state::RobotState *other_state2,
    const AllowedCollisionMatrix *acm) const
{
  work.req_ = req;
  work.res_ = res;
  work.state1_ = state1;
  work.state2_ = state2;
  work.other_robot_ = other_robot;
  work.other_state1_ = other_state1;
  work.other_state2_ = other_state2;
  work.acm_ = acm;

  // debug
  if (1 || work.req_->verbose)
    dumpQuery(work, descrip);
}

inline void collision_detection::CollisionRobotDistanceField::setCloseDistance(WorkArea& work, double distance) const
{
  work.res_->distance = std::min(work.res_->distance, distance);
}

inline bool collision_detection::CollisionRobotDistanceField::never_check_link_pair(const DFLink *link_a, const DFLink *link_b) const
{
  return link_a->acm_bits_.getBit(link_b->index_in_link_order_);
}



#endif
