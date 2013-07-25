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

#include <moveit/collision_detection_distance_field/collision_robot_distance_field.h>
#include "collision_robot_distance_field_inline.h"

inline void collision_detection::CollisionRobotDistanceField::checkSelfCollision(
    WorkArea& work) const
{
  if (work.use_sphere_sphere_for_self_collision_)
  {
    checkSelfCollisionUsingSpheres(work);
  }
  else
  {
    checkSelfCollisionUsingIntraDF(work);
  }
}

void collision_detection::CollisionRobotDistanceField::checkSelfCollision(
    const CollisionRequest &req, 
    CollisionResult &res, 
    const robot_state::RobotState &state,
    const AllowedCollisionMatrix &acm) const
{
  WorkArea& work = getWorkArea();
  initQuery(work, "checkSelfCollision1acm", &req, &res, &state, NULL, NULL, NULL, NULL, &acm);

  checkSelfCollision(work);
}

void collision_detection::CollisionRobotDistanceField::checkSelfCollision(
    const CollisionRequest &req,
    CollisionResult &res,
    const robot_state::RobotState &state) const
{
  WorkArea& work = getWorkArea();
  initQuery(work, "checkSelfCollision1", &req, &res, &state, NULL, NULL, NULL, NULL, NULL);

  checkSelfCollision(work);
}

void collision_detection::CollisionRobotDistanceField::checkSelfCollision(
    const CollisionRequest &req, 
    CollisionResult &res, 
    const robot_state::RobotState &state1, 
    const robot_state::RobotState &state2, 
    const AllowedCollisionMatrix &acm) const
{
  logError("CollisionRobotDistanceField::checkSelfCollision 2a checking not yet implemented");
}

void collision_detection::CollisionRobotDistanceField::checkSelfCollision(
    const CollisionRequest &req, 
    CollisionResult &res, 
    const robot_state::RobotState &state1, 
    const robot_state::RobotState &state2) const
{
  logError("CollisionRobotDistanceField::checkSelfCollision 2b checking not yet implemented");
}

void collision_detection::CollisionRobotDistanceField::checkOtherCollision(
    const CollisionRequest &req, 
    CollisionResult &res, 
    const robot_state::RobotState &state,
    const CollisionRobot &other_robot, 
    const robot_state::RobotState &other_state,
    const AllowedCollisionMatrix &acm) const
{
  logError("CollisionRobotDistanceField::checkSelfCollision 3a not yet implemented");
}

void collision_detection::CollisionRobotDistanceField::checkOtherCollision(
    const CollisionRequest &req, 
    CollisionResult &res, 
    const robot_state::RobotState &state,
    const CollisionRobot &other_robot, 
    const robot_state::RobotState &other_state) const
{
  logError("CollisionRobotDistanceField::checkSelfCollision 3b not yet implemented");
}

void collision_detection::CollisionRobotDistanceField::checkOtherCollision(
    const CollisionRequest &req, 
    CollisionResult &res, 
    const robot_state::RobotState &state1, 
    const robot_state::RobotState &state2,
    const CollisionRobot &other_robot, 
    const robot_state::RobotState &other_state1, 
    const robot_state::RobotState &other_state2,
    const AllowedCollisionMatrix &acm) const
{
  logError("CollisionRobotDistanceField::checkSelfCollision 4a checking not yet implemented");
}

void collision_detection::CollisionRobotDistanceField::checkOtherCollision(
    const CollisionRequest &req, 
    CollisionResult &res, 
    const robot_state::RobotState &state1, 
    const robot_state::RobotState &state2,
    const CollisionRobot &other_robot, 
    const robot_state::RobotState &other_state1, 
    const robot_state::RobotState &other_state2) const
{
  logError("CollisionRobotDistanceField::checkSelfCollision 4b checking not yet implemented");
}

double collision_detection::CollisionRobotDistanceField::distanceSelf(
    const robot_state::RobotState &state,
    const AllowedCollisionMatrix &acm) const
{
  WorkArea& work = getWorkArea();
  CollisionResult res;
  CollisionRequest req;
  req.distance = true;
  initQuery(work, "distanceSelf1acm", &req, &res, &state, NULL, NULL, NULL, NULL, &acm);

  checkSelfCollision(work);

  return res.distance;
}

double collision_detection::CollisionRobotDistanceField::distanceSelf(
    const robot_state::RobotState &state) const
{
  WorkArea& work = getWorkArea();
  CollisionResult res;
  CollisionRequest req;
  req.distance = true;
  initQuery(work, "distanceSelf1acm", &req, &res, &state, NULL, NULL, NULL, NULL, NULL);

  checkSelfCollision(work);

  return res.distance;
}

double collision_detection::CollisionRobotDistanceField::distanceOther(
    const robot_state::RobotState &state,
    const CollisionRobot &other_robot,
    const robot_state::RobotState &other_state,
    const AllowedCollisionMatrix &acm) const
{
  logError("CollisionRobotDistanceField::distanceSelf 2a not yet implemented");
  return 0.0;
}

double collision_detection::CollisionRobotDistanceField::distanceOther(
    const robot_state::RobotState &state,
    const CollisionRobot &other_robot,
    const robot_state::RobotState &other_state) const
{
  logError("CollisionRobotDistanceField::distanceSelf 2b not yet implemented");
  return 0.0;
}

collision_detection::CollisionDistanceFieldRequest::CollisionDistanceFieldRequest(
      const CollisionRequest& req)
    : CollisionRequest(req)
    , use_sphere_sphere_for_self_collision(false)
{
  const CollisionDistanceFieldRequest *dfreq = dynamic_cast<const CollisionDistanceFieldRequest*>(&req);
  if (dfreq)
  {
    use_sphere_sphere_for_self_collision = dfreq->use_sphere_sphere_for_self_collision;
  }
}


void collision_detection::CollisionRobotDistanceField::getSelfCollisionContacts(
    const CollisionRequest &req, 
    CollisionResult &res,
    const robot_state::RobotState &state, 
    const AllowedCollisionMatrix *acm,
    std::vector<DFContact>* df_contacts,
    DFContact *df_distance) const
{
  CollisionDistanceFieldRequest req2(req);
  WorkArea& work = getWorkArea();
  initQuery(work, "getSelfCollisionContacts", &req2, &res, &state, NULL, NULL, NULL, NULL, acm);

  // force generation of contacts
  if (df_contacts && !req2.contacts)
  {
    req2.contacts = true;
    req2.max_contacts = 1000;
    req2.max_contacts_per_pair = 100;
  }

  // force generation of contacts
  if (df_distance)
  {
    req2.distance = true;
    if (!req2.contacts)
    {
      req2.max_contacts = 0;
      req2.max_contacts_per_pair = 0;
    }
    req2.contacts = true;
    df_distance->clear();
  }

  work.df_contacts_ = df_contacts;
  work.df_distance_ = df_distance;
  checkSelfCollision(work);
  work.df_contacts_ = NULL;
  work.df_distance_ = NULL;
}


