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

collision_detection::CollisionRobotDistanceField::CollisionRobotDistanceField(
    const robot_model::RobotModelConstPtr &kmodel,
    double padding,
    double scale)
  : CollisionRobot(kmodel, padding, scale)
{
  initSpheres();
}

collision_detection::CollisionRobotDistanceField::CollisionRobotDistanceField(
    const CollisionRobotDistanceField &other)
  : CollisionRobot(other)
{
  initSpheres();
}

void collision_detection::CollisionRobotDistanceField::checkSelfCollision(
    const CollisionRequest &req, 
    CollisionResult &res, 
    const robot_state::RobotState &state,
    const AllowedCollisionMatrix &acm) const
{
  logError("DistanceField collision checking not yet implemented");
  checkSelfCollisionUsingSpheres(state);
}

void collision_detection::CollisionRobotDistanceField::checkSelfCollision(
    const CollisionRequest &req, 
    CollisionResult &res, 
    const robot_state::RobotState &state1, 
    const robot_state::RobotState &state2, 
    const AllowedCollisionMatrix &acm) const
{
  logError("DistanceField continuous collision checking not yet implemented");
}

void collision_detection::CollisionRobotDistanceField::checkOtherCollision(
    const CollisionRequest &req, 
    CollisionResult &res, 
    const robot_state::RobotState &state,
    const CollisionRobot &other_robot, 
    const robot_state::RobotState &other_state,
    const AllowedCollisionMatrix &acm) const
{
  logError("DistanceField collision checking not yet implemented");
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
  logError("DistanceField continuous collision checking not yet implemented");
}

double collision_detection::CollisionRobotDistanceField::distanceSelf(
    const robot_state::RobotState &state,
    const AllowedCollisionMatrix &acm) const
{
  logError("DistanceField collision checking not yet implemented");
}

double collision_detection::CollisionRobotDistanceField::distanceOther(
    const robot_state::RobotState &state,
    const CollisionRobot &other_robot,
    const robot_state::RobotState &other_state,
    const AllowedCollisionMatrix &acm) const
{
  logError("DistanceField collision checking not yet implemented");
}

void collision_detection::CollisionRobotDistanceField::updatedPaddingOrScaling(
    const std::vector<std::string> &links)
{
}

void collision_detection::CollisionRobotDistanceField::checkSelfCollision(
    const CollisionRequest &req,
    CollisionResult &res,
    const robot_state::RobotState &state) const
{
  checkSelfCollision(req, res, state, empty_acm);
}

void collision_detection::CollisionRobotDistanceField::checkSelfCollision(
    const CollisionRequest &req, 
    CollisionResult &res, 
    const robot_state::RobotState &state1, 
    const robot_state::RobotState &state2) const
{
  checkSelfCollision(req, res, state1, state2, empty_acm);
}

void collision_detection::CollisionRobotDistanceField::checkOtherCollision(
    const CollisionRequest &req, 
    CollisionResult &res, 
    const robot_state::RobotState &state,
    const CollisionRobot &other_robot, 
    const robot_state::RobotState &other_state) const
{
  checkOtherCollision(req, res, state, other_robot, other_state, empty_acm);
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
  checkOtherCollision(req, res, state1, state2, other_robot, other_state1, other_state2, empty_acm);
}

double collision_detection::CollisionRobotDistanceField::distanceSelf(
    const robot_state::RobotState &state) const
{
  return distanceSelf(state, empty_acm);
}

double collision_detection::CollisionRobotDistanceField::distanceOther(
    const robot_state::RobotState &state,
    const CollisionRobot &other_robot,
    const robot_state::RobotState &other_state) const
{
  return distanceOther(state, other_robot, other_state, empty_acm);
}

