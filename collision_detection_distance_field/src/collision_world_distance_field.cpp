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

/* Author Acoern Pooley */

#include <moveit/collision_detection_distance_field/collision_world_distance_field.h>
#include <boost/bind.hpp>

collision_detection::CollisionWorldDistanceField::CollisionWorldDistanceField()
  : CollisionWorld()
{
  // request notifications about changes to world
  observer_handle_ = getWorld()->addObserver(boost::bind(&CollisionWorldDistanceField::notifyObjectChange, this, _1, _2));
}

collision_detection::CollisionWorldDistanceField::CollisionWorldDistanceField(
    const WorldPtr& world)
  : CollisionWorld(world)
{
  // request notifications about changes to world
  observer_handle_ = getWorld()->addObserver(boost::bind(&CollisionWorldDistanceField::notifyObjectChange, this, _1, _2));
  getWorld()->notifyObserverAllObjects(observer_handle_, World::CREATE);
}

collision_detection::CollisionWorldDistanceField::CollisionWorldDistanceField(
    const CollisionWorldDistanceField &other, 
    const WorldPtr& world)
  : CollisionWorld(other, world)
{
  // request notifications about changes to world
  observer_handle_ = getWorld()->addObserver(boost::bind(&CollisionWorldDistanceField::notifyObjectChange, this, _1, _2));
}

collision_detection::CollisionWorldDistanceField::~CollisionWorldDistanceField()
{
  getWorld()->removeObserver(observer_handle_);
}

void collision_detection::CollisionWorldDistanceField::checkRobotCollision(
    const CollisionRequest &req, 
    CollisionResult &res, 
    const CollisionRobot &robot, 
    const robot_state::RobotState &state, 
    const AllowedCollisionMatrix &acm) const
{
  logError("CollisionWorldDistanceField::checkRobotCollision 1 not yet implemented");
}

void collision_detection::CollisionWorldDistanceField::checkRobotCollision(
    const CollisionRequest &req, 
    CollisionResult &res, 
    const CollisionRobot &robot, 
    const robot_state::RobotState &state1, 
    const robot_state::RobotState &state2, 
    const AllowedCollisionMatrix &acm) const
{
  logError("CollisionWorldDistanceField::checkRobotCollision 2 checking not yet implemented");
}

void collision_detection::CollisionWorldDistanceField::checkWorldCollision(
    const CollisionRequest &req, 
    CollisionResult &res, 
    const CollisionWorld &other_world, 
    const AllowedCollisionMatrix &acm) const
{
  logError("CollisionWorldDistanceField::checkWorldCollision 1 not yet implemented");
}

void collision_detection::CollisionWorldDistanceField::setWorld(
    const WorldPtr& world)
{
  if (world == getWorld())
    return;

  // turn off notifications about old world
  getWorld()->removeObserver(observer_handle_);

  // request notifications about changes to new world
  observer_handle_ = getWorld()->addObserver(boost::bind(&CollisionWorldDistanceField::notifyObjectChange, this, _1, _2));

  // get notifications any objects already in the new world
  getWorld()->notifyObserverAllObjects(observer_handle_, World::CREATE);
}

void collision_detection::CollisionWorldDistanceField::notifyObjectChange(
    const ObjectConstPtr& obj, 
    World::Action action)
{
}

double collision_detection::CollisionWorldDistanceField::distanceRobot(
    const CollisionRobot &robot, 
    const robot_state::RobotState &state, 
    const AllowedCollisionMatrix &acm) const
{
  logError("CollisionWorldDistanceField::distanceRobot 1 not yet implemented");
}

double collision_detection::CollisionWorldDistanceField::distanceWorld(
    const CollisionWorld &world, 
    const AllowedCollisionMatrix &acm) const
{
  logError("CollisionWorldDistanceField::distanceWorld 2 not yet implemented");
}

void collision_detection::CollisionWorldDistanceField::checkRobotCollision(
    const CollisionRequest &req, 
    CollisionResult &res, 
    const CollisionRobot &robot, 
    const robot_state::RobotState &state) const
{
  checkRobotCollision(req, res, robot, state, empty_acm);
}

void collision_detection::CollisionWorldDistanceField::checkRobotCollision(
    const CollisionRequest &req, 
    CollisionResult &res, 
    const CollisionRobot &robot, 
    const robot_state::RobotState &state1, 
    const robot_state::RobotState &state2) const
{
  checkRobotCollision(req, res, robot, state1, state2, empty_acm);
}

void collision_detection::CollisionWorldDistanceField::checkWorldCollision(
    const CollisionRequest &req, 
    CollisionResult &res, 
    const CollisionWorld &other_world) const
{
  checkWorldCollision(req, res, other_world, empty_acm);
}

double collision_detection::CollisionWorldDistanceField::distanceRobot(
    const CollisionRobot &robot, 
    const robot_state::RobotState &state) const
{
  return distanceRobot(robot, state, empty_acm);
}

double collision_detection::CollisionWorldDistanceField::distanceWorld(
    const CollisionWorld &world) const
{
  return distanceWorld(world, empty_acm);
}

#include <moveit/collision_detection_distance_field/collision_detector_allocator_distance_field.h>
const std::string collision_detection::CollisionDetectorAllocatorDistanceField::NAME_("DistanceField");

