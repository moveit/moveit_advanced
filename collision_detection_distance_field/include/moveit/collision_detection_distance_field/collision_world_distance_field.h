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

#ifndef MOVEIT_COLLISION_DETECTION_DISTANCE_FIELD_COLLISION_WORLD_DISTANCE_FIEL
#define MOVEIT_COLLISION_DETECTION_DISTANCE_FIELD_COLLISION_WORLD_DISTANCE_FIEL

#include <moveit/collision_detection_distance_field/collision_robot_distance_field.h>

namespace collision_detection
{
  
class CollisionWorldDistanceField : public CollisionWorld
{
public:
  
  CollisionWorldDistanceField();
  explicit CollisionWorldDistanceField(const WorldPtr& world);
  CollisionWorldDistanceField(const CollisionWorldDistanceField &other,
                              const WorldPtr& world);
  virtual ~CollisionWorldDistanceField();

  
  virtual void checkRobotCollision(const CollisionRequest &req,
                                   CollisionResult &res,
                                   const CollisionRobot &robot,
                                   const robot_state::RobotState &state) const;
  virtual void checkRobotCollision(const CollisionRequest &req,
                                   CollisionResult &res,
                                   const CollisionRobot &robot,
                                   const robot_state::RobotState &state,
                                   const AllowedCollisionMatrix &acm) const;
  virtual void checkRobotCollision(const CollisionRequest &req,
                                   CollisionResult &res,
                                   const CollisionRobot &robot,
                                   const robot_state::RobotState &state1,
                                   const robot_state::RobotState &state2) const;
  virtual void checkRobotCollision(const CollisionRequest &req,
                                   CollisionResult &res,
                                   const CollisionRobot &robot,
                                   const robot_state::RobotState &state1,
                                   const robot_state::RobotState &state2,
                                   const AllowedCollisionMatrix &acm) const;
  virtual void checkWorldCollision(const CollisionRequest &req,
                                   CollisionResult &res,
                                   const CollisionWorld &other_world) const;
  virtual void checkWorldCollision(const CollisionRequest &req,
                                   CollisionResult &res,
                                   const CollisionWorld &other_world,
                                   const AllowedCollisionMatrix &acm) const;

  virtual double distanceRobot(const CollisionRobot &robot,
                               const robot_state::RobotState &state) const;  
  virtual double distanceRobot(const CollisionRobot &robot,
                               const robot_state::RobotState &state,
                               const AllowedCollisionMatrix &acm) const;
  virtual double distanceWorld(const CollisionWorld &world) const;
  virtual double distanceWorld(const CollisionWorld &world,
                               const AllowedCollisionMatrix &acm) const;
  
  virtual void setWorld(const WorldPtr& world);
  
  
protected:
  void notifyObjectChange(const ObjectConstPtr& obj, World::Action action);
  World::ObserverHandle observer_handle_;
};
  
}

#endif
