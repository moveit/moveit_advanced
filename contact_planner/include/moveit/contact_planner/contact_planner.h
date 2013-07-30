/*
 * Copyright (c) 2013, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/* Author: Sachin Chitta */

#ifndef MOVEIT_CONTACT_PLANNER_
#define MOVEIT_CONTACT_PLANNER_

#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/joint_state_group.h>

#include <moveit/collision_detection_distance_field/collision_detector_allocator_distance_field.h>
#include <moveit/collision_detection/collision_common.h>

namespace moveit
{

namespace contact_planner
{

class ContactPlanner
{

public:

  ContactPlanner(unsigned int max_iterations = 500,
                 double delta_t = 0.01);
  
  bool moveOutOfContact(const planning_scene::PlanningScene &planning_scene,
                        const robot_state::RobotState &robot_state,
                        const std::string &group_name,
                        robot_trajectory::RobotTrajectory &trajectory) const;  
  
  bool moveIntoContact(const planning_scene::PlanningScene &planning_scene,
                       const robot_state::RobotState &robot_state,
                       const std::string &group_name,
                       robot_trajectory::RobotTrajectory &trajectory) const;
  
private:

  const collision_detection::CollisionRobotDistanceField* getCollisionRobotDistanceField(const planning_scene::PlanningScene &planning_scene) const;    
  unsigned int max_iterations_;
  double delta_t_;    
};
 
}

}

#endif
