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

#include <moveit/contact_planner/contact_planner.h>


namespace moveit
{

namespace contact_planner
{

std::string COLLISION_METHOD_STRING_DISTANCE_FIELD = "DistanceField";

ContactPlanner::ContactPlanner(unsigned int max_iterations, double delta_t): max_iterations_(max_iterations), delta_t_(delta_t)
{
  
}

bool ContactPlanner::moveOutOfContact(const planning_scene::PlanningScene &planning_scene,
                                      const robot_state::RobotState &robot_state,
                                      const std::string &group_name,
                                      robot_trajectory::RobotTrajectory &trajectory) const
{
  const collision_detection::CollisionRobotDistanceField *collision_robot = getCollisionRobotDistanceField(planning_scene);
  if(!collision_robot)
    return false;
  
  // collision_robot->setMaxDistance() - distance field will be propagated to max_distance + radius of largest sphere in robot
  // can call this multiple times with the same distance - will only be a no-op in that case (will only recompute if distance is 
  // increased.

  collision_detection::CollisionDistanceFieldRequest req;
  collision_detection::CollisionResult res;

  // req.use_sphere_sphere_for_self_collision = true;  // need to comment this out if also looking for distance
  req.contacts = true;
  //  req.distance = true;  
  req.max_contacts = 1000;
  req.max_contacts_per_pair = 100;
  req.group_name = group_name;
  
  // 1. Get all contacts
  // 2. Compute delta_q = -J*delta_x (where delta_x corresponds penetration) for each contact
  // 3. Add up all contributions
  // 4. Repeat until out of contact

  robot_state::RobotState local_state(robot_state); //make a copy because we will have to change it
  robot_state::JointStateGroup* group_state = local_state.getJointStateGroup(group_name);
  
  bool found_free_state = false;  
  
  for(unsigned int i=0; i < max_iterations_; ++i)
  {    
    collision_robot->checkSelfCollision(req, res, local_state, planning_scene.getAllowedCollisionMatrix());
    if(res.contacts.empty())
    {
      found_free_state = true;      
      logDebug("No contacts in state at iteration: %d", i);      
      break;
    }    
    // Get the contacts
    collision_detection::CollisionResult::ContactMap::const_iterator it = res.contacts.begin();
    collision_detection::CollisionResult::ContactMap::const_iterator end = res.contacts.end();

    Eigen::VectorXd delta_x = Eigen::VectorXd::Zero(6);
    Eigen::VectorXd delta_q = Eigen::VectorXd::Zero(group_state->getVariableCount());

    for ( ; it != end ; ++it)
    {
      std::vector<collision_detection::Contact>::const_iterator cit = it->second.begin();
      std::vector<collision_detection::Contact>::const_iterator cend = it->second.end();
      for ( ; cit != cend ; ++cit)
      {
        std::string link_name = cit->body_name_2;
        logDebug("Link name in contact: %s", link_name.c_str());        
        Eigen::MatrixXd jacobian;        
        Eigen::Affine3d reference_transform_inverse = local_state.getLinkState(link_name)->getGlobalLinkTransform().inverse();
        Eigen::Vector3d reference_point_position =  reference_transform_inverse * cit->pos;        
        if(!group_state->getJacobian(link_name, reference_point_position, jacobian))
        {
          logError("Could not find jacobian");          
          return false;        
        }        
        delta_x.block<3,1>(0,0) = cit->depth * cit->normal;
        delta_q = delta_q + jacobian.transpose() * delta_x;        
      }
    }
    group_state->integrateJointVelocity(delta_q, 1.0);    
    group_state->enforceBounds();    
    trajectory.addSuffixWayPoint(local_state, delta_t_);    
  }

  if(found_free_state)
    return true;
  else
    return false;  
  
  /*
    collision_detection::CollisionDistanceFieldRequest req;
    collision_detection::CollisionResult res;
    collision_detection::DFContact df_distance;
    req.use_sphere_sphere_for_self_collision = collision_df_use_spheres_->getBool();

//    This call gives you the deepest penetration (if robot is in self-collision) or the closest two points (if robot is not in self-collision)
    crobot_df->getSelfCollisionContacts(req, res, state, &ps->getAllowedCollisionMatrix(), NULL, &df_distance);

//    This call gives you all the contacts
    std::vector<collision_detection::DFContact> df_contacts;
    crobot_df->getSelfCollisionContacts(req, res, state, &ps->getAllowedCollisionMatrix(), &df_contacts);

  */
}


const collision_detection::CollisionRobotDistanceField* ContactPlanner::getCollisionRobotDistanceField(const planning_scene::PlanningScene &planning_scene) const
{  
  const collision_detection::CollisionRobot* crobot = planning_scene.getCollisionRobot(COLLISION_METHOD_STRING_DISTANCE_FIELD).get();
  const collision_detection::CollisionRobotDistanceField* crobot_df =
    dynamic_cast<const collision_detection::CollisionRobotDistanceField*>(crobot);

  if (!crobot_df)
  {
    logError("Could not find the CollisionRobotDistanceField instance. %s:%d");
  }
  return crobot_df;
}

bool ContactPlanner::moveIntoContact(const planning_scene::PlanningScene &planning_scene,
                                     const robot_state::RobotState &robot_state,
                                     const std::string &group_name,
                                     robot_trajectory::RobotTrajectory &trajectory) const
{
  return false;  
}

}

}
