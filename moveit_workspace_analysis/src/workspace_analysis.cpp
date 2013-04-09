/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2012, Willow Garage, Inc.
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
*   * Neither the name of Willow Garage, Inc. nor the names of its
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
*
* Author: Sachin Chitta
*********************************************************************/

#include <moveit/workspace_analysis/workspace_analysis.h>
#include <fstream>
#include <iostream>

namespace workspace_analysis
{

WorkspaceAnalysis::WorkspaceAnalysis(const planning_scene::PlanningSceneConstPtr &planning_scene,
                                     bool position_only,
                                     double joint_limits_penalty_multiplier): planning_scene_(planning_scene), position_only_ik_(position_only)
{
  state_validity_callback_fn_ = boost::bind(&WorkspaceAnalysis::isIKSolutionCollisionFree, this, _1, _2);  
  kinematics_metrics_.reset(new kinematics_metrics::KinematicsMetrics(planning_scene->getCurrentState().getRobotModel()));
  kinematics_metrics_->setPenaltyMultiplier(joint_limits_penalty_multiplier);  
}

bool WorkspaceAnalysis::isIKSolutionCollisionFree(robot_state::JointStateGroup *joint_state_group,
                                                  const std::vector<double> &ik_solution)
{
  joint_state_group->setVariableValues(ik_solution);
  bool result = !planning_scene_->isStateColliding(*joint_state_group->getRobotState(), joint_state_group->getName());
  return result;  
}

std::vector<geometry_msgs::Pose> WorkspaceAnalysis::sampleUniform(const moveit_msgs::WorkspaceParameters &workspace, 
                                                                  const std::vector<geometry_msgs::Quaternion> &orientations,
                                                                  double x_resolution,
                                                                  double y_resolution,
                                                                  double z_resolution) const
{
  std::vector<geometry_msgs::Pose> results;
  std::vector<geometry_msgs::Quaternion> rotations = orientations;  
  if(orientations.empty())
  {
    geometry_msgs::Quaternion quaternion;
    quaternion.w = 1.0;    
    rotations.push_back(quaternion);    
  }
  double x_min = workspace.min_corner.x;
  double y_min = workspace.min_corner.y;
  double z_min = workspace.min_corner.z;

  unsigned int x_num_points,y_num_points,z_num_points;
  double x_dim = std::fabs(workspace.min_corner.x - workspace.max_corner.x);
  double y_dim = std::fabs(workspace.min_corner.y - workspace.max_corner.y);
  double z_dim = std::fabs(workspace.min_corner.z - workspace.max_corner.z);

  x_num_points = (unsigned int) (x_dim/x_resolution) + 1;
  y_num_points = (unsigned int) (y_dim/y_resolution) + 1;
  z_num_points = (unsigned int) (z_dim/z_resolution) + 1;

  ROS_DEBUG("Cache dimension (num grid points) in (x,y,z): %d %d %d",x_num_points,y_num_points,z_num_points);
  geometry_msgs::Pose pose;  
  for(std::size_t i=0; i < x_num_points; ++i)
  {
    pose.position.x = x_min + i * x_resolution;
    for(std::size_t j=0; j < y_num_points; ++j)
    {
      pose.position.y = y_min + j * y_resolution;
      for(std::size_t k=0; k < z_num_points; ++k)
      {
        pose.position.z = z_min + k * z_resolution;
        for(std::size_t m=0; m < rotations.size(); ++m)
        {
          pose.orientation = rotations[m];
          //          Eigen::Affine3d pose_eigen;
          //          tf::poseMsgToEigen(pose, pose_eigen);
          results.push_back(pose);          
        }
      }
    }
  }
  ROS_INFO("Generated %d samples for workspace points",(int) results.size());
  return results;
}

workspace_analysis::WorkspaceMetrics WorkspaceAnalysis::computeMetrics(const moveit_msgs::WorkspaceParameters &workspace,
                                                                       const std::vector<geometry_msgs::Quaternion> &orientations,
                                                                       robot_state::JointStateGroup *joint_state_group,
                                                                       double x_resolution,
                                                                       double y_resolution,
                                                                       double z_resolution) const
{
  workspace_analysis::WorkspaceMetrics metrics;
  if(!joint_state_group || !planning_scene_)
  {
    ROS_ERROR("Joint state group and planning scene should not be null");
    return metrics;
  } 

  std::vector<geometry_msgs::Pose> points = sampleUniform(workspace, orientations, x_resolution, y_resolution, z_resolution);
  //  metrics.joint_values_.resize(metrics.points_.size());
  //  metrics.manipulability_.resize(metrics.points_.size());
  metrics.group_name_ = joint_state_group->getName();
  metrics.robot_name_ = joint_state_group->getRobotState()->getRobotModel()->getName();
  metrics.frame_id_ =  joint_state_group->getRobotState()->getRobotModel()->getModelFrame();  
  
  for(std::size_t i=0; i < points.size(); ++i)
  {   
    bool found_ik = joint_state_group->setFromIK(points[i], 1, 0.01, state_validity_callback_fn_);
    if(found_ik)
    {
      ROS_DEBUG("Found IK: %d", (int) i);      
      double manipulability_index;      
      kinematics_metrics_->getManipulabilityIndex(*joint_state_group->getRobotState(), 
                                                  joint_state_group->getJointModelGroup(),
                                                  manipulability_index,
                                                  position_only_ik_);
      std::pair<double,int> distance = joint_state_group->getMinDistanceToBounds();      
      std::vector<double> joint_values;      
      joint_state_group->getVariableValues(joint_values);
      metrics.points_.push_back(points[i]);      
      metrics.joint_values_.push_back(joint_values);      
      metrics.manipulability_.push_back(manipulability_index);      
      metrics.min_distance_joint_limits_.push_back(distance.first);      
    }
    if(!ros::ok())
      return metrics;    
  }
  return metrics;  
}

bool WorkspaceMetrics::writeToFile(const std::string &filename, const std::string &delimiter, bool exclude_strings)
{
  ROS_DEBUG("Writing %d total points to file: %s",(int) points_.size(),filename.c_str());  
  std::ofstream file;
  file.open(filename.c_str());
  if(!file.is_open())
  {
    ROS_DEBUG("Could not open file: %s",filename.c_str());
    return false;    
  }
  if(file.good())
  {
    if(!exclude_strings)
    {
      file << robot_name_ << std::endl;
      file << group_name_ << std::endl;
      file << frame_id_ << std::endl;
    }    
    for(std::size_t i=0; i < points_.size(); ++i)
    {
      if(joint_values_[i].empty())
        continue;      
      file << points_[i].position.x << delimiter << points_[i].position.y << delimiter << points_[i].position.z << delimiter;
      file << points_[i].orientation.x << delimiter << points_[i].orientation.y  << delimiter << points_[i].orientation.z << delimiter << points_[i].orientation.w << delimiter;
      for(std::size_t j=0; j < joint_values_[i].size(); ++j)
        file << joint_values_[i][j] << delimiter;        
      file << manipulability_[i] << delimiter << min_distance_joint_limits_[i] << std::endl;      
    }
  }  
  file.close();
  return true;   
}

}
