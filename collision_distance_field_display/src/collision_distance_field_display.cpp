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
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
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

/* Author: Acorn Pooley */

#include <collision_distance_field_display/collision_distance_field_display.h>

#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

#include <tf/transform_listener.h>

#include <eigen_conversions/eigen_msg.h>

#include <rviz/visualization_manager.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/int_property.h>
#include <rviz/frame_manager.h>

#include <moveit/rviz_plugin_render_tools/robot_state_visualization.h>

static std_msgs::ColorRGBA ColorRGBA(rviz::ColorProperty* color_prop, rviz::FloatProperty* alpha_prop = NULL)
{
  std_msgs::ColorRGBA result;
  QColor qcolor = color_prop->getColor();
  result.r = qcolor.redF();
  result.g = qcolor.greenF();
  result.b = qcolor.blueF();
  result.a = alpha_prop ? alpha_prop->getFloat() : 1.0;
  return result;
}

moveit_rviz_plugin::CollisionDistanceFieldDisplay::CollisionDistanceFieldDisplay()
  : PlanningSceneDisplay()
  , robot_model_loaded_(false)
  , robot_visual_dirty_(true)
{
  show_robot_visual_property_ = new rviz::BoolProperty(
                                      "Show Robot Visual",
                                      false,
                                      "Show the robot visual appearance (what it looks like).",
                                      this,
                                      SLOT( robotAppearanceChanged() ));
  show_robot_collision_property_ = new rviz::BoolProperty(
                                      "Show Robot Collision",
                                      true,
                                      "Show the robot collision geometry.",
                                      this,
                                      SLOT( robotAppearanceChanged() ));
  publish_tf_property_ = new rviz::BoolProperty(
                                      "Publish Robot State on TF",
                                      true,
                                      "Enable publishing of robot link frames on TF (useful with TF display).",
                                      this);
  attached_object_color_property_ = new rviz::ColorProperty(
                                      "Attached Object Color",
                                      QColor( 204, 51, 204 ),
                                      "Color to draw objects attached to the robot.",
                                      this,
                                      SLOT( robotAppearanceChanged() ));
  robot_alpha_property_ = new rviz::FloatProperty(
                                      "Robot Alpha",
                                      1.0,
                                      "0 is fully transparent, 1.0 is fully opaque.",
                                      this,
                                      SLOT( robotAppearanceChanged() ));

  // turn Scene Robot visual off by default
  if (scene_robot_enabled_property_)
    scene_robot_enabled_property_->setValue(false);
}

moveit_rviz_plugin::CollisionDistanceFieldDisplay::~CollisionDistanceFieldDisplay()
{
  background_process_.setJobUpdateEvent(BackgroundProcessing::JobUpdateCallback());
  clearJobs();

  robot_visual_.reset();
}

void moveit_rviz_plugin::CollisionDistanceFieldDisplay::onInitialize()
{
  PlanningSceneDisplay::onInitialize();

  robot_visual_.reset(new RobotStateVisualization(planning_scene_node_, context_, "Robot", NULL));
  robotAppearanceChanged();
}

void moveit_rviz_plugin::CollisionDistanceFieldDisplay::onDisable()
{
  robot_visual_->setVisible(false);
  PlanningSceneDisplay::onDisable();
}

void moveit_rviz_plugin::CollisionDistanceFieldDisplay::onEnable()
{
  PlanningSceneDisplay::onEnable();
  robotAppearanceChanged();
}

void moveit_rviz_plugin::CollisionDistanceFieldDisplay::onRobotModelLoaded()
{
  PlanningSceneDisplay::onRobotModelLoaded();

  robot_visual_->load(*getRobotModel()->getURDF());

  robot_state_.reset(new robot_state::RobotState(getRobotModel()));
  robot_state_const_ = robot_state_;
  robot_state_->setToDefaultValues();

  robot_model_loaded_ = true;
  robotAppearanceChanged();
}

void moveit_rviz_plugin::CollisionDistanceFieldDisplay::update(float wall_dt, float ros_dt)
{
  updateRobotVisual();

  if (publish_tf_property_->getBool())
    publishTF();

  PlanningSceneDisplay::update(wall_dt, ros_dt);
}

// trigger a call to updateRobotVisual() next update().
void moveit_rviz_plugin::CollisionDistanceFieldDisplay::robotAppearanceChanged()
{
  robot_visual_dirty_ = true;
}

// Update the robot visual appearance based on attributes
void moveit_rviz_plugin::CollisionDistanceFieldDisplay::updateRobotVisual()
{
  if (!robot_model_loaded_ || !robot_visual_dirty_)
    return;

  robot_visual_dirty_ = false;
  bool vis = show_robot_visual_property_->getBool();
  bool col = show_robot_collision_property_->getBool();

  robot_visual_->update(getRobotState(), ColorRGBA(attached_object_color_property_, robot_alpha_property_));

  robot_visual_->setAlpha(robot_alpha_property_->getFloat());
  
  robot_visual_->setCollisionVisible(col);
  robot_visual_->setVisualVisible(vis);
  robot_visual_->setVisible(isEnabled() && (vis || col));

}

void moveit_rviz_plugin::CollisionDistanceFieldDisplay::reset()
{
  robot_model_loaded_ = false;
  robot_visual_->clear();
  robot_visual_->setVisible(false);

  PlanningSceneDisplay::reset();
}

void moveit_rviz_plugin::CollisionDistanceFieldDisplay::publishTF()
{
  const bool PUBLISH_TF_AS_JOINT_TREE = true;

  if (!robot_model_loaded_)
    return;

#if 0
  if (!robot_state_handler_)
    return;
#endif

  robot_state::RobotStateConstPtr state = getRobotState();
  if (!state)
    return;

  const std::vector<robot_state::LinkState*> &ls = state->getLinkStateVector();
  std::vector<geometry_msgs::TransformStamped> transforms(ls.size());
  const std::string &planning_frame = planning_scene_monitor_->getPlanningScene()->getPlanningFrame();
  std::size_t j = 0;
  ros::Time now = ros::Time::now();
  for (std::size_t i = 0 ; i < ls.size() ; ++i)
  {
    if (ls[i]->getName() == planning_frame)
      continue;
    const Eigen::Affine3d &t = ls[i]->getGlobalLinkTransform();
    const robot_state::LinkState* pls = ls[i]->getParentLinkState();
    if (PUBLISH_TF_AS_JOINT_TREE && pls)
    {
      const Eigen::Affine3d &pt = pls->getGlobalLinkTransform();
      Eigen::Affine3d rel = pt.inverse() * t;

      tf::transformEigenToMsg(rel, transforms[j].transform);
      transforms[j].header.frame_id = pls->getName();
    }
    else
    {
      tf::transformEigenToMsg(t, transforms[j].transform);
      transforms[j].header.frame_id = planning_frame;
    }
    transforms[j].header.stamp = now;
    transforms[j].child_frame_id = ls[i]->getName();
    ++j;
  }
  transforms.resize(j);
  tf_broadcaster_.sendTransform(transforms);
}

