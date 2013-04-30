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

#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

#include <tf/transform_listener.h>

#include <rviz/visualization_manager.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/int_property.h>
#include <rviz/frame_manager.h>

#include <moveit/rviz_plugin_render_tools/robot_state_visualization.h>

#include <collision_distance_field_display/collision_distance_field_display.h>

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
                                      false,
                                      "Show the robot collision geometry.",
                                      this,
                                      SLOT( robotAppearanceChanged() ));
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

#if 1
boost::shared_ptr<const robot_model::RobotModel> model = getRobotModel();
ROS_INFO("onRobotModelLoaded model=%08lx", (long)model.get());
#endif

  robot_visual_->load(*getRobotModel()->getURDF());
  robot_model_loaded_ = true;
  robotAppearanceChanged();
}

void moveit_rviz_plugin::CollisionDistanceFieldDisplay::update(float wall_dt, float ros_dt)
{
  updateRobotVisual();

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

#if 0
  robot_visual_->setDefaultAttachedObjectColor(ColorRGBA(attached_object_color_property_, robot_alpha_property_));
#endif



ROS_INFO("Set robot state");
#if 0
  boost::shared_ptr<robot_state::RobotState> state(new robot_state::RobotState(getRobotModel()));
#else
  boost::shared_ptr<const robot_model::RobotModel> model = getRobotModel();
ROS_INFO("robot model=%08lx", (long)model.get());
  if (model)
  {
ROS_INFO("   Update state and color");
    boost::shared_ptr<robot_state::RobotState> state(new robot_state::RobotState(model));
    state->setToDefaultValues();
    robot_visual_->update(state, ColorRGBA(attached_object_color_property_, robot_alpha_property_));
  }
  else
  {
ROS_INFO("   Update color only");
    robot_visual_->setDefaultAttachedObjectColor(ColorRGBA(attached_object_color_property_, robot_alpha_property_));
  }
#endif

  robot_visual_->setAlpha(robot_alpha_property_->getFloat());
  
  robot_visual_->setCollisionVisible(vis);
  robot_visual_->setVisualVisible(col);
  robot_visual_->setVisible(isEnabled() && (vis || col));

}

void moveit_rviz_plugin::CollisionDistanceFieldDisplay::reset()
{
  robot_model_loaded_ = false;
  robot_visual_->clear();
  robot_visual_->setVisible(false);

  PlanningSceneDisplay::reset();
}

