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

#ifndef ACORN_DISPLAY_H
#define ACORN_DISPLAY_H

#include <moveit/planning_scene_rviz_plugin/planning_scene_display.h>
#include <tf/transform_broadcaster.h>

namespace Ogre
{
class SceneNode;
}

namespace rviz
{
class ColorProperty;
class FloatProperty;
class IntProperty;
}

namespace moveit_rviz_plugin
{

// Visualise collision distance field info.
class CollisionDistanceFieldDisplay: public PlanningSceneDisplay
{
Q_OBJECT
public:
  CollisionDistanceFieldDisplay();
  virtual ~CollisionDistanceFieldDisplay();

protected:
  virtual void onInitialize();
  virtual void onEnable();
  virtual void onDisable();
  virtual void onRobotModelLoaded();
  virtual void reset();
  virtual void update(float wall_dt, float ros_dt);

private Q_SLOTS:
  void robotAppearanceChanged();

private:
  void updateRobotVisual();
  void publishTF();

  const robot_state::RobotStateConstPtr& getRobotState()
  {
    return robot_state_const_;
  }
  // TODO: remove these robot_state_ vars
  robot_state::RobotStatePtr robot_state_;
  robot_state::RobotStateConstPtr robot_state_const_;


  // for drawing the robot
  RobotStateVisualizationPtr robot_visual_;
  bool robot_visual_dirty_;
  bool robot_model_loaded_;

  // for publishing robot state in standalone mode
  tf::TransformBroadcaster tf_broadcaster_;
  

  // User-editable property variables.
  rviz::BoolProperty* show_robot_visual_property_;
  rviz::BoolProperty* show_robot_collision_property_;
  rviz::BoolProperty* publish_tf_property_;
  rviz::ColorProperty* attached_object_color_property_;
  rviz::FloatProperty* robot_alpha_property_;
};

}

#endif
