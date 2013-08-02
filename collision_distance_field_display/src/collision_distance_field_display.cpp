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
 *   * Neither the name of Willow Garage nor the names of its
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

#include <collision_distance_field_display/collision_distance_field_display.h>
#include <collision_distance_field_display/df_link.h>
#include <collision_distance_field_display/color_cast.h>
#include <collision_distance_field_display/per_link_object.h>
#include <collision_distance_field_display/shapes_display.h>

#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

#include <tf/transform_listener.h>

#include <eigen_conversions/eigen_msg.h>

#include <rviz/visualization_manager.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/int_property.h>
#include <rviz/properties/enum_property.h>
#include <rviz/properties/editable_enum_property.h>
#include <rviz/frame_manager.h>
#include <rviz/display_factory.h>

#include <moveit/rviz_plugin_render_tools/robot_state_visualization.h>
#include <moveit/collision_detection_distance_field/collision_detector_allocator_distance_field.h>
#include <moveit/collision_detection_fcl/collision_detector_allocator_fcl.h>
#include <moveit/robot_sphere_representation/robot_sphere_representation.h>

#include <mesh_core/mesh.h>
#include <mesh_ros/mesh_rviz.h>

enum {
  CD_UNKNOWN,
  CD_FCL,
  CD_DISTANCE_FIELD,
  CD_CNT
};
std::string moveit_rviz_plugin::CollisionDistanceFieldDisplay::COLLISION_METHOD_STRING_FCL = "FCL";
std::string moveit_rviz_plugin::CollisionDistanceFieldDisplay::COLLISION_METHOD_STRING_DISTANCE_FIELD = "DistanceField";

moveit_rviz_plugin::CollisionDistanceFieldDisplay::CollisionDistanceFieldDisplay()
  : PlanningSceneDisplay(true, false)
  , robot_model_loaded_(false)
  , robot_visual_dirty_(true)
  , robot_visual_position_dirty_(true)
  , robot_markers_dirty_(true)
  , robot_markers_position_dirty_(true)
  , int_marker_display_(NULL)
  , sphere_gen_method_property_(NULL)
  , sphere_qual_method_property_(NULL)
  , sphere_gen_resolution_property_(NULL)
  , sphere_gen_tolerance_property_(NULL)
  , requested_nspheres_property_(NULL)
  , unsetting_property_(false)
  , saving_spheres_to_srdf_(false)
{
  robot_state_category_ = new rviz::Property(
                                      "Robot State",
                                      QVariant(),
                                      "Show the robot in a user-draggable state.",
                                      this);
  show_robot_visual_property_ = new rviz::BoolProperty(
                                      "Show Robot Visual",
                                      false,
                                      "Show the robot visual appearance (what it looks like).",
                                      robot_state_category_,
                                      SLOT( robotVisualChanged() ),
                                      this);
  show_robot_collision_property_ = new rviz::BoolProperty(
                                      "Show Robot Collision",
                                      true,
                                      "Show the robot collision geometry.",
                                      robot_state_category_,
                                      SLOT( robotVisualChanged() ),
                                      this);
  collision_method_property_ = new rviz::EnumProperty(
                                      "Collision Method",
                                      "",
                                      "How to perform collision detection.",
                                      robot_state_category_,
                                      SLOT( changedCollisionMethod() ),
                                      this );
  collision_method_property_->addOption(COLLISION_METHOD_STRING_FCL.c_str(), CD_FCL);
  collision_method_property_->addOption(COLLISION_METHOD_STRING_DISTANCE_FIELD.c_str(), CD_DISTANCE_FIELD);
  collision_method_property_->setValue(COLLISION_METHOD_STRING_FCL.c_str());
  collision_use_padded_robot_property_ = new rviz::BoolProperty(
                                      "Use Padded Robot",
                                      false,
                                      "Use padded robot for collision checks.",
                                      robot_state_category_,
                                      SLOT( changedCollisionMethod() ),
                                      this );
  collision_df_use_spheres_ = new rviz::BoolProperty(
                                      "Use Sphere-Sphere check",
                                      false,
                                      "When using DistanceField collision, do self collision checks using sphere-sphere instead of sphere-df.",
                                      robot_state_category_,
                                      SLOT( changedCollisionMethod() ),
                                      this );
  active_group_property_ = new rviz::EditableEnumProperty(
                                      "Active Group",
                                      "",
                                      "The name of the group of links to interact with (from the ones defined in the SRDF)",
                                      robot_state_category_,
                                      SLOT( changedActiveGroup() ),
                                      this );
  collision_aware_ik_property_ = new rviz::BoolProperty(
                                      "Collision aware IK",
                                      true,
                                      "Check collisions when doing IK.",
                                      robot_state_category_);
  publish_tf_property_ = new rviz::BoolProperty(
                                      "Publish Robot State on TF",
                                      true,
                                      "Enable publishing of robot link frames on TF (useful with TF display).",
                                      robot_state_category_);
  colliding_link_color_property_ = new rviz::ColorProperty(
                                      "Colliding Link Color",
                                      QColor( 255, 0, 0 ),
                                      "Color to draw links that are colliding with something.",
                                      robot_state_category_,
                                      SLOT( robotVisualChanged() ),
                                      this);
  joint_violation_link_color_property_ = new rviz::ColorProperty(
                                      "Joint Violation Link Color",
                                      QColor( 255, 0, 255 ),
                                      "Color to draw links whose parent joints are out of range.",
                                      robot_state_category_,
                                      SLOT( robotVisualChanged() ),
                                      this);
  attached_object_color_property_ = new rviz::ColorProperty(
                                      "Attached Object Color",
                                      QColor( 204, 51, 204 ),
                                      "Color to draw objects attached to the robot.",
                                      robot_state_category_,
                                      SLOT( robotVisualChanged() ),
                                      this);
  robot_alpha_property_ = new rviz::FloatProperty(
                                      "Robot Alpha",
                                      1.0,
                                      "0 is fully transparent, 1.0 is fully opaque.",
                                      robot_state_category_,
                                      SLOT( robotVisualChanged() ),
                                      this);

  collision_detection_category_ = new rviz::Property(
                                      "Collision Detection Info",
                                      QVariant(),
                                      "Info for visualizing and debugging distance field collision detection.",
                                      robot_state_category_);
  contact_points_enable_property_ = new rviz::BoolProperty(
                                      "Show Contact Points",
                                      false,
                                      "Show collision contacts.",
                                      collision_detection_category_,
                                      SLOT( showCollidingSpheresChanged() ),
                                      this);
  contact_points_color_property_ = new rviz::ColorProperty(
                                      "Contact Points Color",
                                      QColor( 140, 0, 255 ),
                                      "Color to draw collision contact points.",
                                      contact_points_enable_property_,
                                      SLOT( showCollidingSpheresChanged() ),
                                      this);
  contact_points_size_property_ = new rviz::FloatProperty(
                                      "Contact Points Size",
                                      0.02,
                                      "Size of collision contact points (diameter of sphere marker).",
                                      contact_points_enable_property_,
                                      SLOT( showCollidingSpheresChanged() ),
                                      this);
  contact_normal_length_property_ = new rviz::FloatProperty(
                                      "Contact Normal Length",
                                      0.04,
                                      "Length of arrow showing contact normal.",
                                      contact_points_enable_property_,
                                      SLOT( showCollidingSpheresChanged() ),
                                      this);

  colliding_spheres_enable_property_ = new rviz::BoolProperty(
                                      "Show Colliding Spheres",
                                      false,
                                      "Show spheres that are colliding with something.",
                                      collision_detection_category_,
                                      SLOT( showCollidingSpheresChanged() ),
                                      this);
  colliding_sphere_color_property_ = new rviz::ColorProperty(
                                      "Colliding Sphere Color",
                                      QColor( 255, 0, 0 ),
                                      "Color to draw spheres that are colliding with something.",
                                      colliding_spheres_enable_property_,
                                      SLOT( showCollidingSpheresChanged() ),
                                      this);
  colliding_sphere_alpha_property_ = new rviz::FloatProperty(
                                      "Colliding Sphere Alpha",
                                      0.3,
                                      "0 is fully transparent, 1.0 is fully opaque.",
                                      colliding_spheres_enable_property_,
                                      SLOT( showCollidingSpheresChanged() ),
                                      this);

  closest_distance_enable_property_ = new rviz::BoolProperty(
                                      "Show Distance",
                                      false,
                                      "Show the distance to the nearest obstacle.",
                                      collision_detection_category_,
                                      SLOT( showCollidingSpheresChanged() ),
                                      this);
  closest_distance_value_property_ = new rviz::FloatProperty(
                                      "Distance",
                                      0.0,
                                      "Distance in meters to closest collision.",
                                      closest_distance_enable_property_,
                                      SLOT( showCollidingSpheresChanged() ),
                                      this);
  closest_distance_value_property_->setReadOnly(true);
  closest_distance_color_nocollide_property_ = new rviz::ColorProperty(
                                      "Distance Color - no collision",
                                      QColor( 252, 255, 38 ),
                                      "Color to draw sphere and arrow for Show Distance when not in collision.",
                                      closest_distance_enable_property_,
                                      SLOT( showCollidingSpheresChanged() ),
                                      this);
  closest_distance_color_collide_property_ = new rviz::ColorProperty(
                                      "Distance Color - colliding",
                                      QColor( 255, 20, 17 ),
                                      "Color to draw sphere and arrow for Show Distance when in collision.",
                                      closest_distance_enable_property_,
                                      SLOT( showCollidingSpheresChanged() ),
                                      this);
  closest_distance_alpha_property_ = new rviz::FloatProperty(
                                      "Distance Alpha",
                                      0.5,
                                      "0 is fully transparent, 1.0 is fully opaque.",
                                      closest_distance_enable_property_,
                                      SLOT( showCollidingSpheresChanged() ),
                                      this);

  df_point_examine_enable_ = new rviz::BoolProperty(
                                      "Examine DF Point",
                                      false,
                                      "Examine DF point at this index (aka cell location). -1 to disable.",
                                      collision_detection_category_,
                                      SLOT( dfPointExamineChanged() ),
                                      this);
  df_point_examine_x_ = new rviz::IntProperty(
                                      "X index of point",
                                      0,
                                      "Which point to display -- X value.",
                                      df_point_examine_enable_,
                                      SLOT( dfPointExamineChanged() ),
                                      this);
  df_point_examine_x_->setMin(0);
  df_point_examine_y_ = new rviz::IntProperty(
                                      "Y index of point",
                                      0,
                                      "Which point to display -- Y value.",
                                      df_point_examine_enable_,
                                      SLOT( dfPointExamineChanged() ),
                                      this);
  df_point_examine_y_->setMin(0);
  df_point_examine_z_ = new rviz::IntProperty(
                                      "Z index of point",
                                      0,
                                      "Which point to display -- Z value.",
                                      df_point_examine_enable_,
                                      SLOT( dfPointExamineChanged() ),
                                      this);
  df_point_examine_z_->setMin(0);
  df_point_examine_color_ = new rviz::ColorProperty(
                                      "Examine DF Point Color",
                                      QColor( 0, 255, 0 ),
                                      "Color to draw the current point in the distance field.",
                                      df_point_examine_enable_,
                                      SLOT( dfPointExamineChanged() ),
                                      this);
  df_point_examine_near_color_ = new rviz::ColorProperty(
                                      "Examine DF Nearby Point Color",
                                      QColor( 0, 128, 0 ),
                                      "Color to draw the point nearest the examined point.",
                                      df_point_examine_enable_,
                                      SLOT( dfPointExamineChanged() ),
                                      this);
  df_point_examine_size_ = new rviz::FloatProperty(
                                      "Examine DF Point Size",
                                      0.005,
                                      "Size of examine and nearby point (sphere diameter).",
                                      df_point_examine_enable_,
                                      SLOT( dfPointExamineChanged() ),
                                      this);

  sphere_gen_category_ = new rviz::Property(
                                      "Sphere Generation",
                                      QVariant(),
                                      "Settings for generating a sphere representation of the robot.  See also per-link settings under Links.",
                                      robot_state_category_);

  debug_iteration_ = new rviz::IntProperty(
                                      "Debug Iteration",
                                      -1,
                                      "Used for internal debugging.",
                                      sphere_gen_category_,
                                      SLOT( dfPointExamineChanged() ),
                                      this);
  debug_iteration_->setMin(-1);

  mesh_vis_category_ = new rviz::Property(
                                      "Mesh Visualization",
                                      QVariant(),
                                      "Visualize different meshes and examine/debug mesh algorithms.",
                                      robot_state_category_);



  robot_state_category_->expand();

}

moveit_rviz_plugin::CollisionDistanceFieldDisplay::~CollisionDistanceFieldDisplay()
{
  background_process_.clearJobUpdateEvent();
  clearJobs();

  robot_visual_.reset();
  delete int_marker_display_;
}

void moveit_rviz_plugin::CollisionDistanceFieldDisplay::onInitialize()
{
  PlanningSceneDisplay::onInitialize();

  // create the robot visualization -- this displays the robot in rviz.
  robot_visual_.reset(new RobotStateVisualization(planning_scene_node_, context_, "Robot", robot_state_category_));
  robot_visual_->getRobot().setLinkFactory(new DFLinkFactory(this));
  robotVisualChanged();

  // add per-link data displays to show aspects of distance field
  addPerLinkData();

  // create an interactive marker display used to display markers for interacting with the robot.
  delete int_marker_display_;
  int_marker_display_ = context_->getDisplayFactory()->make("rviz/InteractiveMarkers");
  int_marker_display_->initialize(context_);
}

void moveit_rviz_plugin::CollisionDistanceFieldDisplay::onDisable()
{
  if (robot_interaction_)
    robot_interaction_->clear();
  int_marker_display_->setEnabled(false);
  robot_visual_->setVisible(false);
  PlanningSceneDisplay::onDisable();
}

void moveit_rviz_plugin::CollisionDistanceFieldDisplay::onEnable()
{
  PlanningSceneDisplay::onEnable();
  robotVisualChanged();
  robotMarkersChanged();
  int_marker_display_->setEnabled(true);
  int_marker_display_->setFixedFrame(fixed_frame_);
  changedActiveGroup();
}

void moveit_rviz_plugin::CollisionDistanceFieldDisplay::changedCollisionMethod()
{
  if (!robot_model_loaded_)
    return;

  if (!getPlanningSceneRW()->setActiveCollisionDetector(collision_method_property_->getStdString()))
  {
    // failed.  Set property string to actual active detector
    collision_method_property_->setStdString(getPlanningSceneRO()->getActiveCollisionDetectorName());
  }
}

void moveit_rviz_plugin::CollisionDistanceFieldDisplay::dfPointExamineChanged()
{
  per_link_objects_->update();
}

int moveit_rviz_plugin::CollisionDistanceFieldDisplay::getDebugIteration() const
{
  return debug_iteration_->getInt();
}

void moveit_rviz_plugin::CollisionDistanceFieldDisplay::showCollidingSpheresChanged()
{
  robot_visual_dirty_ = true;
}

void moveit_rviz_plugin::CollisionDistanceFieldDisplay::reset()
{
  robot_model_loaded_ = false;
  robot_visual_->clear();
  robot_visual_->setVisible(false);

  PlanningSceneDisplay::reset();
}

void moveit_rviz_plugin::CollisionDistanceFieldDisplay::fixedFrameChanged()
{
  PlanningSceneDisplay::fixedFrameChanged();
  if (int_marker_display_)
    int_marker_display_->setFixedFrame(fixed_frame_);
  changedActiveGroup();
}

void moveit_rviz_plugin::CollisionDistanceFieldDisplay::onRobotModelLoaded()
{
  PlanningSceneDisplay::onRobotModelLoaded();

  per_link_objects_->clear();

  robot_sphere_rep_.reset(new robot_sphere_representation::RobotSphereRepresentation(getRobotModel()));

  robot_sphere_rep_->setResolution(sphere_gen_resolution_property_->getFloat());
  robot_sphere_rep_->setTolerance(sphere_gen_tolerance_property_->getFloat());
  robot_sphere_rep_->setGenMethod(sphere_gen_method_property_->getStdString());
  robot_sphere_rep_->setQualMethod(sphere_qual_method_property_->getStdString());

  updateAllSphereGenPropertyValues();


  robot_interaction_.reset(new robot_interaction::RobotInteraction(getRobotModel(), "distance_field_display"));
  int_marker_display_->subProp("Update Topic")->setValue(QString::fromStdString(robot_interaction_->getServerTopic() + "/update"));
  robot_visual_->load(*getRobotModel()->getURDF());

  robot_state::RobotStatePtr state(new robot_state::RobotState(getRobotModel()));
  robot_state_handler_.reset(new robot_interaction::RobotInteraction::InteractionHandler("current", *state, planning_scene_monitor_->getTFClient()));
  robot_state_handler_->setUpdateCallback(boost::bind(&CollisionDistanceFieldDisplay::markersMoved, this, _1, _2));
  robot_state_handler_->setStateValidityCallback(boost::bind(&CollisionDistanceFieldDisplay::isIKSolutionCollisionFree, this, _1, _2));

  if (!active_group_property_->getStdString().empty() &&
      !getRobotModel()->hasJointModelGroup(active_group_property_->getStdString()))
  {
    active_group_property_->setStdString("");
  }

  const std::vector<std::string> &groups = getRobotModel()->getJointModelGroupNames();
  active_group_property_->clearOptions();
  for (std::size_t i = 0 ; i < groups.size() ; ++i)
    active_group_property_->addOptionStd(groups[i]);
  active_group_property_->sortOptions();
  if (!groups.empty() && active_group_property_->getStdString().empty())
    active_group_property_->setStdString(groups[0]);


  // setup collision detectors
  std::vector<std::string> detector_names;
  std::string active_detector_name;
  {
    planning_scene_monitor::LockedPlanningSceneRW ps = getPlanningSceneRW();

    // add all collision detectors which should be available
    ps->addCollisionDetector(collision_detection::CollisionDetectorAllocatorFCL::create());
    ps->addCollisionDetector(collision_detection::CollisionDetectorAllocatorDistanceField::create());

    ps->getCollisionDetectorNames(detector_names);
    active_detector_name = ps->getActiveCollisionDetectorName();
  }

  // add collision detector names as property options
  std::string old_detector_name = collision_method_property_->getStdString();
  collision_method_property_->clearOptions();
  for (std::vector<std::string>::const_iterator it = detector_names.begin() ; it != detector_names.end() ; ++it)
    collision_method_property_->addOptionStd(*it);

  // set default detector -- old value if available, or else current active.
  if (std::find(detector_names.begin(), detector_names.end(), old_detector_name) != detector_names.end())
    collision_method_property_->setStdString(old_detector_name);
  else
    collision_method_property_->setStdString(active_detector_name);


  robot_model_loaded_ = true;
  robotVisualChanged();
  robotMarkersChanged();
  changedActiveGroup();
}

void moveit_rviz_plugin::CollisionDistanceFieldDisplay::markersMoved(robot_interaction::RobotInteraction::InteractionHandler *, bool error_state_changed)
{
  robotMarkerPositionsChanged();
  robotVisualPositionChanged();
}

bool moveit_rviz_plugin::CollisionDistanceFieldDisplay::isIKSolutionCollisionFree(robot_state::JointStateGroup *group, const std::vector<double> &ik_solution) const
{
  if (collision_aware_ik_property_->getBool() && planning_scene_monitor_)
  {
    group->setVariableValues(ik_solution);
    bool res = !getPlanningSceneRO()->isStateColliding(*group->getRobotState(), group->getName());
    return res;
  }
  else
    return true;
}

void moveit_rviz_plugin::CollisionDistanceFieldDisplay::changedActiveGroup()
{
  if (!robot_model_loaded_ || !robot_interaction_)
    return;

  if (active_group_property_->getStdString().empty())
  {
    robot_interaction_->clear();
  }
  else if (!getRobotModel()->hasJointModelGroup(active_group_property_->getStdString()))
  {
    robot_interaction_->clear();
  }
  else
  {
    robot_interaction_->decideActiveComponents(active_group_property_->getStdString(), robot_interaction::RobotInteraction::EEF_6DOF_SPHERE);
  }

  robotVisualChanged();
  robotMarkersChanged();
}

// call when the robot visual state changes to update the robot visual
void moveit_rviz_plugin::CollisionDistanceFieldDisplay::robotVisualChanged()
{
  robot_visual_dirty_ = true;
  robot_visual_position_dirty_ = true;
}

// call when the robot state changes to update the robot visual
void moveit_rviz_plugin::CollisionDistanceFieldDisplay::robotVisualPositionChanged()
{
  robot_visual_position_dirty_ = true;
}

// call when the appearance and position of the markers needs to change
void moveit_rviz_plugin::CollisionDistanceFieldDisplay::robotMarkersChanged()
{
  robot_markers_dirty_ = true;
  robot_markers_position_dirty_ = true;
  addBackgroundJob(boost::bind(&CollisionDistanceFieldDisplay::updateRobotMarkers, this), "updateRobotMarkers:all");
}

// call when only the position of the markers needs to change
void moveit_rviz_plugin::CollisionDistanceFieldDisplay::robotMarkerPositionsChanged()
{
  robot_markers_position_dirty_ = true;
  addBackgroundJob(boost::bind(&CollisionDistanceFieldDisplay::updateRobotMarkers, this), "updateRobotMarkers:position");
}

void moveit_rviz_plugin::CollisionDistanceFieldDisplay::setRobotState(const robot_state::RobotState &state)
{
  robot_state_handler_->setState(state);
  robotVisualPositionChanged();
  robotMarkerPositionsChanged();
}

//###########################################################################
//############################### access functions ##########################
//###########################################################################

robot_state::RobotStateConstPtr moveit_rviz_plugin::CollisionDistanceFieldDisplay::getRobotState() const
{
  return robot_state_handler_->getState();
}

const collision_detection::CollisionRobotDistanceField *moveit_rviz_plugin::CollisionDistanceFieldDisplay::getCollisionRobotDistanceField() const
{
  planning_scene_monitor::LockedPlanningSceneRO ps = getPlanningSceneRO();
  const collision_detection::CollisionRobot* crobot;

  if (collision_use_padded_robot_property_->getBool())
    crobot = &*ps->getCollisionRobot(COLLISION_METHOD_STRING_DISTANCE_FIELD);
  else
    crobot = &*ps->getCollisionRobotUnpadded(COLLISION_METHOD_STRING_DISTANCE_FIELD);

  const collision_detection::CollisionRobotDistanceField* crobot_df =
    dynamic_cast<const collision_detection::CollisionRobotDistanceField*>(crobot);

  if (!crobot_df)
  {
    ROS_ERROR("Could not find the CollisionRobotDistanceField instance. %s:%d",__FILE__,__LINE__);
  }

  return crobot_df;
}

const collision_detection::CollisionWorldDistanceField *moveit_rviz_plugin::CollisionDistanceFieldDisplay::getCollisionWorldDistanceField() const
{
  planning_scene_monitor::LockedPlanningSceneRO ps = getPlanningSceneRO();
  const collision_detection::CollisionWorld* cworld = &*ps->getCollisionWorld(COLLISION_METHOD_STRING_DISTANCE_FIELD);
  const collision_detection::CollisionWorldDistanceField* cworld_df =
    dynamic_cast<const collision_detection::CollisionWorldDistanceField*>(cworld);

  if (!cworld_df)
  {
    ROS_ERROR("Could not find the CollisionWorldDistanceField instance. %s:%d",__FILE__,__LINE__);
  }

  return cworld_df;
}

//###########################################################################
//############################### update() processing #######################
//###########################################################################

void moveit_rviz_plugin::CollisionDistanceFieldDisplay::update(float wall_dt, float ros_dt)
{
  if (int_marker_display_)
    int_marker_display_->update(wall_dt, ros_dt);

  updateRobotVisual();

  PlanningSceneDisplay::update(wall_dt, ros_dt);

  if (publish_tf_property_->getBool())
    publishTF();
}

void moveit_rviz_plugin::CollisionDistanceFieldDisplay::getCollidingLinks(
      planning_scene_monitor::LockedPlanningSceneRO& ps,
      const collision_detection::CollisionRobot* crobot,
      std::vector<std::string> &links,
      const robot_state::RobotState &kstate,
      const collision_detection::AllowedCollisionMatrix& acm) const
{
  collision_detection::CollisionRequest req;
  collision_detection::CollisionDistanceFieldRequest dfreq;
  collision_detection::CollisionRequest *preq = &req;

  if (collision_df_use_spheres_->getBool())
  {
    preq = &dfreq;
    dfreq.use_sphere_sphere_for_self_collision = true;
  }

  preq->contacts = true;
  preq->max_contacts = getRobotModel()->getLinkModelsWithCollisionGeometry().size() + 1;
  preq->max_contacts_per_pair = 1;
  collision_detection::CollisionResult res;

  ps->getCollisionWorld()->checkCollision(*preq, res, *crobot, kstate, acm);


  links.clear();
  for (collision_detection::CollisionResult::ContactMap::const_iterator it = res.contacts.begin() ; it != res.contacts.end() ; ++it)
  {
    for (std::size_t j = 0 ; j < it->second.size() ; ++j)
    {
      if (it->second[j].body_type_1 == collision_detection::BodyTypes::ROBOT_LINK)
        links.push_back(it->second[j].body_name_1);
      if (it->second[j].body_type_2 == collision_detection::BodyTypes::ROBOT_LINK)
        links.push_back(it->second[j].body_name_2);
    }
  }
}

int do_dist_check=0;

void moveit_rviz_plugin::CollisionDistanceFieldDisplay::showContactPointsDF(
      planning_scene_monitor::LockedPlanningSceneRO& ps,
      const collision_detection::CollisionRobotDistanceField *crobot,
      const robot_state::RobotState& state)
{
  collision_detection::CollisionDistanceFieldRequest req;
  collision_detection::CollisionResult res;
  std::vector<collision_detection::DFContact> df_contacts;
  req.use_sphere_sphere_for_self_collision = collision_df_use_spheres_->getBool();
  crobot->getSelfCollisionContacts(req, res, state, &ps->getAllowedCollisionMatrix(), &df_contacts);

  if (!df_contacts.empty() && colliding_spheres_enable_property_->getBool())
  {
    colliding_spheres_display_.reset(new ShapesDisplay(planning_scene_node_,
                                                       color_cast::getColorf(colliding_sphere_color_property_,
                                                       colliding_sphere_alpha_property_)));
    for ( int i = 0 ; i < df_contacts.size() ; ++i )
    {
      if (df_contacts[i].sphere_radius_1 > 0.0)
        colliding_spheres_display_->addSphere(df_contacts[i].sphere_center_1, df_contacts[i].sphere_radius_1);
      if (df_contacts[i].sphere_radius_2 > 0.0)
        colliding_spheres_display_->addSphere(df_contacts[i].sphere_center_2, df_contacts[i].sphere_radius_2);
    }
  }

  double contact_marker_radius = contact_points_size_property_->getFloat() * 0.5;
  double normal_length = contact_normal_length_property_->getFloat();

  if (!df_contacts.empty() && contact_points_enable_property_->getBool())
  {
    contact_points_display_.reset(new ShapesDisplay(planning_scene_node_,
                                                    color_cast::getColorf(contact_points_color_property_)));
    for ( int i = 0 ; i < df_contacts.size() ; ++i )
    {
      if (contact_marker_radius > 0.0)
        contact_points_display_->addSphere(df_contacts[i].pos, contact_marker_radius);
      if (normal_length > contact_marker_radius)
        contact_points_display_->addArrow(df_contacts[i].pos, df_contacts[i].pos + (normal_length * df_contacts[i].normal));
    }
  }
}

void moveit_rviz_plugin::CollisionDistanceFieldDisplay::showContactPoints(
      planning_scene_monitor::LockedPlanningSceneRO& ps,
      const collision_detection::CollisionRobot *crobot,
      const robot_state::RobotState& state)
{
  const collision_detection::CollisionRobotDistanceField* crobot_df =
    dynamic_cast<const collision_detection::CollisionRobotDistanceField*>(crobot);
  if (crobot_df)
  {
    showContactPointsDF(ps, crobot_df, state);
    return;
  }

  if (!contact_points_enable_property_->getBool())
    return;

  collision_detection::CollisionRequest req;
  collision_detection::CollisionResult res;
  req.contacts = true;
  req.max_contacts = 1000;
  req.max_contacts_per_pair = 100;

  crobot->checkSelfCollision(req, res, state, ps->getAllowedCollisionMatrix());

  contact_points_display_.reset(new ShapesDisplay(planning_scene_node_,
                                                  color_cast::getColorf(contact_points_color_property_)));

  double contact_marker_radius = contact_points_size_property_->getFloat() * 0.5;
  double normal_length = contact_normal_length_property_->getFloat();

  collision_detection::CollisionResult::ContactMap::const_iterator it = res.contacts.begin();
  collision_detection::CollisionResult::ContactMap::const_iterator end = res.contacts.end();
  for ( ; it != end ; ++it)
  {
    std::vector<collision_detection::Contact>::const_iterator cit = it->second.begin();
    std::vector<collision_detection::Contact>::const_iterator cend = it->second.end();
    for ( ; cit != cend ; ++cit)
    {
      if (contact_marker_radius > 0.0)
        contact_points_display_->addSphere(cit->pos, contact_marker_radius);
      if (normal_length > contact_marker_radius)
        contact_points_display_->addArrow(cit->pos, cit->pos + (normal_length * cit->normal));
    }
  }
}

void moveit_rviz_plugin::CollisionDistanceFieldDisplay::showCollisionDistance(
      planning_scene_monitor::LockedPlanningSceneRO& ps,
      const collision_detection::CollisionRobot *crobot,
      const robot_state::RobotState& state)
{
  double dist = crobot->distanceSelf(state);
  closest_distance_value_property_->setValue(dist);

  const collision_detection::CollisionRobotDistanceField* crobot_df =
    dynamic_cast<const collision_detection::CollisionRobotDistanceField*>(crobot);

  if (crobot_df)
  {
    collision_detection::CollisionDistanceFieldRequest req;
    collision_detection::CollisionResult res;
    collision_detection::DFContact df_distance;
    req.use_sphere_sphere_for_self_collision = collision_df_use_spheres_->getBool();
    crobot_df->getSelfCollisionContacts(req, res, state, &ps->getAllowedCollisionMatrix(), NULL, &df_distance);

    if (!df_distance.body_name_1.empty() || !df_distance.body_name_2.empty())
    {
      Eigen::Vector4f color = df_distance.depth > 0.0 ?
                                color_cast::getColorf(closest_distance_color_collide_property_, closest_distance_alpha_property_) :
                                color_cast::getColorf(closest_distance_color_nocollide_property_, closest_distance_alpha_property_);
      distance_display_.reset(new ShapesDisplay(planning_scene_node_, color));
      if (df_distance.sphere_radius_1 > 0.0)
      {

        distance_display_->addSphere(df_distance.sphere_center_1, df_distance.sphere_radius_1);
        distance_display_->addArrow(df_distance.pos, df_distance.sphere_center_1);
      }
      if (df_distance.sphere_radius_2 > 0.0)
      {
        distance_display_->addSphere(df_distance.sphere_center_2, df_distance.sphere_radius_2);
        distance_display_->addArrow(df_distance.pos, df_distance.sphere_center_2);
      }
    }
  }
}

void moveit_rviz_plugin::CollisionDistanceFieldDisplay::updateLinkColors(const robot_state::RobotState& state)
{
  planning_scene_monitor::LockedPlanningSceneRO ps = getPlanningSceneRO();


  const collision_detection::CollisionRobot* crobot;
  if (collision_use_padded_robot_property_->getBool())
    crobot = &*ps->getCollisionRobot();
  else
    crobot = &*ps->getCollisionRobotUnpadded();

  std::vector<std::string> collision_links;
  getCollidingLinks(ps, crobot, collision_links, state, ps->getAllowedCollisionMatrix());

  unsetAllColors(&robot_visual_->getRobot());

  std::vector<std::string>::const_iterator link = collision_links.begin();
  std::vector<std::string>::const_iterator link_end = collision_links.end();
  for ( ; link != link_end ; ++link)
  {
    setLinkColor(&robot_visual_->getRobot(), *link, colliding_link_color_property_->getColor());
  }

  std::vector<robot_state::JointState*>::const_iterator joint = state.getJointStateVector().begin();
  std::vector<robot_state::JointState*>::const_iterator joint_end = state.getJointStateVector().end();
  for ( ; joint != joint_end ; ++joint)
  {
    if (!(*joint)->satisfiesBounds())
    {
      const std::string& link = (*joint)->getJointModel()->getChildLinkModel()->getName();
      setLinkColor(&robot_visual_->getRobot(), link, joint_violation_link_color_property_->getColor());
    }
  }

  colliding_spheres_display_.reset();
  contact_points_display_.reset();
  if (colliding_spheres_enable_property_->getBool() ||
       contact_points_enable_property_->getBool())
  {
    showContactPoints(ps, crobot, state);
  }

  distance_display_.reset();
  if (closest_distance_enable_property_->getBool())
  {
    showCollisionDistance(ps, crobot, state);
  }
}

// Update the robot visual appearance based on attributes.
// Should only be called from update().  To trigger this call robotVisualChanged() or robotVisualPositionChanged()
void moveit_rviz_plugin::CollisionDistanceFieldDisplay::updateRobotVisual()
{
  if (!robot_model_loaded_)
    return;

  if (robot_visual_dirty_)
  {
    robot_visual_dirty_ = false;
    robot_visual_position_dirty_ = false;

    bool vis = show_robot_visual_property_->getBool();
    bool col = show_robot_collision_property_->getBool();

    robot_visual_->setAlpha(robot_alpha_property_->getFloat());

    robot_visual_->setCollisionVisible(col);
    robot_visual_->setVisualVisible(vis);
    robot_visual_->setVisible(isEnabled() && (vis || col));

    robot_state::RobotStateConstPtr state = getRobotState();
    updateLinkColors(*state);
    robot_visual_->update(state, color_cast::getColorRGBA(attached_object_color_property_, robot_alpha_property_));

    per_link_objects_->updateState();
    context_->queueRender();
  }
  else if (robot_visual_position_dirty_)
  {
    robot_visual_position_dirty_ = false;
    robot_state::RobotStateConstPtr state = getRobotState();
    updateLinkColors(*state);
    robot_visual_->update(state);
    per_link_objects_->updateState();
    context_->queueRender();
  }
}

void moveit_rviz_plugin::CollisionDistanceFieldDisplay::publishTF()
{
  const bool PUBLISH_TF_AS_JOINT_TREE = true;

  if (!robot_model_loaded_)
    return;

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

//###########################################################################
//############################### background tasks ##########################
//###########################################################################

// Update the marker visual appearance based on the robot state.
// Do not call directly.  To trigger this call robotMarkersChanged() or robotMarkerPositionsChanged().
void moveit_rviz_plugin::CollisionDistanceFieldDisplay::updateRobotMarkers()
{
  if (!robot_interaction_)
    return;

  if (robot_markers_dirty_)
  {
    const static double DEFAULT_MARKER_SCALE = 0.0;
    robot_markers_dirty_ = false;
    robot_markers_position_dirty_ = false;
    robot_interaction_->clearInteractiveMarkers();
    robot_interaction_->addInteractiveMarkers(robot_state_handler_, DEFAULT_MARKER_SCALE);
    robot_interaction_->publishInteractiveMarkers();
  }
  else if (robot_markers_position_dirty_)
  {
    robot_markers_position_dirty_ = false;
    robot_interaction_->updateInteractiveMarkers(robot_state_handler_);
  }
}
