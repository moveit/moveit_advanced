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
#include <collision_distance_field_display/df_link.h>
#include <collision_distance_field_display/color_cast.h>
#include <collision_distance_field_display/per_link_object.h>
#include <collision_distance_field_display/spheres_display.h>

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
  show_contact_points_property_ = new rviz::BoolProperty(
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
                                      collision_detection_category_,
                                      SLOT( showCollidingSpheresChanged() ),
                                      this);
  contact_points_size_property_ = new rviz::FloatProperty(
                                      "Contact Points Size",
                                      0.02,
                                      "Size of collision contact points (diameter of sphere marker).",
                                      collision_detection_category_,
                                      SLOT( showCollidingSpheresChanged() ),
                                      this);
  show_colliding_spheres_property_ = new rviz::BoolProperty(
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
                                      collision_detection_category_,
                                      SLOT( showCollidingSpheresChanged() ),
                                      this);
  colliding_sphere_alpha_property_ = new rviz::FloatProperty(
                                      "Colliding Sphere Alpha",
                                      0.3,
                                      "0 is fully transparent, 1.0 is fully opaque.",
                                      collision_detection_category_,
                                      SLOT( showCollidingSpheresChanged() ),
                                      this);
  colliding_sphere_color_property_->setHidden(!show_colliding_spheres_property_->getBool());
  colliding_sphere_alpha_property_->setHidden(!show_colliding_spheres_property_->getBool());
  contact_points_color_property_->setHidden(!show_contact_points_property_->getBool());
  contact_points_size_property_->setHidden(!show_contact_points_property_->getBool());

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

  robot_state_category_->expand();

}

moveit_rviz_plugin::CollisionDistanceFieldDisplay::~CollisionDistanceFieldDisplay()
{
  background_process_.setJobUpdateEvent(BackgroundProcessing::JobUpdateCallback());
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
  addPerLinkData(collision_detection_category_, sphere_gen_category_);

  // create an interactive marker display used to display markers for interacting with the robot.
  delete int_marker_display_;
  int_marker_display_ = context_->getDisplayFactory()->make("rviz/InteractiveMarkers");
  int_marker_display_->initialize(context_);

#if 0
// Show a unit sphere at origin to prove that radius=1.0 really shows a sphere with radius=1.0
{
  moveit_rviz_plugin::SpheresDisplay *sd = new moveit_rviz_plugin::SpheresDisplay(planning_scene_node_);
  sd->addSphere(Eigen::Vector3d(0,0,0), 1.0, Eigen::Vector4f(1,1,0,0.5));
}
#endif
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

void moveit_rviz_plugin::CollisionDistanceFieldDisplay::showCollidingSpheresChanged()
{
  colliding_sphere_color_property_->setHidden(!show_colliding_spheres_property_->getBool());
  colliding_sphere_alpha_property_->setHidden(!show_colliding_spheres_property_->getBool());
  contact_points_color_property_->setHidden(!show_contact_points_property_->getBool());
  contact_points_size_property_->setHidden(!show_contact_points_property_->getBool());
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
  const collision_detection::CollisionRobot* crobot = &*ps->getCollisionRobot(COLLISION_METHOD_STRING_DISTANCE_FIELD);
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

void moveit_rviz_plugin::CollisionDistanceFieldDisplay::updateLinkColors(const robot_state::RobotState& state)
{
  std::vector<std::string> collision_links;
  getPlanningSceneRO()->getCollidingLinks(collision_links, state);

  unsetAllColors(&robot_visual_->getRobot());

  for (std::vector<std::string>::const_iterator link = collision_links.begin() ; link != collision_links.end() ; ++link)
  {
    setLinkColor(&robot_visual_->getRobot(), *link, colliding_link_color_property_->getColor());
  }

  for (std::vector<robot_state::JointState*>::const_iterator joint = state.getJointStateVector().begin() ; joint != state.getJointStateVector().end() ; ++joint)
  {
    if (!(*joint)->satisfiesBounds())
    {
      const std::string& link = (*joint)->getJointModel()->getChildLinkModel()->getName();
      setLinkColor(&robot_visual_->getRobot(), link, joint_violation_link_color_property_->getColor());
    }
  }

  colliding_spheres_display_.reset();
  contact_points_display_.reset();
  if ((show_colliding_spheres_property_->getBool() ||
       show_contact_points_property_->getBool()) &&
      collision_method_property_->getStdString() == COLLISION_METHOD_STRING_DISTANCE_FIELD)
  {
    const collision_detection::CollisionRobotDistanceField *crobot = getCollisionRobotDistanceField();
    if (crobot)
    {
      collision_detection::CollisionRequest req;
      collision_detection::CollisionResult res;
      std::vector<collision_detection::DFContact> df_contacts;
      crobot->getSelfCollisionContacts(req, res, state, &getPlanningSceneRO()->getAllowedCollisionMatrix(), &df_contacts);

      if (!df_contacts.empty() && show_colliding_spheres_property_->getBool())
      {
        colliding_spheres_display_.reset(new SpheresDisplay(planning_scene_node_,
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

      if (!df_contacts.empty() && show_contact_points_property_->getBool())
      {
        contact_points_display_.reset(new SpheresDisplay(planning_scene_node_,
                                                            color_cast::getColorf(contact_points_color_property_)));
        for ( int i = 0 ; i < df_contacts.size() ; ++i )
        {
          contact_points_display_->addSphere(df_contacts[i].pos, contact_points_size_property_->getFloat());
        }
      }
    }
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

    context_->queueRender();
  }
  else if (robot_visual_position_dirty_)
  {
    robot_visual_position_dirty_ = false;
    robot_state::RobotStateConstPtr state = getRobotState();
    updateLinkColors(*state);
    robot_visual_->update(state);
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

