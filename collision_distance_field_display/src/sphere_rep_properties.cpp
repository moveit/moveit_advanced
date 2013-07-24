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
#include <collision_distance_field_display/per_link_object.h>
#include <collision_distance_field_display/df_link.h>
#include <collision_distance_field_display/color_cast.h>

#include <moveit/robot_sphere_representation/link_sphere_representation.h>

#include <moveit/rviz_plugin_render_tools/robot_state_visualization.h>

#include <rviz/properties/color_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/enum_property.h>
#include <rviz/properties/int_property.h>

#include <QFileDialog>



static void setGenMethodValues(rviz::EnumProperty *prop)
{
  prop->clearOptions();

  std::vector<std::string>::const_iterator it = robot_sphere_representation::GenMethod::getNames().begin();
  std::vector<std::string>::const_iterator it_end = robot_sphere_representation::GenMethod::getNames().end();
  for ( ; it != it_end ; ++it )
  {
    prop->addOptionStd(*it);
  }
}

static void setGenMethodDefault(rviz::EnumProperty *prop)
{
  prop->setStdString(robot_sphere_representation::GenMethod::getDefaultName());
}

static void setQualMethodValues(rviz::EnumProperty *prop)
{
  prop->clearOptions();

  std::vector<std::string>::const_iterator it = robot_sphere_representation::QualMethod::getNames().begin();
  std::vector<std::string>::const_iterator it_end = robot_sphere_representation::QualMethod::getNames().end();
  for ( ; it != it_end ; ++it )
  {
    prop->addOptionStd(*it);
  }
}

static void setQualMethodDefault(rviz::EnumProperty *prop)
{
  prop->setStdString(robot_sphere_representation::QualMethod::getDefaultName());
}


void moveit_rviz_plugin::CollisionDistanceFieldDisplay::addSphereGenProperties(rviz::Property* parent_property)
{
  save_to_srdf_property_ = new rviz::BoolProperty(
                                      "Save Generated Spheres to SRDF",
                                      false,
                                      "Save generated spheres to SRDF.  Spheres will be generated with current settings.  "
                                      "Click 'Show Generated Spheres' to see spheres before saving.",
                                      parent_property,
                                      SLOT( changedSaveSpheresToSrdf() ),
                                      this );
  sphere_gen_method_property_ = new rviz::EnumProperty(
                                      "Sphere Gen Method",
                                      "",
                                      "How to generate collision spheres.",
                                      parent_property,
                                      SLOT( changedSphereGenMethod() ),
                                      this );
  setGenMethodValues(sphere_gen_method_property_);
  setGenMethodDefault(sphere_gen_method_property_);

  sphere_qual_method_property_ = new rviz::EnumProperty(
                                      "Sphere Qual Method",
                                      "",
                                      "How to evaluate quality of collision spheres.",
                                      parent_property,
                                      SLOT( changedSphereQualMethod() ),
                                      this );
  setQualMethodValues(sphere_qual_method_property_);
  setQualMethodDefault(sphere_qual_method_property_);

  sphere_gen_resolution_property_ = new rviz::FloatProperty(
                                      "Sphere Gen Resolution", 
                                      0.03, 
                                      "What resolution to use for the distance field for calculating spheres.  Smaller is slower but more accurate.",
                                      parent_property, 
                                      SLOT( changedSphereGenResolution() ), 
                                      this);
  sphere_gen_resolution_property_->setMin( 0.0000001 );

  sphere_gen_tolerance_property_ = new rviz::FloatProperty(
                                      "Tolerance", 
                                      1.0, 
                                      "How much the generated collision sphere volume is allowed to stick out from the actual mesh volume, in multiples of '<b>Sphere Gen Resolution</b>'."
                                      "Values less than or equal to 0 indicate each link is using an individual tolerance",
                                      parent_property, 
                                      SLOT( changedSphereGenTolerance() ), 
                                      this);

  requested_nspheres_property_ = new rviz::IntProperty(
                                      "Number of Spheres Requested",
                                      10,
                                      "How many spheres to generate for each link (ignored by some methods).",
                                      parent_property,
                                      SLOT( changedRequestedNspheres() ),
                                      this );
}

void moveit_rviz_plugin::CollisionDistanceFieldDisplay::changedSphereGenMethod()
{
  if (!robot_model_loaded_ || unsetting_property_)
    return;

  robot_sphere_rep_->setGenMethod(sphere_gen_method_property_->getStdString());

  per_link_objects_->update();
  updateLinkSphereGenPropertyValues();
}

void moveit_rviz_plugin::CollisionDistanceFieldDisplay::changedSphereQualMethod()
{
  if (!robot_model_loaded_ || unsetting_property_)
    return;

  robot_sphere_rep_->setQualMethod(sphere_qual_method_property_->getStdString());

  per_link_objects_->update();
  updateLinkSphereGenPropertyValues();
}

void moveit_rviz_plugin::CollisionDistanceFieldDisplay::changedSphereGenResolution()
{
  if (!robot_model_loaded_ || unsetting_property_)
    return;

  if (robot_sphere_rep_)
    robot_sphere_rep_->setResolution(sphere_gen_resolution_property_->getFloat());

  per_link_objects_->update();
  updateLinkSphereGenPropertyValues();
}

void moveit_rviz_plugin::CollisionDistanceFieldDisplay::changedSphereGenTolerance()
{
  if (!robot_model_loaded_ || unsetting_property_)
    return;

  double tolerance = sphere_gen_tolerance_property_->getFloat();
  if (tolerance <= 0.0)
    return;

  if (robot_sphere_rep_)
      robot_sphere_rep_->setTolerance(sphere_gen_tolerance_property_->getFloat());

  per_link_objects_->update();
  updateLinkSphereGenPropertyValues();
}

void moveit_rviz_plugin::CollisionDistanceFieldDisplay::changedRequestedNspheres()
{
  if (!robot_model_loaded_ || unsetting_property_)
    return;

  if (robot_sphere_rep_)
    robot_sphere_rep_->setRequestedNumSpheres(requested_nspheres_property_->getInt());

  per_link_objects_->update();
  updateLinkSphereGenPropertyValues();
}

void moveit_rviz_plugin::CollisionDistanceFieldDisplay::changedSaveSpheresToSrdf()
{
  if (!robot_sphere_rep_)
  {
    save_to_srdf_property_->setValue(false);
    save_to_srdf_property_->setReadOnly(false);
    return;
  }

  if (saving_spheres_to_srdf_)
    return;

  if (!save_to_srdf_property_->getBool())
    return;

  save_to_srdf_property_->setReadOnly(true);
  save_to_srdf_property_->setValue("GENERATING SPHERES");
  saving_spheres_to_srdf_ = true;

  addMainLoopJob(boost::bind(&CollisionDistanceFieldDisplay::saveSpheresToSrdf, this));
}

void moveit_rviz_plugin::CollisionDistanceFieldDisplay::saveSpheresToSrdf()
{
  if (saving_spheres_to_srdf_)
  {

    // Generate spheres for all links if not already done
    // This is not necessary, but it ensures the spheres are all successfully
    // generated before asking the user for a filename.
    robot_sphere_rep_->genSpheresForAllLinks();

    QString srdf_qfilename = QFileDialog::getSaveFileName(
                  0,
                  "SRDF file to save",
                  QString(),
                  "*.srdf",
                  0,
                  0);
                                            
    std::string srdf_filename = srdf_qfilename.toStdString();

    if (!srdf_filename.empty())
    {
      ROS_INFO("Saving spheres to file %s", srdf_filename.c_str());
      robot_sphere_rep_->saveToSrdfFile(srdf_filename);
    }
    else
    {
      ROS_WARN("No filename -- not saving SRDF");
    }
  }

  save_to_srdf_property_->setValue(false);
  save_to_srdf_property_->setReadOnly(false);
  saving_spheres_to_srdf_ = false;
}


// Set property values for links to match current RobotSphereRepresentation values.
void moveit_rviz_plugin::CollisionDistanceFieldDisplay::updateLinkSphereGenPropertyValues()
{
  rviz::Robot::M_NameToLink::const_iterator link_it = robot_visual_->getRobot().getLinks().begin();
  rviz::Robot::M_NameToLink::const_iterator link_end = robot_visual_->getRobot().getLinks().end();
  for ( ; link_it != link_end ; ++link_it )
  {
    DFLink *link = dynamic_cast<DFLink*>(link_it->second);
    if (link)
    {
      link->updatePropertyValues();
    }
  }
}

// Set property values (global and per link properties) to match current RobotSphereRepresentation values.
void moveit_rviz_plugin::CollisionDistanceFieldDisplay::updateAllSphereGenPropertyValues()
{
  if (!robot_model_loaded_ || unsetting_property_)
    return;

  robot_sphere_representation::GenMethod gen_method = robot_sphere_representation::GenMethod::DEFAULT;
  robot_sphere_representation::QualMethod qual_method = robot_sphere_representation::QualMethod::DEFAULT;
  float tolerance = 1.0;
  int requested_nspheres = 10;

  bool gen_method_is_same = true;
  bool qual_method_is_same = true;
  bool tolerance_is_same = true;
  bool requested_nspheres_is_same = true;

  bool first = true;

  // Find property values from link
  rviz::Robot::M_NameToLink::const_iterator link_it = robot_visual_->getRobot().getLinks().begin();
  rviz::Robot::M_NameToLink::const_iterator link_end = robot_visual_->getRobot().getLinks().end();
  for ( ; link_it != link_end ; ++link_it )
  {
    DFLink *link = dynamic_cast<DFLink*>(link_it->second);
    if (link)
    {
      if (first)
      {
        first = false;
        gen_method = link->getLinkSphereRepresentation()->getGenMethod();
        qual_method = link->getLinkSphereRepresentation()->getQualMethod();
        tolerance = link->getLinkSphereRepresentation()->getTolerance();
        requested_nspheres = link->getLinkSphereRepresentation()->getRequestedNumSpheres();
      }
      else
      {
        if (gen_method != link->getLinkSphereRepresentation()->getGenMethod())
          gen_method_is_same = false;
        if (qual_method != link->getLinkSphereRepresentation()->getQualMethod())
          qual_method_is_same = false;
        if (tolerance != link->getLinkSphereRepresentation()->getTolerance())
          tolerance_is_same = false;
        if (requested_nspheres != link->getLinkSphereRepresentation()->getRequestedNumSpheres())
          requested_nspheres_is_same = false;

      }
    }
  }

  unsetting_property_ = true;

  sphere_gen_method_property_->setStringStd(
        gen_method_is_same ? gen_method.toName() : "");

  sphere_qual_method_property_->setStringStd(
        qual_method_is_same ? qual_method.toName() : "");

  sphere_gen_tolerance_property_->setValue(
        tolerance_is_same ? tolerance : -1.0);

  requested_nspheres_property_->setValue(
        requested_nspheres_is_same ? requested_nspheres : -1);

  unsetting_property_ = false;

  updateLinkSphereGenPropertyValues();
}


void moveit_rviz_plugin::DFLink::addSphereGenProperties(rviz::Property *parent_property)
{
  inUpdatePropertyValues = true;

  sphere_gen_method_property_ = new rviz::EnumProperty(
                                      "Sphere Gen Method",
                                      "",
                                      "How to generate collision spheres.",
                                      parent_property,
                                      SLOT( changedSphereGenMethod() ),
                                      this );
  setGenMethodValues(sphere_gen_method_property_);
  setGenMethodDefault(sphere_gen_method_property_);

  sphere_qual_method_property_ = new rviz::EnumProperty(
                                      "Sphere Qual Method",
                                      "",
                                      "How to evaluate quality of collision spheres.",
                                      parent_property,
                                      SLOT( changedSphereQualMethod() ),
                                      this );
  setQualMethodValues(sphere_qual_method_property_);
  setQualMethodDefault(sphere_qual_method_property_);

  sphere_gen_tolerance_property_ = new rviz::FloatProperty(
                                      "Tolerance", 
                                      1.0, 
                                      "How much the generated collision sphere volume is allowed to stick out from the actual mesh volume, in multiples of '<b>Sphere Gen Resolution</b>'."
                                      "Values less than or equal to 0 indicate each link is using an individual tolerance",
                                      parent_property, 
                                      SLOT( changedSphereGenTolerance() ), 
                                      this);

  requested_nspheres_property_ = new rviz::IntProperty(
                                      "Number of Spheres Requested",
                                      10,
                                      "How many spheres to generate for this link (ignored by some methods).",
                                      parent_property,
                                      SLOT( changedRequestedNspheres() ),
                                      this );

  generated_nspheres_property_ = new rviz::IntProperty(
                                      "Number of Spheres generated for link",
                                      0,
                                      "How many spheres are being used to represent this link).",
                                      parent_property);
  generated_nspheres_property_->setReadOnly(true);

  inUpdatePropertyValues = false;
}

void moveit_rviz_plugin::DFLink::updatePropertyValues()
{
  if (inUpdatePropertyValues)
    return;

  inUpdatePropertyValues = true;

  sphere_gen_method_property_->setStringStd(getLinkSphereRepresentation()->getGenMethod().toName());
  sphere_qual_method_property_->setStringStd(getLinkSphereRepresentation()->getQualMethod().toName());
  sphere_gen_tolerance_property_->setValue(getLinkSphereRepresentation()->getTolerance());
  requested_nspheres_property_->setValue(getLinkSphereRepresentation()->getRequestedNumSpheres());
  generated_nspheres_property_->setValue(getLinkSphereRepresentation()->getActualNumSpheres());

  inUpdatePropertyValues = false;
}

void moveit_rviz_plugin::DFLink::updateObjects()
{
  if (inUpdatePropertyValues)
    return;

  if (getLinkSphereRepresentation())
  {
    getLinkSphereRepresentation()->setGenMethod(sphere_gen_method_property_->getStdString());
    getLinkSphereRepresentation()->setQualMethod(sphere_qual_method_property_->getStdString());
    getLinkSphereRepresentation()->setTolerance(sphere_gen_tolerance_property_->getFloat());
    getLinkSphereRepresentation()->setRequestedNumSpheres(requested_nspheres_property_->getInt());
  }

  std::vector<PerLinkSubObjBase*>::iterator it = per_link_objects_.begin();
  std::vector<PerLinkSubObjBase*>::iterator it_end = per_link_objects_.end();
  for ( ; it != it_end ; ++it )
  {
    (*it)->changed();
  }
  updatePropertyValues();
}

void moveit_rviz_plugin::DFLink::changedSphereGenMethod()
{
  if (inUpdatePropertyValues)
    return;

  updateObjects();
  display_->updateAllSphereGenPropertyValues();
}

void moveit_rviz_plugin::DFLink::changedSphereQualMethod()
{
  if (inUpdatePropertyValues)
    return;

  updateObjects();
  display_->updateAllSphereGenPropertyValues();
}

void moveit_rviz_plugin::DFLink::changedSphereGenTolerance()
{
  if (inUpdatePropertyValues)
    return;

  updateObjects();
  display_->updateAllSphereGenPropertyValues();
}

void moveit_rviz_plugin::DFLink::changedRequestedNspheres()
{
  if (inUpdatePropertyValues)
    return;

  updateObjects();
  display_->updateAllSphereGenPropertyValues();
}


