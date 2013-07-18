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

#include <collision_distance_field_display/per_link_object.h>
#include <collision_distance_field_display/df_link.h>
#include <collision_distance_field_display/shapes_display.h>
#include <collision_distance_field_display/color_cast.h>

#include <rviz/properties/color_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/int_property.h>

#include <console_bridge/console.h>


void moveit_rviz_plugin::PerLinkObjList::addVisObject(PerLinkObjBase* obj)
{
  objs_.push_back(obj);
}

void moveit_rviz_plugin::PerLinkObjList::clear()
{
  for (std::vector<PerLinkObjBase*>::iterator it = objs_.begin() ; it != objs_.end() ; ++it)
    (*it)->clear();
}

void moveit_rviz_plugin::PerLinkObjList::update()
{
  for (std::vector<PerLinkObjBase*>::iterator it = objs_.begin() ; it != objs_.end() ; ++it)
    (*it)->changed();
}

void moveit_rviz_plugin::PerLinkObjList::updateState()
{
  for (std::vector<PerLinkObjBase*>::iterator it = objs_.begin() ; it != objs_.end() ; ++it)
    (*it)->stateChanged();
}

void moveit_rviz_plugin::PerLinkObjList::disableAll()
{
  for (std::vector<PerLinkObjBase*>::iterator it = objs_.begin() ; it != objs_.end() ; ++it)
    (*it)->setValue(false);
}

void moveit_rviz_plugin::PerLinkObjList::addLink(DFLink *link, std::vector<PerLinkSubObjBase*>& added_objects)
{
  for (std::vector<PerLinkObjBase*>::iterator it = objs_.begin() ; it != objs_.end() ; ++it)
  {
    PerLinkSubObjBase *vso = (*it)->createLinkObject(link);
    if (vso)
      added_objects.push_back(vso);
  }
}



moveit_rviz_plugin::PerLinkObjBase::PerLinkObjBase(
                rviz::Property *parent,
                const std::string& name,
                const std::string& descrip,
                const QColor& default_color,
                double default_alpha,
                Style style,
                double size,
                bool update_with_state)
  : style_(style)
  , avoid_enable_update_(false)
  , rviz::BoolProperty(name.c_str(), false, descrip.c_str(), parent)
  , update_with_state_(update_with_state)
{
  connect(this, SIGNAL( changed() ), this, SLOT( changedEnableSlot() ) );

  color_ = new rviz::ColorProperty(
                      ("Color for " + name).c_str(),
                      default_color,
                      ("Color for " + name).c_str(),
                      this,
                      SLOT( changedSlot() ),
                      this );
  alpha_ = new rviz::FloatProperty(
                      ("Alpha for " + name).c_str(),
                      default_alpha,
                      ("Alpha (transparency) for " + name).c_str(),
                      this,
                      SLOT( changedSlot() ),
                      this );
  alpha_->setMin( 0.0 );
  alpha_->setMax( 1.0 );
  size_ = new rviz::FloatProperty(
                      ("Pointsize for " + name).c_str(),
                      size,
                      ("Size of points displayed for " + name).c_str(),
                      this,
                      SLOT( changedSlot() ),
                      this );
  setStyle(style_);
}

void moveit_rviz_plugin::PerLinkObjBase::stateChanged()
{
  if (update_with_state_)
    changed();
}

void moveit_rviz_plugin::PerLinkObjBase::setStyle(Style style)
{
  style_ = style;
  if (style_ != POINTS)
    size_->hide();
  else
    size_->show();
}

void moveit_rviz_plugin::PerLinkObjBase::subObjEnabled()
{
  if (!getBool())
  {
    avoid_enable_update_ = true;
    setValue(true);
    avoid_enable_update_ = false;
  }
}

void moveit_rviz_plugin::PerLinkObjBase::addIntProperty(
      const std::string& name,
      int value,
      const std::string& descrip)
{
  rviz::IntProperty* prop = new rviz::IntProperty(
                      name.c_str(),
                      value,
                      descrip.c_str(),
                      this,
                      SLOT( changedSlot() ),
                      this );
  extra_property_map_[name] = prop;
}

void moveit_rviz_plugin::PerLinkObjBase::addFloatProperty(
      const std::string& name,
      double value,
      const std::string& descrip)
{
  rviz::FloatProperty* prop = new rviz::FloatProperty(
                      name.c_str(),
                      value,
                      descrip.c_str(),
                      this,
                      SLOT( changedSlot() ),
                      this );
  extra_property_map_[name] = prop;
}

void moveit_rviz_plugin::PerLinkObjBase::addBoolProperty(
      const std::string& name,
      bool value,
      const std::string& descrip)
{
  rviz::BoolProperty* prop = new rviz::BoolProperty(
                      name.c_str(),
                      value,
                      descrip.c_str(),
                      this,
                      SLOT( changedSlot() ),
                      this );
  extra_property_map_[name] = prop;
}

rviz::IntProperty* moveit_rviz_plugin::PerLinkObjBase::getIntProperty(
      const std::string& name)
{
  rviz::IntProperty *p = NULL;
  std::map<std::string, rviz::Property*>::iterator it = extra_property_map_.find(name);
  if (it != extra_property_map_.end())
    p = dynamic_cast<rviz::IntProperty*>(it->second);
  if (!p)
    logWarn("No int property '%s' found",name.c_str());
  return p;
}

rviz::FloatProperty* moveit_rviz_plugin::PerLinkObjBase::getFloatProperty(
      const std::string& name)
{
  rviz::FloatProperty *p = NULL;
  std::map<std::string, rviz::Property*>::iterator it = extra_property_map_.find(name);
  if (it != extra_property_map_.end())
    p = dynamic_cast<rviz::FloatProperty*>(it->second);
  if (!p)
    logWarn("No float property '%s' found",name.c_str());
  return p;
}

rviz::BoolProperty* moveit_rviz_plugin::PerLinkObjBase::getBoolProperty(
      const std::string& name)
{
  rviz::BoolProperty *p = NULL;
  std::map<std::string, rviz::Property*>::iterator it = extra_property_map_.find(name);
  if (it != extra_property_map_.end())
    p = dynamic_cast<rviz::BoolProperty*>(it->second);
  if (!p)
    logWarn("No bool property '%s' found",name.c_str());
  return p;
}

void moveit_rviz_plugin::PerLinkObjBase::changedEnableSlot()
{
  if (avoid_enable_update_)
    return;

  bool enabled = getBool();
  for (std::vector<PerLinkSubObjBase*>::iterator it = sub_objs_.begin() ; it != sub_objs_.end() ; ++it)
    (*it)->setValue(enabled);
}

void moveit_rviz_plugin::PerLinkObjBase::changedSlot()
{
  changed();
}

void moveit_rviz_plugin::PerLinkObjBase::clear()
{
  sub_objs_.clear();
}

void moveit_rviz_plugin::PerLinkObjBase::changed()
{
  for (std::vector<PerLinkSubObjBase*>::iterator it = sub_objs_.begin() ; it != sub_objs_.end() ; ++it)
    (*it)->changed();
}

Eigen::Vector4f moveit_rviz_plugin::PerLinkObjBase::getColor()
{
  return color_cast::getColorf(color_, alpha_);
}

double moveit_rviz_plugin::PerLinkObjBase::getSize()
{
  return size_->getFloat();
}

void moveit_rviz_plugin::PerLinkObjBase::addSubObject(PerLinkSubObjBase* vso)
{
  if (vso)
    sub_objs_.push_back(vso);
}



moveit_rviz_plugin::PerLinkSubObjBase::PerLinkSubObjBase(PerLinkObjBase *base, rviz::Property *parent)
  : BoolProperty(base->getName(), false, base->getDescription(), parent)
  , base_(base)
  , robot_relative_(true)
{
  connect(this, SIGNAL( changed() ), this, SLOT( changedEnableSlot() ) );
}

void moveit_rviz_plugin::PerLinkSubObjBase::changed()
{
  shapes_.reset();
  centers_.clear();
  radii_.clear();

  if (!getBool())
    return;

  getGeom(robot_relative_, centers_, radii_);

  if (centers_.empty())
    return;

  shapes_.reset(new ShapesDisplay(getSceneNode(), base_->getColor(), base_->getSize()));

  if (centers_.size() == radii_.size() || radii_.size() == 1)
  {
    shapes_->addSpheres(centers_, radii_);
    base_->setStyle(PerLinkObjBase::SPHERES);
  }
  else if (base_->getStyle() == PerLinkObjBase::SPHERES)
  {
    shapes_->addSpheres(centers_, base_->getSize() * 0.5);
  }
  else
  {
    shapes_->addPoints(centers_);
  }
}

void moveit_rviz_plugin::PerLinkSubObjBase::changedSlot()
{
  changed();
}

void moveit_rviz_plugin::PerLinkSubObjBase::changedEnableSlot()
{
  if (getBool())
    base_->subObjEnabled();
  changed();
}

// helper to get points from a marker.  Can be used by PerLinkSubObjBase subclass implementations of getGeom()
void moveit_rviz_plugin::PerLinkSubObjBase::pointsFromMarker(const visualization_msgs::Marker& marker)
{
  radii_.clear();
  for (visualization_msgs::Marker::_points_type::const_iterator it = marker.points.begin() ;
       it != marker.points.end() ;
       ++it)
  {
    centers_.push_back(Eigen::Vector3d(it->x, it->y, it->z));
  }
}

moveit_rviz_plugin::PerLinkSubObj::PerLinkSubObj(
      PerLinkObjBase *base,
      DFLink *link)
  : PerLinkSubObjBase(base, link->getLinkProperty())
  , link_(link)
{
}

Ogre::SceneNode *moveit_rviz_plugin::PerLinkSubObj::getSceneNode()
{
  if (robot_relative_)
    return link_->getRobot()->getCollisionNode();
  else
    return link_->getCollisionNode();
}

