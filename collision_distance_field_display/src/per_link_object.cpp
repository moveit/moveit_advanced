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



moveit_rviz_plugin::PerLinkObjBase::~PerLinkObjBase()
{
  for (int i = 0 ; i < prop_list_.size() ; ++i)
    delete prop_list_[i];
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

void moveit_rviz_plugin::PerLinkObjBase::addProp(PerLinkProperty *prop)
{
  prop_map_[prop->getName()] = prop;
  prop_list_.push_back(prop);
}

moveit_rviz_plugin::PerLinkProperty* moveit_rviz_plugin::PerLinkObjBase::getProp(const std::string& name)
{
  std::map<std::string, PerLinkProperty*>::iterator it = prop_map_.find(name);
  if (it == prop_map_.end())
  {
    logError("Looking up nonexistant per-link property '%s'",name.c_str());
    return NULL;
  }
  return it->second;
}

void moveit_rviz_plugin::PerLinkObjBase::setPropValue(const std::string& name, QVariant value)
{
  getProp(name)->setValue(value);
  for (std::vector<PerLinkSubObjBase*>::iterator it = sub_objs_.begin() ; it != sub_objs_.end() ; ++it)
    (*it)->getProp(name)->setValue(value, false);
}

void moveit_rviz_plugin::PerLinkObjBase::propChanged(PerLinkProperty* prop)
{
  const std::string& name = prop->getName();
  QVariant value = prop->getValue();

  for (std::vector<PerLinkSubObjBase*>::iterator it = sub_objs_.begin() ; it != sub_objs_.end() ; ++it)
    (*it)->getProp(name)->setValue(value, false);
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
  {
    sub_objs_.push_back(vso);

    for (int i = 0 ; i < prop_list_.size() ; ++i)
      new PerLinkSubProperty(prop_list_[i], vso);
  }
}



moveit_rviz_plugin::PerLinkSubObjBase::PerLinkSubObjBase(PerLinkObjBase *base, rviz::Property *parent)
  : BoolProperty(base->getName(), false, base->getDescription(), parent)
  , base_(base)
  , robot_relative_(true)
{
  connect(this, SIGNAL( changed() ), this, SLOT( changedEnableSlot() ) );
}

moveit_rviz_plugin::PerLinkSubObjBase::~PerLinkSubObjBase()
{
  std::map<std::string, PerLinkSubProperty*>::iterator pit = prop_map_.begin();
  std::map<std::string, PerLinkSubProperty*>::iterator pend = prop_map_.end();
  for ( ; pit != pend ; ++pit)
    delete pit->second;
}

void moveit_rviz_plugin::PerLinkSubObjBase::addProp(PerLinkSubProperty *prop)
{
  prop_map_[prop->getName()] = prop;
}

void moveit_rviz_plugin::PerLinkSubObjBase::propChanged(PerLinkSubProperty *prop)
{
  changed();
}

moveit_rviz_plugin::PerLinkSubProperty* moveit_rviz_plugin::PerLinkSubObjBase::getProp(const std::string& name)
{
  std::map<std::string, PerLinkSubProperty*>::iterator it = prop_map_.find(name);
  if (it == prop_map_.end())
  {
    logError("asked for getProp('%s') which does not exist", name.c_str());
    return NULL;
  }
  return it->second;
}

moveit_rviz_plugin::PerLinkProperty* moveit_rviz_plugin::PerLinkSubObjBase::getObjProp(const std::string& name)
{
  PerLinkSubProperty* prop = getProp(name);
  return prop ? prop->getObjProp() : NULL;
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




moveit_rviz_plugin::PropertyHolder::PropertyHolder(
      rviz::Property *parent_property,
      const std::string& name,
      const std::string& descr,
      PropertyType type,
      QVariant value,
      unsigned int flags)
  : name_(name)
  , type_(type)
  , flags_(flags)
  , property_(NULL)
{
logInform(" construct PropertyHolder %s at %d (full)",name_.c_str(), __LINE__);
  createProperty(parent_property, descr, value);
}

moveit_rviz_plugin::PropertyHolder::PropertyHolder(
      rviz::Property *parent_property,
      const PropertyHolder* clone)
  : name_(clone->name_)
  , type_(clone->type_)
  , flags_(clone->flags_)
  , property_(NULL)
{
logInform(" construct PropertyHolder %s at %d (clone)",name_.c_str(), __LINE__);
  createProperty(parent_property,
                 clone->property_->getDescription().toStdString(),
                 clone->property_->getValue());
}

void moveit_rviz_plugin::PropertyHolder::createProperty(
      rviz::Property *parent_property,
      const std::string& descr,
      QVariant value)
{
  switch(type_)
  {
    case PT_EMPTY:
      property_ = new rviz::Property(
                             name_.c_str(),
                             QVariant(),
                             descr.c_str(),
                             parent_property);
      break;
    case PT_INT:
      property_ = new rviz::IntProperty(
                             name_.c_str(),
                             value.toInt(),
                             descr.c_str(),
                             parent_property,
                             SLOT( changedSlot() ),
                             this );
      break;
    case PT_FLOAT:
      property_ = new rviz::FloatProperty(
                             name_.c_str(),
                             value.toDouble(),
                             descr.c_str(),
                             parent_property,
                             SLOT( changedSlot() ),
                             this );
      break;
    case PT_BOOL:
    default:
      type_ = PT_BOOL;
      property_ = new rviz::BoolProperty(
                             name_.c_str(),
                             value.toBool(),
                             descr.c_str(),
                             parent_property,
                             SLOT( changedSlot() ),
                             this );
      break;
  }
logInform(" createProperty %08lx",long(property_));
  if (flags_ & PF_READ_ONLY)
    property_->setReadOnly(true);
}

moveit_rviz_plugin::PropertyHolder::~PropertyHolder()
{
}

void moveit_rviz_plugin::PropertyHolder::changedSlot()
{
  changed();
}

void moveit_rviz_plugin::PropertyHolder::changed()
{
}

double moveit_rviz_plugin::PropertyHolder::getFloat() const
{
  rviz::FloatProperty *fprop = dynamic_cast<rviz::FloatProperty*>(property_);
  if (!fprop)
  {
    logError("Attempt to get float from non-float property %s",name_.c_str());
    return 0;
  }
  return fprop->getFloat();
}

int moveit_rviz_plugin::PropertyHolder::getInt() const
{
  rviz::IntProperty *iprop = dynamic_cast<rviz::IntProperty*>(property_);
  if (!iprop)
  {
    logError("Attempt to get int from non-int property %s",name_.c_str());
    return 0;
  }
  return iprop->getInt();
}

bool moveit_rviz_plugin::PropertyHolder::getBool() const
{
  rviz::BoolProperty *bprop = dynamic_cast<rviz::BoolProperty*>(property_);
  if (!bprop)
  {
    logError("Attempt to get bool from non-bool property %s",name_.c_str());
    return false;
  }
  return bprop->getBool();
}

QVariant moveit_rviz_plugin::PropertyHolder::getValue() const
{
  return property_->getValue();
}

moveit_rviz_plugin::PerLinkProperty::PerLinkProperty(
      PerLinkObjBase* obj,
      const std::string& name,
      const std::string& descr,
      PropertyType type,
      QVariant value,
      unsigned int flags)
  : PropertyHolder(obj, name, descr, type, value, flags)
  , obj_(obj)
{
logInform(" construct PerLinkProperty %s at %d",name.c_str(), __LINE__);
  obj_->addProp(this);
  if (flags_ & PF_NOT_GLOBAL)
    property_->hide();
}

void moveit_rviz_plugin::PerLinkProperty::changed()
{
  obj_->propChanged(this);
}

void moveit_rviz_plugin::PerLinkProperty::setValue(
      QVariant value,
      bool update_children)
{
  property_->setValue(value);
  if (update_children)
  {
    obj_->setPropValue(name_, property_->getValue());
  }
}

moveit_rviz_plugin::PerLinkSubProperty::PerLinkSubProperty(
      PerLinkProperty* parent,
      PerLinkSubObjBase* sub_obj)
  : PropertyHolder(sub_obj, parent)
  , parent_(parent)
  , sub_obj_(sub_obj)
  , modified_(false)
{
logInform(" construct PerLinkSubProperty %s at %d",name_.c_str(), __LINE__);
  sub_obj_->addProp(this);
  if (flags_ & PF_NOT_PERLINK)
    property_->hide();
}

void moveit_rviz_plugin::PerLinkSubProperty::changed()
{
  sub_obj_->propChanged(this);
}

void moveit_rviz_plugin::PerLinkSubProperty::setValue(
      QVariant value,
      bool update_all)
{
  property_->setValue(value);
  if (update_all)
  {
    parent_->setValue(value, true);
  }
}
