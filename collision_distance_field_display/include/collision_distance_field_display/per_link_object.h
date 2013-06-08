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

#ifndef COLLISION_DISTANCE_FIELD_PER_LINK_OBJECT_H
#define COLLISION_DISTANCE_FIELD_PER_LINK_OBJECT_H

#include <rviz/robot/robot.h>
#include <rviz/robot/robot_link.h>
#include <rviz/properties/bool_property.h>
#include <Eigen/Geometry>
#include <eigen_stl_containers/eigen_stl_vector_container.h>
#include <visualization_msgs/Marker.h>

namespace rviz
{
class ColorProperty;
class FloatProperty;
class IntProperty;
class EnumProperty;
class EditableEnumProperty;
}

namespace moveit_rviz_plugin
{

class DFLink;
class PointsDisplay;
class SpheresDisplay;
class CylindersDisplay;
class PerLinkObjBase;
class PerLinkSubObjBase;


/// Represents a set of objects that may be drawn for any link(s).
class PerLinkObjList
{
public:

  // call the update functions for all objects in all links.
  void update();

  // turn off all objects for all links
  void disableAll();

  // remove all sub-objects
  void clear();


  // add a new object which will be available to be added to any link.
  // Generally this is called by a PerLinkSubObj subclass' addSelf() method.
  void addVisObject(PerLinkObjBase* obj);

  // Add all per-link objects to a link.  Called by link constructor.
  void addLink(DFLink *link, std::vector<PerLinkSubObjBase*>& added_objects);

  // This would be used if we had any per-joint objects
  //void addJoint(DFJoint *link);

private:
  std::vector<PerLinkObjBase*> objs_;
};



/// Represents one piece of data from each link.
class PerLinkObjBase : public rviz::BoolProperty
{
Q_OBJECT;
public:
  enum Style { POINTS, SPHERES, CYLINDERS };

  Eigen::Vector4f getColor();
  double getSize();
  double getStyle() { return style_; }
  void setStyle(Style style);
  virtual void changed();
  void clear();
  void disableAll();

  virtual PerLinkSubObjBase* createLinkObject(DFLink *link) = 0;
  //virtual void createJointObject(DFJoint *joint) = 0;

  void subObjEnabled();

protected:
  PerLinkObjBase(rviz::Property *parent,
                const std::string& name,
                const std::string& descrip,
                const QColor& default_color,
                double default_alpha,
                Style style,
                double size);

  void addSubObject(PerLinkSubObjBase*);

private Q_SLOTS:
  void changedSlot();
  void changedEnableSlot();

private:
  rviz::ColorProperty* color_;
  rviz::FloatProperty* alpha_;
  rviz::FloatProperty* size_;
  Style style_;
  std::vector<PerLinkSubObjBase*> sub_objs_;
  bool avoid_enable_update_;
};



/// Represents one piece of data from each link.
template<class VSO>
class PerLinkObj : public PerLinkObjBase
{
public:
  PerLinkObj(rviz::Property *parent,
            const std::string& name,
            const std::string& descrip,
            const QColor& default_color = Qt::white,
            double default_alpha = 1.0,
            Style style = SPHERES,
            double size = 0.005) :
    PerLinkObjBase(parent, name, descrip, default_color, default_alpha, style, size)
  {}

protected:
  virtual PerLinkSubObjBase* createLinkObject(DFLink *link)
  {
    PerLinkSubObjBase *vso = new VSO(this, link);
    addSubObject(vso);
    return vso;
  }
  //virtual void createJointObject(DFJoint *joint) {}
};

//class PerJointObj ...   not implemented.


/// Represents one piece of data in a particular link or joint.
// The piece of data is drawn as a set of points, spheres, or cylinders.
class PerLinkSubObjBase : public rviz::BoolProperty
{
Q_OBJECT;
public:
  // Call to recalc and redisplay the vis.  Calls getGeom.
  virtual void changed();

protected:
  PerLinkSubObjBase(PerLinkObjBase *base, rviz::Property *parent);

  // Override this.  It should calculate the geometry of the shapes to display.
  // If getGeom() is not flexible enough to draw what you need, you can instead
  // override changed() and do all the work yourself.
  //   robot_relative - Set to false if coordinates are relative to link, true if relative to robot.
  //   centers - Fill in with centers of the spheres/points to draw.
  //   radii - Fill in with radii of spheres.  If all spheres are same size
  //            this can be a single entry.  Leave empty for points.
  virtual void getGeom(bool& robot_relative,
                       EigenSTL::vector_Vector3d& centers,
                       std::vector<double>& radii) {}

  // The scene node to draw the geometry on.
  virtual Ogre::SceneNode *getSceneNode() = 0;

  // call from getGeom() to set geometry from marker.points.
  void pointsFromMarker(const visualization_msgs::Marker& marker);

  // call from getGeom to set the geometry to a cylinder
  void setCylinder(const Eigen::Affine3d& pose, double radius, double length);

  EigenSTL::vector_Vector3d centers_;
  std::vector<double> radii_;
  PerLinkObjBase *base_;
  bool robot_relative_;

  // Display a set of shapes.  Only one of these gets used, depending on what
  // getGeom does
  boost::shared_ptr<PointsDisplay> points_;
  boost::shared_ptr<SpheresDisplay> spheres_;
  boost::shared_ptr<CylindersDisplay> cylinders_;

private Q_SLOTS:
  // some property changed.
  void changedSlot();
  void changedEnableSlot();
};


/// Represents one piece of data in a particular link.
class PerLinkSubObj : public PerLinkSubObjBase
{
protected:
  PerLinkSubObj(PerLinkObjBase *base,
                DFLink *link);
  virtual Ogre::SceneNode *getSceneNode();

  DFLink *link_;
};


// class PerJointSubObj ... not implemented.


}

#endif
