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
#include <collision_distance_field_display/points_display.h>
#include <collision_distance_field_display/spheres_display.h>
#include <collision_distance_field_display/cylinders_display.h>
#include <collision_distance_field_display/color_cast.h>

#include <moveit/robot_sphere_representation/link_sphere_representation.h>

#include <moveit/collision_detection_distance_field/collision_robot_distance_field.h>

#include <rviz/properties/color_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/enum_property.h>
#include <rviz/properties/int_property.h>


namespace moveit_rviz_plugin
{
  // Draw Link Spheres from model
  class LinkObj_ModelLinkSpheres : public PerLinkSubObj
  {
  public:
    LinkObj_ModelLinkSpheres(PerLinkObjBase *base, DFLink *link) :
      PerLinkSubObj(base, link)
    {}

    static void addSelf(rviz::Property *parent, PerLinkObjList& per_link_objects)
    {
      per_link_objects.addVisObject(new PerLinkObj<LinkObj_ModelLinkSpheres>(
                                  parent,
                                  "Show Current SRDF sphere representation",
                                  "Show spheres used for DistanceField collision detection as stored in the SRDF.",
                                  QColor(0, 0, 255),
                                  0.5,
                                  PerLinkObjBase::SPHERES));
    }

    virtual void getGeom(bool& robot_relative, EigenSTL::vector_Vector3d& centers, std::vector<double>& radii)
    {
      robot_relative = false;
      link_->getLinkSpheres(centers, radii);
    }
  };
}

namespace moveit_rviz_plugin
{
  // Draw Link Spheres from RobotSphereRep
  class LinkObj_RepLinkSpheres : public PerLinkSubObj
  {
  public:
    LinkObj_RepLinkSpheres(PerLinkObjBase *base, DFLink *link) :
      PerLinkSubObj(base, link)
    {}

    static void addSelf(rviz::Property *parent, PerLinkObjList& per_link_objects)
    {
      per_link_objects.addVisObject(new PerLinkObj<LinkObj_RepLinkSpheres>(
                                  parent,
                                  "Show Generated Spheres",
                                  "Show spheres generated with RobotSphereRepresentation.",
                                  QColor(100, 100, 255),
                                  0.5,
                                  PerLinkObjBase::SPHERES));
    }

    virtual void getGeom(bool& robot_relative, EigenSTL::vector_Vector3d& centers, std::vector<double>& radii)
    {
      robot_relative = false;
      link_->getSphereRep()->getSpheres(centers, radii);
    }
  };
}

namespace moveit_rviz_plugin
{
  // Draw Link bounding sphere
  class LinkObj_BCyl : public PerLinkSubObj
  {
  public:
    LinkObj_BCyl(PerLinkObjBase *base, DFLink *link) :
      PerLinkSubObj(base, link)
    {}

    static void addSelf(rviz::Property *parent, PerLinkObjList& per_link_objects)
    {
      per_link_objects.addVisObject(new PerLinkObj<LinkObj_BCyl>(
                                  parent,
                                  "Show Link Bounding Cylinder",
                                  "Cylinder enclosing link.",
                                  QColor(128, 0, 255),
                                  0.5,
                                  PerLinkObjBase::CYLINDERS));
    }

    virtual void changed()
    {
      cylinders_.reset();
      if (!getBool())
        return;

      robot_relative_ = false;
      cylinders_.reset(new CylindersDisplay(getSceneNode(), base_->getColor()));

      bodies::BoundingCylinder cylinder;
      link_->getSphereRep()->getBoundingCylinder(cylinder);
      if (cylinder.radius > 0.0)
        cylinders_->addZCylinder(cylinder.pose, cylinder.radius, cylinder.length);
    }
  };
}

namespace moveit_rviz_plugin
{
  // Draw link's static distance field
  class LinkObj_StaticDF : public PerLinkSubObj
  {
  public:
    LinkObj_StaticDF(PerLinkObjBase *base, DFLink *link) :
      PerLinkSubObj(base, link)
    {}

    static void addSelf(rviz::Property *parent, PerLinkObjList& per_link_objects)
    {
      per_link_objects.addVisObject(new PerLinkObj<LinkObj_StaticDF>(
                                  parent,
                                  "Show Static Distance Field Points",
                                  "Show surface as described by distance field points on or just inside the surface..",
                                  QColor(255, 255, 0),
                                  1.0,
                                  PerLinkObjBase::POINTS,
                                  0.005));
    }

    virtual void getGeom(bool& robot_relative, EigenSTL::vector_Vector3d& centers, std::vector<double>& radii)
    {
      robot_relative = false;
      radii.clear();
      centers.clear();
      centers.reserve(100);
      const collision_detection::CollisionRobotDistanceField *crobot = link_->getDisplay()->getCollisionRobotDistanceField();
      if (crobot)
      {
        crobot->getStaticDistanceFieldPoints(
                        link_->getName(),
                        centers);
      }
    }
  };
}


void moveit_rviz_plugin::CollisionDistanceFieldDisplay::addPerLinkData(rviz::Property* df_collision_property,
                                                                       rviz::Property* sphere_gen_propety)
{
  per_link_objects_.reset(new PerLinkObjList());

  LinkObj_StaticDF::addSelf(df_collision_property, *per_link_objects_);

  addSphereGenProperties(sphere_gen_propety);

  LinkObj_RepLinkSpheres::addSelf(sphere_gen_propety, *per_link_objects_);
  LinkObj_BCyl::addSelf(sphere_gen_propety, *per_link_objects_);
  LinkObj_ModelLinkSpheres::addSelf(sphere_gen_propety, *per_link_objects_);

}


