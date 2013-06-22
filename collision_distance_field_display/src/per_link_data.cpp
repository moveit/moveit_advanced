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
#include <collision_distance_field_display/shapes_display.h>
#include <collision_distance_field_display/color_cast.h>
#include "dfexamine.h"

#include <moveit/robot_sphere_representation/link_sphere_representation.h>
#include <moveit/robot_sphere_representation/bounding_sphere.h>

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
                                  "Show Link Collision Spheres",
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
  // Draw Link Spheres from model
  class LinkObj_ModelLinkBSpheres : public PerLinkSubObj
  {
  public:
    LinkObj_ModelLinkBSpheres(PerLinkObjBase *base, DFLink *link) :
      PerLinkSubObj(base, link)
    {}

    static void addSelf(rviz::Property *parent, PerLinkObjList& per_link_objects)
    {
      per_link_objects.addVisObject(new PerLinkObj<LinkObj_ModelLinkBSpheres>(
                                  parent,
                                  "Show Link Bounding Sphere",
                                  "Show Link's single Bounding Sphere.",
                                  QColor(128, 128, 255),
                                  0.5,
                                  PerLinkObjBase::SPHERES));
    }

    virtual void getGeom(bool& robot_relative, EigenSTL::vector_Vector3d& centers, std::vector<double>& radii)
    {
      robot_relative = false;
      centers.resize(1);
      radii.resize(1);
      link_->getLinkBoundingSphere(centers[0], radii[0]);
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
      shapes_.reset();
      if (!getBool())
        return;

      robot_relative_ = false;
      shapes_.reset(new ShapesDisplay(getSceneNode(), base_->getColor()));

      bodies::BoundingCylinder cylinder;
      link_->getSphereRep()->getBoundingCylinder(cylinder);
      if (cylinder.radius > 0.0)
        shapes_->addZCylinder(cylinder.pose, cylinder.radius, cylinder.length);
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
                                  "Show Static Distance Field",
                                  "Show surface as described by distance field points on or just inside the surface.",
                                  QColor(255, 255, 0),
                                  1.0,
                                  PerLinkObjBase::POINTS,
                                  0.005));
    }

    virtual void changed()
    {
      df_examine_.reset();
      PerLinkSubObjBase::changed();
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
        df_examine_.reset(link_->getDisplay()->examineDF("StaticDF for ",
                                                         link_->getName().c_str(),
                                                         crobot->getStaticDistanceField(link_->getName()),
                                                         link_->getCollisionNode(),
                                                         Eigen::Affine3d::Identity()));
      }
    }
  private:
    boost::shared_ptr<moveit_rviz_plugin::CollisionDistanceFieldDisplay::DFExamine> df_examine_;
  };
}

namespace moveit_rviz_plugin
{
  // Draw link's static distance field points (points used to generate static DF)
  class LinkObj_StaticDFPoints : public PerLinkSubObj
  {
  public:
    LinkObj_StaticDFPoints(PerLinkObjBase *base, DFLink *link) :
      PerLinkSubObj(base, link)
    {}

    static void addSelf(rviz::Property *parent, PerLinkObjList& per_link_objects)
    {
      per_link_objects.addVisObject(new PerLinkObj<LinkObj_StaticDFPoints>(
                                  parent,
                                  "Show Points in link used to generate Static Distance Field",
                                  "Original points inside the link.",
                                  QColor(255, 0, 0),
                                  1.0,
                                  PerLinkObjBase::POINTS,
                                  0.006));
    }

    virtual void getGeom(bool& robot_relative, EigenSTL::vector_Vector3d& centers, std::vector<double>& radii)
    {
      robot_relative = false;
      radii.clear();
      centers.clear();
      const collision_detection::CollisionRobotDistanceField *crobot = link_->getDisplay()->getCollisionRobotDistanceField();
      if (crobot)
      {
        const collision_detection::StaticDistanceField *df = crobot->getStaticDistanceField(link_->getName());
        if (df)
          centers = df->getPoints();
      }
    }
  };
}

namespace moveit_rviz_plugin
{
  // Draw link's bounding sphere based on df points
  class LinkObj_SDFBSphere : public PerLinkSubObj
  {
  public:
    LinkObj_SDFBSphere(PerLinkObjBase *base, DFLink *link)
      : PerLinkSubObj(base, link)
    { }

    static void addSelf(rviz::Property *parent, PerLinkObjList& per_link_objects)
    {
      per_link_objects.addVisObject(new PerLinkObj<LinkObj_SDFBSphere>(
                                  parent,
                                  "Show tight bounding sphere around df points",
                                  "This demonstrates the tight bounding sphere code",
                                  QColor(255, 0, 0),
                                  0.5,
                                  PerLinkObjBase::SPHERES));
    }

    virtual void changed()
    {
      shapes_.reset();
      centers_.clear();
      radii_.clear();

      if (!getBool())
        return;

      robot_relative_ = false;
      shapes_.reset(new ShapesDisplay(getSceneNode(), base_->getColor(), base_->getSize()));

      const collision_detection::CollisionRobotDistanceField *crobot = link_->getDisplay()->getCollisionRobotDistanceField();
      if (crobot)
      {
        const collision_detection::StaticDistanceField *df = crobot->getStaticDistanceField(link_->getName());
        if (df)
        {
          EigenSTL::vector_Vector3d points;
          crobot->getStaticDistanceFieldPoints(
                          link_->getName(),
                          points);

          Eigen::Vector3d center;
          double radius;
          robot_sphere_representation::generateBoundingSphere(
            points,
            center,
            radius);
          shapes_->addSphere(center, radius);

        }
      }
    }
  };
}


void moveit_rviz_plugin::CollisionDistanceFieldDisplay::addPerLinkData(rviz::Property* df_collision_property,
                                                                       rviz::Property* sphere_gen_propety)
{
  per_link_objects_.reset(new PerLinkObjList());

  LinkObj_StaticDF::addSelf(df_collision_property, *per_link_objects_);
  LinkObj_StaticDFPoints::addSelf(df_collision_property, *per_link_objects_);
  LinkObj_ModelLinkSpheres::addSelf(sphere_gen_propety, *per_link_objects_);
  LinkObj_ModelLinkBSpheres::addSelf(sphere_gen_propety, *per_link_objects_);

  addSphereGenProperties(sphere_gen_propety);

  LinkObj_RepLinkSpheres::addSelf(sphere_gen_propety, *per_link_objects_);
  LinkObj_BCyl::addSelf(sphere_gen_propety, *per_link_objects_);
  LinkObj_SDFBSphere::addSelf(sphere_gen_propety, *per_link_objects_);
}


