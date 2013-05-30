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

#include <rviz/properties/color_property.h>
#include <rviz/properties/float_property.h>


namespace moveit_rviz_plugin
{
// Draw Link Spheres (spheres used for collision)
class LinkObj_LinkSpheres : public PerLinkSubObj
{
public:
  LinkObj_LinkSpheres(PerPartObjBase *base, DFLink *link) :
    PerLinkSubObj(base, link)
  {}

  static void addSelf(rviz::Property *parent, PerPartObjList& per_link_objects)
  {
    per_link_objects.addVisObject(new PerLinkObj<LinkObj_LinkSpheres>(
                                parent,
                                "Show Link Collision spheres",
                                "Show spheres used for DistanceField collision detection.",
                                QColor(0, 0, 255),
                                0.5,
                                PerPartObjBase::SPHERES));
  }

  virtual void getGeom(bool& robot_relative, EigenSTL::vector_Vector3d& centers, std::vector<double>& radii)
  {
    robot_relative = true;
    link_->getLinkSpheres(centers, radii);
  }
};
}



void moveit_rviz_plugin::CollisionDistanceFieldDisplay::add_per_link_data(rviz::Property* parent_property)
{
  per_link_objects_.reset(new PerPartObjList());

  LinkObj_LinkSpheres::addSelf(parent_property, *per_link_objects_);
}

