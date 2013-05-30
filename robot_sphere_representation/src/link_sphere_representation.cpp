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
 *   * Neither the name of the Willow Garage nor the names of its
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

#include <moveit/robot_sphere_representation/link_sphere_representation.h>
#include <moveit/robot_sphere_representation/robot_sphere_representation.h>
#include <moveit/robot_model/robot_model.h>

#include <geometric_shapes/shape_operations.h>

collision_detection::LinkSphereRepresentation::LinkSphereRepresentation(
      RobotSphereRepresentation *parent, 
      const robot_model::LinkModel *link_model)
  : parent_(parent)
  , link_model_(link_model)
{
}

collision_detection::LinkSphereRepresentation::~LinkSphereRepresentation()
{}



void collision_detection::LinkSphereRepresentation::useSrdfSpheres(const srdf::Model *srdf)
{
  if (!srdf)
    srdf = parent_->getRobotModel()->getSRDF().get();

  centers_.clear();
  radii_.clear();

  const std::string& link_name = getName();

  std::vector<srdf::Model::LinkSpheres>::const_iterator lsp = srdf->getLinkSphereApproximations().begin();
  std::vector<srdf::Model::LinkSpheres>::const_iterator lsp_end = srdf->getLinkSphereApproximations().end();
  for ( ;; ++lsp )
  {

    // if no spheres in srdf, use a single bounding sphere
    if (lsp == lsp_end)
    {
      useBoundingSphere();
      break;
    }

    // use the spheres from srdf
    if (lsp->link_ == link_name)
    {
      std::vector<srdf::Model::Sphere>::const_iterator sp = lsp->spheres_.begin();
      std::vector<srdf::Model::Sphere>::const_iterator sp_end = lsp->spheres_.end();
      for ( ; sp != sp_end ; ++sp )
      {
        if (sp->radius_ > std::numeric_limits<double>::min())
        {
          Eigen::Vector3d center(sp->center_x_,
                                 sp->center_y_,
                                 sp->center_z_);
          centers_.push_back(center);
          radii_.push_back(sp->radius_);
        }
      }
      break;
    }
  }
}

void collision_detection::LinkSphereRepresentation::useBoundingSphere()
{
  centers_.clear();
  radii_.clear();

  const shapes::ShapeConstPtr& shape = link_model_->getShape();
  if (!shape)
    return;

  double radius;
  Eigen::Vector3d center;
  shapes::computeShapeBoundingSphere(shape.get(), center, radius);
  if (radius > std::numeric_limits<double>::min())
  {
    centers_.push_back(center);
    radii_.push_back(radius);
  }
}
