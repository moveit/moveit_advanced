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

#include <moveit/robot_sphere_representation/robot_sphere_representation.h>
#include <moveit/robot_model/robot_model.h>

#include <geometric_shapes/shape_operations.h>

#include <vector>

collision_detection::RobotSphereRepresentation::RobotSphereRepresentation(
    boost::shared_ptr<const robot_model::RobotModel> robot_model)
  : robot_model_(robot_model)
{
}

void collision_detection::RobotSphereRepresentation::spheresFromSrdf(const srdf::Model *srdf)
{
  if (!srdf)
    srdf = robot_model_->getSRDF().get();

  centers_.clear();
  radii_.clear();

  for (std::vector<const robot_model::LinkModel*>::const_iterator lm = robot_model_->getLinkModels().begin() ;
       lm != robot_model_->getLinkModels().end() ;
       ++lm)
  {
    const std::string& link_name = (*lm)->getName();
    for (std::vector<srdf::Model::LinkSpheres>::const_iterator lsp = srdf->getLinkSphereApproximations().begin() ;; ++lsp)
    {

      // if no spheres in srdf, use a single bounding sphere
      if (lsp == srdf->getLinkSphereApproximations().end())
      {
        useBoundingSphereForLink(**lm);
        break;
      }

      // use the spheres from srdf
      if (lsp->link_ == link_name)
      {
        for (std::vector<srdf::Model::Sphere>::const_iterator sp = lsp->spheres_.begin() ;
             sp != lsp->spheres_.end() ;
             ++sp)
        {
          if (sp->radius_ > std::numeric_limits<double>::min())
          {
            Eigen::Vector3d center(sp->center_x_,
                                   sp->center_y_,
                                   sp->center_z_);
            centers_[link_name].push_back(center);
            radii_[link_name].push_back(sp->radius_);
          }
        }
        break;
      }
    }
  }
}

void collision_detection::RobotSphereRepresentation::genSpheres()
{
  genBoundingSpheres();
}

void collision_detection::RobotSphereRepresentation::genBoundingSpheres()
{
  centers_.clear();
  radii_.clear();

  for (std::vector<const robot_model::LinkModel*>::const_iterator lm = robot_model_->getLinkModels().begin() ;
       lm != robot_model_->getLinkModels().end() ;
       ++lm)
    useBoundingSphereForLink(**lm);
}

void collision_detection::RobotSphereRepresentation::useBoundingSphereForLink(const robot_model::LinkModel& lm)
{
  centers_[lm.getName()].clear();
  radii_[lm.getName()].clear();

  const shapes::ShapeConstPtr& shape = lm.getShape();
  if (!shape)
    return;

  double radius;
  Eigen::Vector3d center;
  shapes::computeShapeBoundingSphere(shape.get(), center, radius);
  if (radius > std::numeric_limits<double>::min())
  {
    centers_[lm.getName()].push_back(center);
    radii_[lm.getName()].push_back(radius);
  }
}

collision_detection::RobotSphereRepresentation::~RobotSphereRepresentation()
{ }
