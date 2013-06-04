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
#include <moveit/robot_sphere_representation/sphere_rep.h>
#include <moveit/robot_model/robot_model.h>

#include <geometric_shapes/shape_operations.h>
#include <geometric_shapes/bodies.h>
#include <geometric_shapes/body_operations.h>

#include <assert.h>

robot_sphere_representation::LinkSphereRepresentation::LinkSphereRepresentation(
      RobotSphereRepresentation *robot, 
      const robot_model::LinkModel *link_model)
  : robot_(robot)
  , link_model_(link_model)
  , dirty_(true)
  , method_(GenMethod::DEFAULT)
{
}

robot_sphere_representation::LinkSphereRepresentation::~LinkSphereRepresentation()
{}



void robot_sphere_representation::LinkSphereRepresentation::setMethod(const std::string& method)
{
  setMethod(robot_->getMethodValue(method));
}

void robot_sphere_representation::LinkSphereRepresentation::setMethod(GenMethod::GenMethod method)
{
  // GM_SRDF_EXT can only be set by calling copySrdfSpheres()
  if (method == GenMethod::SRDF_EXT && method_ != GenMethod::SRDF_EXT)
    method = GenMethod::SRDF;

  if (method != method_)
  {
    dirty_ = true;
    method_ = method;

    // if method is srdf, do it right away since srdf could change.
    if (method == GenMethod::SRDF)
      useSrdfSpheres();
  }
}

void robot_sphere_representation::LinkSphereRepresentation::genSpheres() const
{
  if (!dirty_)
    return;

  switch(method_)
  {
    #define x(e,f,n) \
      case GenMethod::e: f(); break;
    collision_detection__RobotSphereRepresentation__GenMethods__strings(x)
    #undef x
    case GenMethod::DEFAULT:
    default:
      useSrdfSpheres();
      break;
  }
  assert(dirty_ == false);
}

void robot_sphere_representation::LinkSphereRepresentation::copySrdfSpheres(const srdf::Model *srdf)
{
  if (!srdf)
    srdf = robot_->getRobotModel()->getSRDF().get();
  if (srdf == robot_->getRobotModel()->getSRDF().get())
    method_ = GenMethod::SRDF;
  else
    method_ = GenMethod::SRDF_EXT;
  useSrdfSpheres(srdf);
}

void robot_sphere_representation::LinkSphereRepresentation::useSrdfSpheresExt() const
{
  assert(dirty_ == false);
}

void robot_sphere_representation::LinkSphereRepresentation::useSrdfSpheres(const srdf::Model *srdf) const
{
  if (!srdf)
    srdf = robot_->getRobotModel()->getSRDF().get();

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
      useBoundingSpheres();
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
  dirty_ = false;
}

void robot_sphere_representation::LinkSphereRepresentation::useBoundingSpheres() const
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
  dirty_ = false;
}

void robot_sphere_representation::LinkSphereRepresentation::getSpheres(EigenSTL::vector_Vector3d& centers, std::vector<double>& radii) const
{
  genSpheres();
  centers = centers_;
  radii = radii_;
}

const boost::shared_ptr<const bodies::Body>& robot_sphere_representation::LinkSphereRepresentation::getBody() const
{
  if (!body_)
  {
    static const shapes::Sphere empty_shape(0.0);
    const shapes::Shape *shape = link_model_->getShape().get();
    if (!shape)
      shape = &empty_shape;
    body_.reset(bodies::createBodyFromShape(shape));
    body_const_ = body_;
  }
  return body_const_;
}

void robot_sphere_representation::LinkSphereRepresentation::getBoundingCylinder(bodies::BoundingCylinder& cylinder) const
{
  getBody()->computeBoundingCylinder(cylinder);
}

void robot_sphere_representation::LinkSphereRepresentation::updateSphereRepLink() const
{
  sphere_rep_link_ = robot_->getSphereRepRobot()->getLink(getName());
  if (method_ != GenMethod::SRDF_EXT)
    dirty_ = true;
}

