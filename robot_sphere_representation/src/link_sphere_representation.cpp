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
#include <moveit/robot_sphere_representation/sphere_calc.h>
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
  , tolerance_(1.0)
  , requested_nspheres_(10)
{
}

robot_sphere_representation::LinkSphereRepresentation::~LinkSphereRepresentation()
{}


void robot_sphere_representation::LinkSphereRepresentation::setRequestedNumSpheres(int nspheres)
{
  nspheres = nspheres >= 1 ? nspheres : 1;
  if (requested_nspheres_ != nspheres)
  {
    requested_nspheres_ = nspheres;
    dirty_ = true;
  }
}

void robot_sphere_representation::LinkSphereRepresentation::setTolerance(double tolerance)
{
  tolerance = tolerance>0.0 ? tolerance : 1.0;
  if (tolerance_ != tolerance)
  {
    tolerance_ = tolerance;
    dirty_ = true;
  }
}

void robot_sphere_representation::LinkSphereRepresentation::setGenMethod(const std::string& gen_method)
{
  setGenMethod(GenMethod(gen_method));
}

void robot_sphere_representation::LinkSphereRepresentation::setGenMethod(GenMethod gen_method)
{
  if (!gen_method.isValid() || gen_method_ == gen_method)
    return;

  gen_method_ = gen_method;
  invalidateSpheres();
}

void robot_sphere_representation::LinkSphereRepresentation::setQualMethod(const std::string& qual_method)
{
  setQualMethod(QualMethod(qual_method));
}

void robot_sphere_representation::LinkSphereRepresentation::setQualMethod(QualMethod qual_method)
{
  if (!qual_method.isValid() || qual_method_ == qual_method)
    return;

  qual_method_ = qual_method;
  invalidateSpheres();
}

void robot_sphere_representation::LinkSphereRepresentation::getSpheres(
        EigenSTL::vector_Vector3d& centers, 
        std::vector<double>& radii) const
{
  genSpheres();
  centers = centers_;
  radii = radii_;
}

void robot_sphere_representation::LinkSphereRepresentation::genSpheres() const
{
  if (!dirty_)
    return;

  switch(gen_method_)
  {
  case GenMethod::SRDF:
    useSrdfSpheres();
    break;

  case GenMethod::SRDF_EXT:
    break; // nothing to do

  case GenMethod::BOUNDING_SPHERES:
    genSpheresUsingBoundingSpheres();
    break;

  default:
    genSpheresUsingSphereCalc();
    break;
  }
  assert(dirty_ == false);
}

void robot_sphere_representation::LinkSphereRepresentation::copySrdfSpheres(const srdf::Model *srdf)
{
  if (!srdf)
    srdf = robot_->getRobotModel()->getSRDF().get();

  if (srdf == robot_->getRobotModel()->getSRDF().get())
    gen_method_ = GenMethod::SRDF;
  else
    gen_method_ = GenMethod::SRDF_EXT;

  useSrdfSpheres(srdf);
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
      genSpheresUsingBoundingSpheres();
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

void robot_sphere_representation::LinkSphereRepresentation::genSpheresUsingBoundingSpheres() const
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

void robot_sphere_representation::LinkSphereRepresentation::updateSphereCalcLink() const
{
  sphere_calc_link_ = robot_->getSphereCalcRobot()->getLink(getName());
  if (gen_method_ != GenMethod::SRDF_EXT)
    dirty_ = true;
}

const robot_sphere_representation::SphereCalc* robot_sphere_representation::LinkSphereRepresentation::getSphereCalc() const
{
  robot_->ensureSphereCalcRobot();

  return sphere_calc_link_->getSphereCalc(
                                    requested_nspheres_,
                                    gen_method_,
                                    tolerance_,
                                    qual_method_);
}

void robot_sphere_representation::LinkSphereRepresentation::genSpheresUsingSphereCalc() const
{
  const SphereCalc* sphere_rep = getSphereCalc();
  if (sphere_rep)
  {
    radii_ = sphere_rep->getSphereRadii();
    centers_ = sphere_rep->getSphereCenters();
    sphere_calc_link_->transformRobotToLink(centers_.begin(), centers_.end());
  }
  else
  {
    radii_.clear();
    centers_.clear();
  }
  dirty_ = false;
}


