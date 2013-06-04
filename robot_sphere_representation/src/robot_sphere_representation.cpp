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
#include <moveit/robot_sphere_representation/link_sphere_representation.h>
#include <moveit/robot_sphere_representation/sphere_rep.h>
#include <moveit/robot_model/robot_model.h>

robot_sphere_representation::RobotSphereRepresentation::RobotSphereRepresentation(
      boost::shared_ptr<const robot_model::RobotModel> robot_model)
  : robot_model_(robot_model)
  , sphere_rep_robot_dirty_(true)
  , resolution_(0.03)
{
  std::vector<const robot_model::LinkModel*>::const_iterator lm = robot_model_->getLinkModels().begin();
  std::vector<const robot_model::LinkModel*>::const_iterator lm_end = robot_model_->getLinkModels().end();
  for ( ; lm != lm_end ; ++lm )
  {
    LinkSphereRepresentation *lsr = new LinkSphereRepresentation(this, *lm);
    links_.insert( std::pair<std::string, LinkSphereRepresentation*>( lsr->getName(), lsr ) );
  }

  // initialize method_names_ and method_map_
  #define x(e,f,n) \
    method_names_.push_back(n); \
    method_map_.insert( std::pair<std::string, GenMethod::GenMethod>( n, GenMethod::e ) );
  collision_detection__RobotSphereRepresentation__GenMethods__strings(x)
  #undef x
}

robot_sphere_representation::RobotSphereRepresentation::~RobotSphereRepresentation()
{
  std::map<std::string, LinkSphereRepresentation*>::iterator lsr = links_.begin();
  std::map<std::string, LinkSphereRepresentation*>::iterator lsr_end = links_.end();
  for ( ; lsr != lsr_end ; ++lsr )
  {
    delete lsr->second;
  }
}

void robot_sphere_representation::RobotSphereRepresentation::copySrdfSpheres(const srdf::Model *srdf)
{
  if (!srdf)
    srdf = getRobotModel()->getSRDF().get();

  std::map<std::string, LinkSphereRepresentation*>::iterator lsr = links_.begin();
  std::map<std::string, LinkSphereRepresentation*>::iterator lsr_end = links_.end();
  for ( ; lsr != lsr_end ; ++lsr )
    lsr->second->copySrdfSpheres(srdf);
}

robot_sphere_representation::LinkSphereRepresentation* robot_sphere_representation::RobotSphereRepresentation::getLink(
      const std::string& link_name) const
{
  std::map<std::string, LinkSphereRepresentation*>::const_iterator it = links_.find(link_name);
  if (it != links_.end())
    return it->second;
  else
    return NULL;
}

robot_sphere_representation::GenMethod::GenMethod robot_sphere_representation::RobotSphereRepresentation::getMethodValue(const std::string& method) const
{
  std::map<std::string, GenMethod::GenMethod>::const_iterator it = method_map_.find(method);
  if (it != method_map_.end())
    return it->second;
  else
    return GenMethod::DEFAULT;
}

void robot_sphere_representation::RobotSphereRepresentation::setMethod(const std::string& method)
{
  setMethod(getMethodValue(method));
}

void robot_sphere_representation::RobotSphereRepresentation::setMethod(GenMethod::GenMethod method)
{
  std::map<std::string, LinkSphereRepresentation*>::iterator lsr = links_.begin();
  std::map<std::string, LinkSphereRepresentation*>::iterator lsr_end = links_.end();
  for ( ; lsr != lsr_end ; ++lsr )
    lsr->second->setMethod(method);
}

void robot_sphere_representation::RobotSphereRepresentation::updateSphereRepRobot() const
{
  sphere_rep_robot_.reset(new Robot(robot_model_, resolution_));

  sphere_rep_robot_dirty_ = false;

  std::map<std::string, LinkSphereRepresentation*>::const_iterator lsr = links_.begin();
  std::map<std::string, LinkSphereRepresentation*>::const_iterator lsr_end = links_.end();
  for ( ; lsr != lsr_end ; ++lsr )
    lsr->second->updateSphereRepLink();
}

void robot_sphere_representation::RobotSphereRepresentation::setResolution(double resolution)
{
  sphere_rep_robot_dirty_ = true;
  resolution_ = resolution;
}

void robot_sphere_representation::RobotSphereRepresentation::setTolerance(double tolerance)
{
  sphere_rep_robot_dirty_ = true;
  tolerance_ = tolerance;
}



