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
#include <moveit/setup_assistant/tools/srdf_writer.h>

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

const std::vector<std::string>& robot_sphere_representation::RobotSphereRepresentation::getGenMethods() const
{
  return GenMethod::getNames();
}

const std::vector<std::string>& robot_sphere_representation::RobotSphereRepresentation::getQualMethods() const
{
  return QualMethod::getNames();
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

void robot_sphere_representation::RobotSphereRepresentation::setGenMethod(const std::string& gen_method)
{
  setGenMethod(GenMethod(gen_method));
}

void robot_sphere_representation::RobotSphereRepresentation::setGenMethod(GenMethod gen_method)
{
  std::map<std::string, LinkSphereRepresentation*>::const_iterator lsr = links_.begin();
  std::map<std::string, LinkSphereRepresentation*>::const_iterator lsr_end = links_.end();
  for ( ; lsr != lsr_end ; ++lsr )
    lsr->second->setGenMethod(gen_method);
}

void robot_sphere_representation::RobotSphereRepresentation::setQualMethod(const std::string& qual_method)
{
  setQualMethod(QualMethod(qual_method));
}

void robot_sphere_representation::RobotSphereRepresentation::setRequestedNumSpheres(int nspheres)
{
  std::map<std::string, LinkSphereRepresentation*>::const_iterator lsr = links_.begin();
  std::map<std::string, LinkSphereRepresentation*>::const_iterator lsr_end = links_.end();
  for ( ; lsr != lsr_end ; ++lsr )
    lsr->second->setRequestedNumSpheres(nspheres);
}

void robot_sphere_representation::RobotSphereRepresentation::setQualMethod(QualMethod qual_method)
{
  std::map<std::string, LinkSphereRepresentation*>::const_iterator lsr = links_.begin();
  std::map<std::string, LinkSphereRepresentation*>::const_iterator lsr_end = links_.end();
  for ( ; lsr != lsr_end ; ++lsr )
    lsr->second->setQualMethod(qual_method);
}

void robot_sphere_representation::RobotSphereRepresentation::invalidateSpheresForAllLinks()
{
  std::map<std::string, LinkSphereRepresentation*>::iterator lsr = links_.begin();
  std::map<std::string, LinkSphereRepresentation*>::iterator lsr_end = links_.end();
  for ( ; lsr != lsr_end ; ++lsr )
    lsr->second->invalidateSpheres();
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
  if (resolution_ != resolution)
  {
    sphere_rep_robot_dirty_ = true;
    resolution_ = resolution;
  }
}

void robot_sphere_representation::RobotSphereRepresentation::setTolerance(double tolerance)
{
  std::map<std::string, LinkSphereRepresentation*>::const_iterator lsr = links_.begin();
  std::map<std::string, LinkSphereRepresentation*>::const_iterator lsr_end = links_.end();
  for ( ; lsr != lsr_end ; ++lsr )
    lsr->second->setTolerance(tolerance);
}

void robot_sphere_representation::RobotSphereRepresentation::genSpheresForAllLinks() const
{
  std::map<std::string, LinkSphereRepresentation*>::const_iterator lsr = links_.begin();
  std::map<std::string, LinkSphereRepresentation*>::const_iterator lsr_end = links_.end();
  for ( ; lsr != lsr_end ; ++lsr )
    lsr->second->genSpheres();
}

bool robot_sphere_representation::RobotSphereRepresentation::saveToSrdfFile(const std::string& srdf_filename) const
{
  genSpheresForAllLinks();

  // Get an SRDFWriter with the data from the current RobotModel
  moveit_setup_assistant::SRDFWriter writer;
  writer.initModel( *robot_model_->getURDF(), *robot_model_->getSRDF() );

  // Delete any existing spheres
  writer.link_sphere_approximations_.clear();

  // Insert generated spheres into SRDFWriter
  EigenSTL::vector_Vector3d centers;
  std::vector<double> radii;
  std::map<std::string, LinkSphereRepresentation*>::const_iterator lsr = links_.begin();
  std::map<std::string, LinkSphereRepresentation*>::const_iterator lsr_end = links_.end();
  for ( ; lsr != lsr_end ; ++lsr )
  {
    centers.clear();
    radii.clear();
    lsr->second->getSpheres(centers, radii);

    if (centers.empty())
    {
      // a link with no geometry is represented by a single radius=0 sphere
      radii.clear();
      radii.push_back(0);
      centers.push_back(Eigen::Vector3d(0,0,0));
    }

    srdf::Model::LinkSpheres lsp;
    lsp.link_ = lsr->first;

    for ( std::size_t i = 0 ; i < centers.size() ; ++i )
    {
      srdf::Model::Sphere sphere;
      sphere.center_x_ = centers[i].x();
      sphere.center_y_ = centers[i].y();
      sphere.center_z_ = centers[i].z();
      sphere.radius_ = radii[i];

      lsp.spheres_.push_back(sphere);
    }

    writer.link_sphere_approximations_.push_back(lsp);
  }

  // write the SRDF to file and return true on success.
  return writer.writeSRDF(srdf_filename);
}


