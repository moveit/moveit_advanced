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

#ifndef MOVEIT_ROBOT_SPHERE_REPRESENTATION_ROBOT_SPHERE_REPRESENTATION_
#define MOVEIT_ROBOT_SPHERE_REPRESENTATION_ROBOT_SPHERE_REPRESENTATION_

//#include <eigen_stl_containers/eigen_stl_containers.h>
#include <boost/shared_ptr.hpp>
#include <map>

namespace robot_model
{
class RobotModel;
class LinkModel;
}

namespace srdf
{
class Model;
}

namespace collision_detection
{
class LinkSphereRepresentation;

class RobotSphereRepresentation
{
public:
  RobotSphereRepresentation(boost::shared_ptr<const robot_model::RobotModel> robot_model);
  ~RobotSphereRepresentation();

  // generate spheres for each link. 
  // Discards old spheres (if any).
  // An entry will be added to centers_ and radii_ for every link in the model.
  // Links with no The entry will be empty for links with no collision geometry.
  void genSpheres();

  // read spheres from the srdf
  // (by default the one associated with RobotModel)
  void useSrdfSpheres(const srdf::Model *srdf = NULL);

  // use one bounding sphere per link.
  void useBoundingSpheres();

  const boost::shared_ptr<const robot_model::RobotModel>& getRobotModel() const { return robot_model_; }

  // access the links
  const std::map<std::string, LinkSphereRepresentation*>& getLinks() const { return links_; }

  // Get link info given link name
  LinkSphereRepresentation* getLink(const std::string& link_name) const;


private:
  // generate a single sphere for each link that bounds the entire link.
  void genBoundingSpheres();

  boost::shared_ptr<const robot_model::RobotModel> robot_model_;

  std::map<std::string, LinkSphereRepresentation*> links_;
};

}

#endif

