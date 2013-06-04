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

#ifndef MOVEIT_ROBOT_SPHERE_REPRESENTATION_LINK_SPHERE_REPRESENTATION_
#define MOVEIT_ROBOT_SPHERE_REPRESENTATION_LINK_SPHERE_REPRESENTATION_

#include <moveit/robot_sphere_representation/robot_sphere_representation.h>
#include <eigen_stl_containers/eigen_stl_containers.h>
#include <boost/shared_ptr.hpp>
#include <map>
#include <vector>

#include <moveit/robot_model/link_model.h>

namespace robot_model
{
class RobotModel;
class LinkModel;
}

namespace srdf
{
class Model;
}

namespace bodies
{
class Body;
class BoundingCylinder;
}

namespace robot_sphere_representation
{
class Link;

class LinkSphereRepresentation
{
public:
  LinkSphereRepresentation(RobotSphereRepresentation *robot, const robot_model::LinkModel *link_model);
  ~LinkSphereRepresentation();

  const std::string& getName() const { return link_model_->getName(); }

  // Set what method to use to generate spheres.
  // Method names available from RobotSphereRepresentation::getGenMethods()
  // Note: The spheres are not actually generated until needed (e.g. getSpheres() generates them)
  void setMethod(const std::string& method);
  void setMethod(GenMethod::GenMethod method = GenMethod::DEFAULT);

  // get the spheres.  Calculates their values (based on currently set parameters) if necessary.
  void getSpheres(EigenSTL::vector_Vector3d& centers, std::vector<double>& radii) const;

  // copy spheres from srdf.
  void copySrdfSpheres(const srdf::Model *srdf = NULL);

  // get a cylinder that approximates the link.  Pose is center of cylinder in
  // link collision frame.  z is major axis.
  void getBoundingCylinder(bodies::BoundingCylinder& cylinder) const;

  // get a body representing the link in the link collision frame
  const boost::shared_ptr<const bodies::Body>& getBody() const;

  const RobotSphereRepresentation* getRobot() const { return robot_; }
  RobotSphereRepresentation* getRobot() { return robot_; }

  const Link* getSphereRepLink() const;
  Link* getSphereRepLink();

  // Actually generate the spheres.  This is not needed as it is called from
  // getSpheres() when required.  However, calling it will ensure that a future
  // call to getSpheres() returns quickly. 
  // This function is const because it only changes mutable data.  Conceptually
  // the spheres themselves are "changed" only when parameters change.  This
  // function is just updating them.
  void genSpheres() const;

  // called by RobotSphereRepresentation to update sphere_rep_link_
  void updateSphereRepLink() const;

private:

  // Use a single sphere for a link that bounds the entire link.
  // If there is no collision geometry this creates an empty entry for this
  // link.
  void useBoundingSpheres() const;

  // copy spheres from srdf.
  void useSrdfSpheres(const srdf::Model *srdf = NULL) const;

  // This does nothing.  If method is GenMethod::SRDF_EXT then dirty is always false.
  // To set method to GenMethod::SRDF_EXT call 
  void useSrdfSpheresExt() const;

  RobotSphereRepresentation *robot_;
  const robot_model::LinkModel *link_model_;

  // the spheres that bound this link.
  mutable EigenSTL::vector_Vector3d centers_;
  mutable std::vector<double> radii_;

  // method set with genSpheres().
  GenMethod::GenMethod method_;
  mutable bool dirty_;

  // a body representing the link in its own collision frame
  mutable boost::shared_ptr<bodies::Body> body_;
  mutable boost::shared_ptr<const bodies::Body> body_const_;

  mutable Link *sphere_rep_link_;
};

}

inline const robot_sphere_representation::Link* robot_sphere_representation::LinkSphereRepresentation::getSphereRepLink() const
{
  robot_->ensureSphereRepRobot();
  return sphere_rep_link_;
}

inline robot_sphere_representation::Link* robot_sphere_representation::LinkSphereRepresentation::getSphereRepLink()
{
  robot_->ensureSphereRepRobot();
  return sphere_rep_link_;
}


#endif

