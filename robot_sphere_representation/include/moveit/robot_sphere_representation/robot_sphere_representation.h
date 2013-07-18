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

#include <boost/shared_ptr.hpp>
#include <map>
#include <vector>
#include <moveit/robot_sphere_representation/method_enums.h>

namespace robot_model
{
class RobotModel;
class LinkModel;
}

namespace srdf
{
class Model;
}

namespace robot_sphere_representation
{
class LinkSphereRepresentation;
class Robot;


class RobotSphereRepresentation
{
public:
  RobotSphereRepresentation(boost::shared_ptr<const robot_model::RobotModel> robot_model);
  ~RobotSphereRepresentation();

  // Return the list of available sphere generation methods or quality methods.
  const std::vector<std::string>& getGenMethods() const;
  const std::vector<std::string>& getQualMethods() const;

  // Set what method to use to generate spheres.
  // Spheres are actually generated when they are requested from
  // LinkSphereRepresentation::getSpheres() (and any other methods that need to
  // generate the spheres in order to operate).
  void setGenMethod(const std::string& gen_method);
  void setGenMethod(GenMethod gen_method = GenMethod::DEFAULT);

  // Set what method to use to measure quality when generating spheres
  void setQualMethod(const std::string& qual_method);
  void setQualMethod(QualMethod qual_method = QualMethod::DEFAULT);

  // set distance field resolution to use for calculations.
  void setResolution(double resolution);
  double getResolution() { return resolution_; }

  // set tolerance to use.  For methods that pay attention to tolerance,
  // spheres will stick out at most <tolerance> from the surface
  // (+/- resolution).
  // Tolerance can also be set per-link.  Calling this method sets the
  // tolerance for all links to the same value.
  void setTolerance(double tolerance);

  // how many spheres do we want for each link.
  // Usually this would be set per-link, not globally here.
  // For many GenMethods this is ignored.
  void setRequestedNumSpheres(int nspheres);

  // read spheres from srdf
  // (by default the one associated with RobotModel)
  void copySrdfSpheres(const srdf::Model *srdf = NULL);

  // save to srdf file.  Clobbers file.  Return true on success.
  bool saveToSrdfFile(const std::string& filename) const;

  // generate all spheres.  This is never needed, but it forces all Links to
  // update themselves which can make things faster later.
  void genSpheresForAllLinks() const;

  const Robot* getSphereCalcRobot() const;
  Robot* getSphereCalcRobot();

  // access
  const boost::shared_ptr<const robot_model::RobotModel>& getRobotModel() const { return robot_model_; }
  const std::map<std::string, LinkSphereRepresentation*>& getLinks() const { return links_; }
  LinkSphereRepresentation* getLink(const std::string& link_name) const;

  void ensureSphereCalcRobot() const
  {
    if (sphere_calc_robot_dirty_)
      updateSphereCalcRobot();
  }

  void invalidateSphereCalc() { sphere_calc_robot_dirty_ = true; }


private:
  void invalidateSpheresForAllLinks();

  // update sphere_calc_robot_.  Should only be called by ensureSphereCalcRobot()
  void updateSphereCalcRobot() const;



  boost::shared_ptr<const robot_model::RobotModel> robot_model_;

  std::map<std::string, LinkSphereRepresentation*> links_;

  // resolution for SphereCalc distance field 
  double resolution_;

  mutable boost::shared_ptr<Robot> sphere_calc_robot_;
  mutable bool sphere_calc_robot_dirty_;
};

}

inline const robot_sphere_representation::Robot* robot_sphere_representation::RobotSphereRepresentation::getSphereCalcRobot() const
{
  ensureSphereCalcRobot();
  return &*sphere_calc_robot_;
}

inline robot_sphere_representation::Robot* robot_sphere_representation::RobotSphereRepresentation::getSphereCalcRobot()
{
  ensureSphereCalcRobot();
  return &*sphere_calc_robot_;
}


#endif

