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

#include <moveit/collision_detection_distance_field/collision_robot_distance_field.h>
#include "collision_robot_distance_field_inline.h"
#include <console_bridge/console.h>
#include <cassert>

collision_detection::CollisionRobotDistanceField::CollisionRobotDistanceField(
    const robot_model::RobotModelConstPtr &kmodel,
    double padding,
    double scale)
  : CollisionRobot(kmodel, padding, scale)
{
  initialize();
}

collision_detection::CollisionRobotDistanceField::CollisionRobotDistanceField(
    const CollisionRobotDistanceField &other)
  : CollisionRobot(other)
{
  initialize();
}

void collision_detection::CollisionRobotDistanceField::initialize()
{
  initParams();
  initSpheres();
  initLinkDF();
}

void collision_detection::CollisionRobotDistanceField::initParams()
{
  // The distance() queries will only be accurate up to this distance apart.
  MAX_DISTANCE_ = 0.25;

  SELF_COLLISION_RESOLUTION_ = 0.03;

  method_ = METHOD_INTRA_DF;
}

void collision_detection::CollisionRobotDistanceField::getMethods(std::vector<std::string>& methods) const
{
  methods.clear();
  methods.push_back("INTRA_DF");
  methods.push_back("SPHERES");
}

void collision_detection::CollisionRobotDistanceField::setMethod(const std::string& method)
{
  if (method == "INTRA_DF")
    method_ = METHOD_INTRA_DF;
  else if (method == "SPHERES")
    method_ = METHOD_SPHERES;
  else
  {
    logError("CollisionRobotDistanceField::setMethod() - bad method %s",method.c_str());
  }
}


void collision_detection::CollisionRobotDistanceField::updatedPaddingOrScaling(
    const std::vector<std::string> &links)
{
}

