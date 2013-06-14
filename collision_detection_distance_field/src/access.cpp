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

const int collision_detection::CollisionRobotDistanceField::linkNameToIndex(const std::string& link_name) const
{
  std::map<std::string,int>::const_iterator it = link_name_to_link_index_.find(link_name);
  if (it != link_name_to_link_index_.end())
  {
    return it->second;
  }
  else
  {
    logError("linkNameToIndex() could not find nonexistant link %s",link_name.c_str());
    return -1;
  }
}

const collision_detection::StaticDistanceField* collision_detection::CollisionRobotDistanceField::getStaticDistanceField(
                      const std::string& link_name) const
{
  const DFLink *link = linkNameToDFLink(link_name);
  return link ? &link->df_ : NULL;
}

void collision_detection::CollisionRobotDistanceField::getStaticDistanceFieldPoints(
      const std::string& link_name,
      EigenSTL::vector_Vector3d& points,
      double min_dist,
      double max_dist,
      bool resolution_relative) const
{
  points.clear();
  const StaticDistanceField *df = getStaticDistanceField(link_name);
  if (!df)
    return;

  if (resolution_relative)
  {
    min_dist *= df->getResolution(distance_field::DIM_X);
    max_dist *= df->getResolution(distance_field::DIM_X);
  }

  int xend = df->getNumCells(distance_field::DIM_X);
  int yend = df->getNumCells(distance_field::DIM_Y);
  int zend = df->getNumCells(distance_field::DIM_Z);
  for (int z = 0 ; z < zend ; ++z)
  {
    for (int y = 0 ; y < yend ; ++y)
    {
      for (int x = 0 ; x < xend ; ++x)
      {
        const collision_detection::DistPosEntry& entry = df->getCell(x,y,z);
        if (entry.distance_ >= min_dist && entry.distance_ <= max_dist)
        {
          Eigen::Vector3d p;
          df->gridToWorld(x, y, z, p.x(), p.y(), p.z());
          points.push_back(p);
        }
      }
    }
  }
}


