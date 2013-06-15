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

collision_detection::CollisionRobotDistanceField::WorkArea& collision_detection::CollisionRobotDistanceField::getWorkArea() const
{
  if (!work_area_.get())
  {
    work_area_.reset(new WorkArea);
    WorkArea& work = *work_area_;
    work.transformed_sphere_centers_.resize(sphere_centers_.size());
    work.df_contacts_ = NULL;
  }

  WorkArea& work = *work_area_;
  return work;
}

collision_detection::CollisionRobotDistanceField::WorkArea::~WorkArea()
{
}

// show the state of the current query
void collision_detection::CollisionRobotDistanceField::dumpQuery(const WorkArea& work, const char *descrip) const
{
  logInform("CollisionRobotDistanceField Query %s", descrip);

  std::stringstream ss_cost;
  ss_cost << ", cost(max=" << work.req_->max_cost_sources << ", dens=" << work.req_->min_cost_density << ")";
  std::stringstream ss_contacts;
  ss_contacts << ", contact(max=" << work.req_->max_contacts << ", cpp=" << work.req_->max_contacts_per_pair << ")";
  std::stringstream ss_acm;
  if (work.acm_)
  {
    std::vector<std::string> names;
    work.acm_->getAllEntryNames(names);
    ss_acm << ", acm(nnames=" << names.size() << ", sz=" << work.acm_->getSize() << ")";
  }
  logInform("   request: result%s%s%s%s%s%s",
    work.req_->distance ? ", distance" : "",
    work.req_->cost ? ss_cost.str().c_str() : "",
    work.req_->contacts ? ss_contacts.str().c_str() : "",
    work.req_->is_done ? ", is_done-func" : "",
    work.req_->verbose ? ", VERBOSE" : "",
    ss_acm.str().c_str());
}

void collision_detection::DFContact::copyFrom(const Contact& contact)
{
  pos = contact.pos;
	normal = contact.normal;
	depth = contact.depth;
	body_name_1 = contact.body_name_1;
	body_type_1 = contact.body_type_1;
	body_name_2 = contact.body_name_2;
	body_type_2 = contact.body_type_2;

  sphere_radius_1 = 0;
  sphere_radius_2 = 0;
  sdf_1 = NULL;
  sdf_2 = NULL;
  eliminated_by_acm_function = false;
}
