/*
 * Copyright (c) 2013, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/* Author: Acorn Pooley */

#include <collision_distance_field_display/df_link.h>
#include <rviz/properties/bool_property.h>

moveit_rviz_plugin::DFLink::DFLink(
    rviz::Robot* robot,
    const urdf::LinkConstPtr& link,
    const std::string& parent_joint_name,
    bool visual,
    bool collision)
  : RobotLink(robot, link, parent_joint_name, visual, collision)
  , sample_prop_(NULL)
{
  sample_prop_ = new rviz::BoolProperty(
                            "Dummy example property",
                            false,
                            "This does nothing.",
                            link_property_,
                            SLOT( updateSampleProp() ),
                            this );
}

moveit_rviz_plugin::DFLink::~DFLink()
{
}

void moveit_rviz_plugin::DFLink::updateSampleProp()
{
  ROS_INFO("updateSampleProp called for %s", getName().c_str());
}

void moveit_rviz_plugin::DFLink::hideSubProperties(bool hide)
{
  RobotLink::hideSubProperties(hide);
  sample_prop_->setHidden(hide);
}

rviz::RobotLink* moveit_rviz_plugin::DFLinkFactory::createLink(
    rviz::Robot* robot,
    const boost::shared_ptr<const urdf::Link>& link,
    const std::string& parent_joint_name,
    bool visual,
    bool collision)
{
  return new DFLink(robot, link, parent_joint_name, visual, collision);
}
