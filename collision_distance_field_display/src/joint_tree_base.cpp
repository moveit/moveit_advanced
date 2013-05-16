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

#include <collision_distance_field_display/joint_tree_base.h>

#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_model/joint_model.h>
#include <moveit/robot_model/link_model.h>
#include <moveit/robot_state/robot_state.h>

#include <rviz/properties/bool_property.h>

#include <ros/console.h>

joint_tree::JointTreeBase::JointTreeBase(rviz::Property* parent)
  : Property("Joint Tree", QVariant(), "Tree of robot joints and links.", parent)
  , show_all_links_(NULL)
  , expand_all_joints_(NULL)
  , expand_all_links_(NULL)
  , show_joint_properties_(NULL)
  , show_link_properties_(NULL)
  , tree_root_(NULL)
  , loading_(false)
{
}

joint_tree::JointTreeBase::~JointTreeBase()
{
}


void joint_tree::JointTreeBase::setRobotState(const boost::shared_ptr<const robot_state::RobotState>& state)
{
  if (!state)
  {
    removeChildren();
    return;
  }

  state_ = state;

  if (model_ != state->getRobotModel())
  {
    model_ = state->getRobotModel();
    loadTree();
  }

}

void joint_tree::JointTreeBase::changedExpandAllJoints()
{
  bool expand = true;
  if (expand_all_joints_)
    expand = expand_all_joints_->getBool();

  for (std::map<std::string, Joint*>::iterator it = joint_props_.begin() ; it != joint_props_.end() ; ++it)
  {
    if (expand)
      it->second->expand();
    else
      it->second->collapse();
  }
}

void joint_tree::JointTreeBase::changedExpandAllLinks()
{
  bool expand = true;
  if (expand_all_links_)
    expand = expand_all_links_->getBool();

  for (std::map<std::string, Link*>::iterator it = link_props_.begin() ; it != link_props_.end() ; ++it)
  {
    if (expand)
      it->second->expand();
    else
      it->second->collapse();
  }
}

void joint_tree::JointTreeBase::changedShowAllLinks()
{

}

void joint_tree::JointTreeBase::changedTreeStructure()
{
  loadTree();
}

void joint_tree::JointTreeBase::addTreeProperties()
{
  show_all_links_ = new rviz::BoolProperty(
                                      "Show all links",
                                      true,
                                      "Hide/show all links.",
                                      this,
                                      SLOT( changedShowAllLinks() ));
  expand_all_joints_ = new rviz::BoolProperty(
                                      "Expand all joints",
                                      false,
                                      "Expand/collapse entire joint tree.",
                                      this,
                                      SLOT( changedExpandAllJoints() ));
  expand_all_links_ = new rviz::BoolProperty(
                                      "Expand all links",
                                      false,
                                      "Expand/contract each link property.",
                                      this,
                                      SLOT( changedExpandAllLinks() ));
  show_joint_properties_ = new rviz::BoolProperty(
                                      "Show Joint Properties",
                                      true,
                                      "Show Joints in the tree.",
                                      this,
                                      SLOT( changedTreeStructure() ));
  show_link_properties_ = new rviz::BoolProperty(
                                      "Show Link Properties",
                                      true,
                                      "Show Links in the tree.",
                                      this,
                                      SLOT( changedTreeStructure() ));
}

void joint_tree::JointTreeBase::addJointProperties(Joint *joint)
{

}

void joint_tree::JointTreeBase::addLinkProperties(Link *link)
{

}

void joint_tree::JointTreeBase::loadTree()
{
  if (loading_)
    return;
  loading_ = true;

  if (!tree_root_)
  {
    removeChildren();
    show_all_links_ = NULL;
    expand_all_joints_ = NULL;
    expand_all_links_ = NULL;
    show_joint_properties_ = NULL;
    show_link_properties_ = NULL;
    tree_root_ = NULL;

    addTreeProperties();
    tree_root_ = new rviz::Property("Tree Root", QVariant(), "", this);
  }

  tree_root_->removeChildren();
  joint_props_.clear();
  link_props_.clear();
  addJoint(tree_root_, model_->getRoot());

  loading_ = false;
}


joint_tree::JointTreeBase::Joint *joint_tree::JointTreeBase::newJoint(
                            rviz::Property *parent,
                            const robot_model::JointModel *joint_model)
{
  return new joint_tree::JointTreeBase::Joint(parent, joint_model, this);
}

joint_tree::JointTreeBase::Link *joint_tree::JointTreeBase::newLink(
                          rviz::Property *parent,
                          const robot_model::LinkModel *link_model)
{
  return new joint_tree::JointTreeBase::Link(parent, link_model, this);
}


joint_tree::JointTreeBase::Joint *joint_tree::JointTreeBase::addJoint(
                            rviz::Property *parent,
                            const robot_model::JointModel *joint_model)
{
  if (show_joint_properties_->getBool())
  {
    Joint* joint = newJoint(parent, joint_model);
    joint_props_[joint_model->getName()] = joint;
    addJointProperties(joint);
    parent = joint;
  }
  addLink(parent, joint_model->getChildLinkModel());
}

joint_tree::JointTreeBase::Link *joint_tree::JointTreeBase::addLink(
                          rviz::Property *parent,
                          const robot_model::LinkModel *link_model)
{
  if (show_link_properties_)
  {
    Link* link = newLink(parent, link_model);
    link_props_[link_model->getName()] = link;
    addLinkProperties(link);
    parent = link;
  }
  for (std::vector<robot_model::JointModel*>::const_iterator it = link_model->getChildJointModels().begin() ;
       it != link_model->getChildJointModels().end() ;
       ++it)
  {
    addJoint(parent, *it);
  }
}


//===========================================================================
//  Joint
//===========================================================================

joint_tree::JointTreeBase::Joint::Joint(rviz::Property *parent,
                                        const robot_model::JointModel *joint_model,
                                        JointTreeBase *tree)
  : BoolProperty("", true, "", parent)
  , tree_(tree)
  , joint_model_(joint_model)
{
  setName(joint_model_->getName().c_str());

  std::stringstream desc;
  desc << "Joint with ";
  if (joint_model->getParentLinkModel())
  {
    desc << "parent_link=" << joint_model->getParentLinkModel()->getName() << ", ";
    if (joint_model->getParentLinkModel()->getParentJointModel())
    {
      desc << "parent_joint=" << joint_model->getParentLinkModel()->getParentJointModel()->getName() << ", ";
    }
  }
  else
  {
    desc << "no parent, ";
  }
  desc << "and child_link=" << joint_model->getChildLinkModel()->getName() << ".";
  setDescription(desc.str().c_str());
}

joint_tree::JointTreeBase::Joint::~Joint()
{}

void joint_tree::JointTreeBase::Joint::changeVisibility()
{
}

//===========================================================================
//  Link
//===========================================================================

joint_tree::JointTreeBase::Link::Link(rviz::Property *parent,
                                      const robot_model::LinkModel *link_model,
                                      JointTreeBase *tree)
  : BoolProperty("", true, "", parent)
  , tree_(tree)
  , link_model_(link_model)
{
  setName(link_model_->getName().c_str());

  std::stringstream desc;
  desc << "Link with ";
  if (link_model_->getParentJointModel())
  {
    desc << "parent_joint=" << link_model_->getParentJointModel()->getName() << ", ";
    if (link_model_->getParentJointModel()->getParentLinkModel())
    {
      desc << "parent_link=" << link_model_->getParentJointModel()->getParentLinkModel()->getName() << ", ";
    }
  }
  else
  {
    desc << "no parent, ";
  }
  int nchild = link_model_->getChildJointModels().size();
  if (nchild==0)
    desc << "and no children";
  else if (nchild==1)
    desc << "and one child joint: ";
  else
    desc << "and " << nchild << " child joints: ";
  for (int i=0; i<nchild; i++)
  {
    if (i)
      desc << ", ";
    desc << link_model_->getChildJointModels()[i]->getName();
  }
  desc << ".";
  setDescription(desc.str().c_str());
}

joint_tree::JointTreeBase::Link::~Link()
{}

void joint_tree::JointTreeBase::Link::changeVisibility()
{

}
