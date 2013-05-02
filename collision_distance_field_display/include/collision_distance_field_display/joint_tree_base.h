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

#ifndef COLLISION_DISTANCE_FIELD_DISPLAY_JOINT_TREE_BASE_H
#define COLLISION_DISTANCE_FIELD_DISPLAY_JOINT_TREE_BASE_H

#include <rviz/properties/property.h>
#include <rviz/properties/bool_property.h>
#include <boost/shared_ptr.hpp>


#ifndef Q_MOC_RUN
#endif

namespace rviz
{
  class StringProperty;
  class BoolProperty;
}
namespace robot_model
{
  class RobotModel;
  class JointModel;
  class LinkModel;
}
namespace robot_state
{
  class RobotState;
}

namespace joint_tree
{


/// display tree of joint properties
class JointTreeBase : public rviz::Property
{
Q_OBJECT
public:
  JointTreeBase(rviz::Property* parent);
  virtual ~JointTreeBase();

  class Joint;
  class Link;

  virtual void setRobotState(const boost::shared_ptr<const robot_state::RobotState>& state);
  const boost::shared_ptr<const robot_state::RobotState>& getRobotState()
  {
    return state_;
  }
  const boost::shared_ptr<const robot_model::RobotModel>& getRobotModel()
  {
    return model_;
  }

private Q_SLOTS:
  void changedExpandAllJoints();
  void changedExpandAllLinks();
  void changedShowAllLinks();
  void changedTreeStructure();

protected:
  // creates the link tree.  Called automatically by setRobotState() when RobotModel changes.
  // Subclasses can override to do post-load operations.
  virtual void loadTree();

  // create new joint/link.  Override to subclass Joint or Link.  Called by addJoint/addLink.
  virtual Joint *newJoint(rviz::Property *parent,
                          const robot_model::JointModel *joint_model);
  virtual Link *newLink(rviz::Property *parent,
                        const robot_model::LinkModel *link_model);

  // creates the joints and links of the tree.  Called (recursively) by loadTree().
  virtual Joint *addJoint(rviz::Property *parent,
                          const robot_model::JointModel *joint_model);
  virtual Link *addLink(rviz::Property *parent,
                        const robot_model::LinkModel *link_model);

  // Hooks to add properties to the tree
  virtual void addTreeProperties();
  virtual void addJointProperties(Joint *joint);
  virtual void addLinkProperties(Link *link);

  Joint* getJoint(const std::string& name)
  {
    std::map<std::string, Joint*>::iterator it = joint_props_.find(name);
    return it == joint_props_.end() ? NULL : it->second;
  }

  Link* getLink(const std::string& name)
  {
    std::map<std::string, Link*>::iterator it = link_props_.find(name);
    return it == link_props_.end() ? NULL : it->second;
  }

  // properties controlling the joint tree
  // subclasses can modify defaults or hide these by overriding addTreeProperties()
  rviz::BoolProperty* show_all_links_;
  rviz::BoolProperty* expand_all_joints_;
  rviz::BoolProperty* expand_all_links_;
  rviz::BoolProperty* show_joint_properties_;
  rviz::BoolProperty* show_link_properties_;

private:
  boost::shared_ptr<const robot_model::RobotModel> model_;
  boost::shared_ptr<const robot_state::RobotState> state_;

  // link and joint properties
  rviz::Property* tree_root_;
  std::map<std::string, Joint*> joint_props_;
  std::map<std::string, Link*> link_props_;
  bool loading_;
};

//===========================================================================
//  Joint
//===========================================================================

/// properties of a joint in the joint tree
class JointTreeBase::Joint : public rviz::BoolProperty
{
Q_OBJECT
public:
  Joint(rviz::Property *parent,
        const robot_model::JointModel *joint_model,
        JointTreeBase *tree);
  virtual ~Joint();

  const robot_model::JointModel* getJointModel() const
  {
    return joint_model_;
  }

  JointTreeBase* getTree()
  {
    return tree_;
  }

private Q_SLOTS:
  void changeVisibility();

private:
  JointTreeBase *tree_;
  const robot_model::JointModel *joint_model_;
};

//===========================================================================
//  Link
//===========================================================================

/// properties of a link in the joint tree
class JointTreeBase::Link : public rviz::BoolProperty
{
Q_OBJECT
public:
  Link(rviz::Property *parent,
       const robot_model::LinkModel *link_model,
       JointTreeBase *tree);
  virtual ~Link();

#if 0
  void changeVisibilityIncludingChildren(bool visible);

  void setTransforms(const Ogre::Vector3& visual_position,
                     const Ogre::Quaternion& visual_orientation,
                     const Ogre::Vector3& collision_position, 
                     const Ogre::Quaternion& collision_orientation);
#endif

  const robot_model::LinkModel* getLinkModel() const
  {
    return link_model_;
  }

  JointTreeBase* getTree()
  {
    return tree_;
  }

private Q_SLOTS:
  void changeVisibility();

private:
  JointTreeBase *tree_;
  const robot_model::LinkModel *link_model_;

#if 0
  Property *robot_link_prop_;

  std::vector<Joint*> joints_;

  Ogre::SceneNode* visual_node_;
  Ogre::SceneNode* collision_node_;
#endif
};

}
#endif
