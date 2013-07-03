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

#ifndef MESH_ROS__MESH_RVIZ
#define MESH_ROS__MESH_RVIZ

#include <OGRE/OgreMaterial.h>
#include <Eigen/Geometry>

namespace mesh_core
{
  class Mesh;
}

namespace rviz
{
  class DisplayContext;
}

namespace Ogre
{
  class SceneNode;
  class Vector3;
  class Quaternion;
  class ManualObject;
}

namespace mesh_ros
{

class RvizMeshShape
{
public:
  RvizMeshShape(rviz::DisplayContext* context,
                Ogre::SceneNode* parent_node,
                const mesh_core::Mesh* mesh = NULL,
                const Eigen::Vector4f& color = Eigen::Vector4f(1,1,1,1));

  ~RvizMeshShape();

  // reset to a new (or NULL) shape.
  // Start with first_tri. If tri_cnt>0 then use at most that many tris.
  void reset(const mesh_core::Mesh* mesh = NULL,
             int first_tri = 0,
             int tri_cnt = -1);

  // set color and alpha
  void setColor(const Eigen::Vector4f& color);

  virtual void setPosition(const Ogre::Vector3& position);
  virtual void setOrientation(const Ogre::Quaternion& orientation);
  const Ogre::Vector3& getPosition();
  const Ogre::Quaternion& getOrientation();

private:
  Ogre::ManualObject* manual_object_;
  Ogre::MaterialPtr material_;
  std::string material_name_;

  rviz::DisplayContext* context_;
  Ogre::SceneNode* scene_node_;

  // TODO: is this needed?  If not delete it.
  //boost::shared_ptr<MarkerSelectionHandler> handler_;
};

}


#endif

