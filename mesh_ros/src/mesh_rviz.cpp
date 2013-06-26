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

#include <mesh_ros/mesh_rviz.h>
#include <mesh_core/mesh.h>
#include <console_bridge/console.h>

//#include <rviz/default_plugin/markers/marker_selection_handler.h>
//#include <rviz/default_plugin/marker_display.h>
//#include <rviz/selection/selection_manager.h>

#include <rviz/display_context.h>
//#include <rviz/mesh_loader.h>
//#include <rviz/default_plugin/marker_display.h>

#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreManualObject.h>
#include <OGRE/OgreMaterialManager.h>
#include <OGRE/OgreTextureManager.h>
#include <OGRE/OgreTechnique.h>

#include <sstream>


mesh_ros::RvizMeshShape::RvizMeshShape(
      rviz::DisplayContext* context,
      Ogre::SceneNode* parent_node,
      const mesh_core::Mesh* mesh,
      const Eigen::Vector4f& color)
  : context_(context)
  , scene_node_(parent_node->createChildSceneNode())
{
  static uint32_t count = 0;
  std::stringstream ss;
  ss << "RvizMeshShape" << count++;
  manual_object_ = context_->getSceneManager()->createManualObject(ss.str());
  scene_node_->attachObject(manual_object_);

  ss << "Material";
  material_name_ = ss.str();
  material_ = Ogre::MaterialManager::getSingleton().create( material_name_, "rviz" );
  material_->setReceiveShadows(false);
  material_->getTechnique(0)->setLightingEnabled(true);
  material_->setCullingMode(Ogre::CULL_NONE);

  //handler_.reset( new MarkerSelectionHandler( this, MarkerID( new_message->ns, new_message->id ), context_ ));

  setColor(color);

  reset(mesh);
}

void mesh_ros::RvizMeshShape::setColor(
      const Eigen::Vector4f& color)
{
  material_->getTechnique(0)->setLightingEnabled(true);
  float r = color.x();
  float g = color.y();
  float b = color.z();
  float a = color.w();
  material_->getTechnique(0)->setAmbient( r*0.3, g*0.3, b*0.3 );
  material_->getTechnique(0)->setDiffuse( r, g, b, a );

  if (a < 0.9998)
  {
    material_->getTechnique(0)->setSceneBlending( Ogre::SBT_TRANSPARENT_ALPHA );
    material_->getTechnique(0)->setDepthWriteEnabled( false );
  }
  else
  {
    material_->getTechnique(0)->setSceneBlending( Ogre::SBT_REPLACE );
    material_->getTechnique(0)->setDepthWriteEnabled( true );
  }
}


mesh_ros::RvizMeshShape::~RvizMeshShape()
{
  context_->getSceneManager()->destroyManualObject(manual_object_);
  material_->unload();
  Ogre::MaterialManager::getSingleton().remove(material_->getName());
  context_->getSceneManager()->destroySceneNode(scene_node_);
}

void mesh_ros::RvizMeshShape::reset(
      const mesh_core::Mesh* mesh)
{
  manual_object_->clear();

  if (!mesh)
    return;

  manual_object_->estimateVertexCount(mesh->getTriCount() * 3);
  manual_object_->begin(material_name_, Ogre::RenderOperation::OT_TRIANGLE_LIST);

  std::vector<mesh_core::Mesh::Triangle>::const_iterator it = mesh->getTris().begin();
  std::vector<mesh_core::Mesh::Triangle>::const_iterator end = mesh->getTris().end();
  for ( ; it != end ; ++it)
  {
    const Eigen::Vector3d& v0 = mesh->getVert(it->verts_[0]);
    const Eigen::Vector3d& v1 = mesh->getVert(it->verts_[1]);
    const Eigen::Vector3d& v2 = mesh->getVert(it->verts_[2]);
    Eigen::Vector3d norm = ((v1 - v0).cross(v2 - v0)).normalized();

    for (int i = 0 ; i < 3 ; ++i)
    {
      const Eigen::Vector3d& v = mesh->getVert(it->verts_[i]);
      manual_object_->position(v.x(), v.y(), v.z());
      manual_object_->normal(norm.x(), norm.y(), norm.z());
    }
  }

  manual_object_->end();

  //handler_->addTrackedObject( manual_object_ );
}

void mesh_ros::RvizMeshShape::setPosition( const Ogre::Vector3& position )
{
  scene_node_->setPosition( position );
}

void mesh_ros::RvizMeshShape::setOrientation( const Ogre::Quaternion& orientation )
{
  scene_node_->setOrientation( orientation );
}

const Ogre::Vector3& mesh_ros::RvizMeshShape::getPosition()
{
  return scene_node_->getPosition();
}

const Ogre::Quaternion& mesh_ros::RvizMeshShape::getOrientation()
{
  return scene_node_->getOrientation();
}


