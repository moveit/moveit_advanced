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
      const Eigen::Vector4f& color,
      int first_tri,
      int tri_cnt)
  : context_(context)
  , scene_node_(parent_node->createChildSceneNode())
{
  initialize(color);
  reset(mesh, first_tri, tri_cnt);
}

mesh_ros::RvizMeshShape::RvizMeshShape(
      rviz::DisplayContext* context,
      Ogre::SceneNode* parent_node,
      const mesh_core::Mesh* mesh,
      const Eigen::Vector4f& color,
      const std::vector<int>& tris,
      int first_tri,
      int tri_cnt)
  : context_(context)
  , scene_node_(parent_node->createChildSceneNode())
{
  initialize(color);
  reset(mesh, tris, first_tri, tri_cnt);
}

void mesh_ros::RvizMeshShape::initialize(
      const Eigen::Vector4f& color)
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
}

mesh_ros::RvizMeshShape::~RvizMeshShape()
{
  context_->getSceneManager()->destroyManualObject(manual_object_);
  material_->unload();
  Ogre::MaterialManager::getSingleton().remove(material_->getName());
  context_->getSceneManager()->destroySceneNode(scene_node_);
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


void mesh_ros::RvizMeshShape::addTri(
      const mesh_core::Mesh* mesh,
      const mesh_core::Mesh::Triangle& tri)
{
  const Eigen::Vector3d& v0 = mesh->getVert(tri.verts_[0]);
  const Eigen::Vector3d& v1 = mesh->getVert(tri.verts_[1]);
  const Eigen::Vector3d& v2 = mesh->getVert(tri.verts_[2]);
  Eigen::Vector3d norm = ((v1 - v0).cross(v2 - v0)).normalized();

  for (int j = 0 ; j < 3 ; ++j)
  {
    const Eigen::Vector3d& v = mesh->getVert(tri.verts_[j]);
    manual_object_->position(v.x(), v.y(), v.z());
    manual_object_->normal(norm.x(), norm.y(), norm.z());
  }
}

void mesh_ros::RvizMeshShape::reset(
      const mesh_core::Mesh* mesh,
      int first_tri,
      int tri_cnt)
{
  manual_object_->clear();

  if (!mesh)
    return;

  if (first_tri >= mesh->getTriCount() || tri_cnt==0)
    return;

  if (first_tri < 0)
    first_tri = 0;

  int end_tri = tri_cnt < 0 ? mesh->getTriCount() : first_tri + tri_cnt;
  if (end_tri > mesh->getTriCount())
    end_tri = mesh->getTriCount();

  manual_object_->estimateVertexCount((end_tri - first_tri) * 3);
  manual_object_->begin(material_name_, Ogre::RenderOperation::OT_TRIANGLE_LIST);

  for (int i = first_tri ; i < end_tri ; ++i)
  {
    addTri(mesh, mesh->getTri(i));
  }

  manual_object_->end();

  //handler_->addTrackedObject( manual_object_ );
}

void mesh_ros::RvizMeshShape::reset(
      const mesh_core::Mesh* mesh,
      const std::vector<int>& tris,
      int first_tri,
      int tri_cnt)
{
  manual_object_->clear();

  if (!mesh)
    return;

  if (first_tri >= tris.size() || tri_cnt==0)
    return;

  if (first_tri < 0)
    first_tri = 0;

  int end_tri = tri_cnt < 0 ? tris.size() : first_tri + tri_cnt;
  if (end_tri > tris.size())
    end_tri = tris.size();

  manual_object_->estimateVertexCount((end_tri - first_tri) * 3);
  manual_object_->begin(material_name_, Ogre::RenderOperation::OT_TRIANGLE_LIST);

  int mesh_tri_cnt = mesh->getTriCount();
  for (int i = first_tri ; i < end_tri ; ++i)
  {
    if (tris[i] >= mesh_tri_cnt)
    {
      logError("mesh_ros::RvizMeshShape::reset() tris contains out of bound triangle index");
      continue;
    }
    addTri(mesh, mesh->getTri(tris[i]));
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


