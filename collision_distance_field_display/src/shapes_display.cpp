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

#include <collision_distance_field_display/shapes_display.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreQuaternion.h>
#include <rviz/ogre_helpers/shape.h>
#include <boost/math/constants/constants.hpp>
#include <console_bridge/console.h>
#include <rviz/ogre_helpers/point_cloud.h>


moveit_rviz_plugin::ShapesDisplay::ShapesDisplay(
    Ogre::SceneNode* parent,
    const Eigen::Vector4f& color,
    double point_size)
  : color_(color)
  , node_(parent->createChildSceneNode())
  , point_size_(point_size)
  , points_(NULL)
{ }

moveit_rviz_plugin::ShapesDisplay::~ShapesDisplay()
{
  clear();
  node_->getParentSceneNode()->removeAndDestroyChild(node_->getName());
  delete points_;
}

inline void moveit_rviz_plugin::ShapesDisplay::addPointCloud()
{
  if (!points_)
  {
    points_ = new rviz::PointCloud();
    node_->attachObject(points_);
    points_->setRenderMode(rviz::PointCloud::RM_SQUARES);
    points_->setDimensions(point_size_, point_size_, 0.0f);
    points_->setAlpha(color_.w(), false);
  }
}

void moveit_rviz_plugin::ShapesDisplay::clear()
{
  std::vector<Shape>::iterator it = shapes_.begin();
  std::vector<Shape>::iterator end = shapes_.end();
  for ( ; it != end ; ++it)
  {
    delete it->shape_;
  }
  shapes_.clear();
  if (points_)
    points_->clear();
}

static inline Ogre::Vector3 OVec3(const Eigen::Vector3d& v)
{
  return Ogre::Vector3(v.x(), v.y(), v.z());
}

static inline Ogre::Quaternion OQuat(const Eigen::Quaterniond& q)
{
  return Ogre::Quaternion(q.w(), q.x(), q.y(), q.z());
}

void moveit_rviz_plugin::ShapesDisplay::addPoint(
      const Eigen::Vector3d& point,
      const color_cast::Color& color)
{
  const Eigen::Vector4f& colorf = color.isDefault() ? color_ : color.getColorf();
  addPointCloud();
  rviz::PointCloud::Point p;
  p.position.x = point.x();
  p.position.y = point.y();
  p.position.z = point.z();
  p.color.r = colorf.x();
  p.color.g = colorf.y();
  p.color.b = colorf.z();
  p.color.a = colorf.w();
  points_->addPoints(&p, 1);
  if (colorf.w() != color_.w())
    points_->setAlpha(1.0, true);
}

void moveit_rviz_plugin::ShapesDisplay::addPoints(
      const EigenSTL::vector_Vector3d& points,
      const color_cast::Color& color)
{
  EigenSTL::vector_Vector3d::const_iterator it = points.begin();
  EigenSTL::vector_Vector3d::const_iterator end = points.end();
  for ( ; it != end ; ++it)
  {
    addPoint(*it, color);
  }
}

void moveit_rviz_plugin::ShapesDisplay::addSphere(
    const Eigen::Vector3d& center,
    double radius,
    const color_cast::Color& color)
{
  const Eigen::Vector4f& colorf = color.isDefault() ? color_ : color.getColorf();
  Shape s;
  s.node_ = node_->createChildSceneNode();
  s.shape_ = new rviz::Shape(rviz::Shape::Sphere,
                             node_->getCreator(),
                             s.node_);

  Ogre::Vector3 pos(center.x(), center.y(), center.z());
  Ogre::Vector3 scale(radius*2, radius*2, radius*2);
  s.node_->setPosition(pos);
  s.shape_->setScale(scale);
  s.shape_->setColor(colorf.x(), colorf.y(), colorf.z(), colorf.w());

  shapes_.push_back(s);
}

void moveit_rviz_plugin::ShapesDisplay::addSpheres(
      const EigenSTL::vector_Vector3d& centers,
      std::vector<double> radii,
      const color_cast::Color& color)
{
  std::size_t cnt = centers.size();
  if (!cnt)
    return;
  if (radii.size() != cnt && radii.size() != 1)
  {
    logError("radius.size() != centers.size()");
    return;
  }
  bool unique_radius = radii.size() == cnt;

  for (std::size_t i = 0 ; i < cnt ; ++i)
  {
    if (unique_radius)
      addSphere(centers[i], radii[i], color);
    else
      addSphere(centers[i], radii[0], color);
  }
}

void moveit_rviz_plugin::ShapesDisplay::addSpheres(
      const EigenSTL::vector_Vector3d& centers,
      double radius,
      const color_cast::Color& color)
{
  std::size_t cnt = centers.size();
  for (std::size_t i = 0 ; i < cnt ; ++i)
    addSphere(centers[i], radius, color);
}

void moveit_rviz_plugin::ShapesDisplay::addBox(
    const Eigen::Affine3d& pose, 
    const Eigen::Vector3d& size,
    const color_cast::Color& color)
{
  const Eigen::Vector4f& colorf = color.isDefault() ? color_ : color.getColorf();
  Shape s;
  s.node_ = node_->createChildSceneNode();
  s.shape_ = new rviz::Shape(rviz::Shape::Cube,
                             node_->getCreator(),
                             s.node_);
  s.shape_->setScale(OVec3(size));
  s.shape_->setColor(colorf.x(), colorf.y(), colorf.z(), colorf.w());

  Ogre::Vector3 position(pose.translation().x(),
                         pose.translation().y(),
                         pose.translation().z());
  Eigen::Quaterniond q(pose.rotation());
  Ogre::Quaternion orientation(q.w(), q.x(), q.y(), q.z());

  s.node_->setPosition(position);
  s.node_->setOrientation(orientation);

  shapes_.push_back(s);
}

void moveit_rviz_plugin::ShapesDisplay::addBox(
    const Eigen::Affine3d& pose, 
    double size,
    const color_cast::Color& color)
{
  addBox(pose, Eigen::Vector3d(size, size, size), color);
}

void moveit_rviz_plugin::ShapesDisplay::addBox(
    const Eigen::Vector3d& position,
    double size,
    const color_cast::Color& color)
{
  addBox(Eigen::Affine3d(Eigen::Translation3d(position)), Eigen::Vector3d(size, size, size), color);
}

void moveit_rviz_plugin::ShapesDisplay::addZCylinder(
    const Eigen::Affine3d& pose, 
    double radius, 
    double length, 
    const color_cast::Color& color)
{
  const Eigen::Vector4f& colorf = color.isDefault() ? color_ : color.getColorf();
  Shape s;
  s.node_ = node_->createChildSceneNode();
  s.shape_ = new rviz::Shape(rviz::Shape::Cylinder,
                             node_->getCreator(),
                             s.node_);

  // unscaled cylinder goes from y=-0.5 to y=0.5
  // unscaled cylinder diameter is 1
  double diam = 2.0 * radius;
  s.shape_->setScale(Ogre::Vector3(diam, length, diam));

  Ogre::Vector3 position(pose.translation().x(),
                         pose.translation().y(),
                         pose.translation().z());
  Eigen::Quaterniond q(pose.rotation());
  Ogre::Quaternion orientation(q.w(), q.x(), q.y(), q.z());

  // rotate the cylinder from z-axis to y-axis
  Ogre::Quaternion fix(Ogre::Radian(boost::math::constants::pi<double>()/2.0),
                       Ogre::Vector3(1.0, 0.0, 0.0));
  orientation = orientation * fix;

  s.node_->setPosition(position);
  s.node_->setOrientation(orientation);

  s.shape_->setColor(colorf.x(), colorf.y(), colorf.z(), colorf.w());

  shapes_.push_back(s);
}

static void calcOrientationFromEndpoints(
    const Eigen::Vector3d& base,    // in
    const Eigen::Vector3d& tip,     // in
    double& len,                    // out
    Eigen::Vector3d& dir,           // out
    Ogre::Quaternion& quat)         // out
{
  dir = tip - base;
  len = dir.norm();
  if (len < std::numeric_limits<double>::epsilon())
  {
    len = 0.0;
    dir = Eigen::Vector3d::Zero();
    quat = Ogre::Quaternion(1.0,0.0,0.0,0.0);
    return;
  }

  dir *= 1.0/len;
  Eigen::Vector3d tmp(dir.y(), dir.z(), dir.x());
  Eigen::Vector3d tan0 = dir.cross(tmp);
  tan0.normalize();
  Eigen::Vector3d tan1 = tan0.cross(dir);
  
  quat = Ogre::Quaternion(OVec3(tan0), OVec3(dir), OVec3(tan1));
}
 

void moveit_rviz_plugin::ShapesDisplay::addCylinderOrCone(
    bool cone,
    const Eigen::Vector3d& base,
    const Eigen::Vector3d& dir,
    double length,
    double radius,
    const Ogre::Quaternion& quat,
    const color_cast::Color& color)
{
  if (length < std::numeric_limits<double>::epsilon())
    return;

  const Eigen::Vector4f& colorf = color.isDefault() ? color_ : color.getColorf();

  Shape s;
  s.node_ = node_->createChildSceneNode();
  s.shape_ = new rviz::Shape(cone ? rviz::Shape::Cone : rviz::Shape::Cylinder,
                             node_->getCreator(),
                             s.node_);
  s.shape_->setColor(colorf.x(), colorf.y(), colorf.z(), colorf.w());

  // unscaled cylinder length is from y=-0.5 to y=+0.5
  // unscaled cylinder diameter is 1
  //
  // unscaled cone base is at y=-0.5 and tip is at y=+0.5
  // unscaled cone diameter is 1
  s.shape_->setScale(Ogre::Vector3(radius * 2.0, length, radius * 2.0));

  Eigen::Vector3d pos = base + 0.5 * length * dir;

  s.node_->setPosition(OVec3(pos));
  s.node_->setOrientation(quat);
  shapes_.push_back(s);
}

void moveit_rviz_plugin::ShapesDisplay::addCylinder(
    const Eigen::Vector3d& end0,
    const Eigen::Vector3d& end1,
    double radius,
    const color_cast::Color& color)
{
  double len;
  Eigen::Vector3d dir;
  Ogre::Quaternion quat;
  calcOrientationFromEndpoints(end0, end1, len, dir, quat);

  addCylinderOrCone(false, end0, dir, len, radius, quat, color);
}

void moveit_rviz_plugin::ShapesDisplay::addCone(
    const Eigen::Vector3d& base,
    const Eigen::Vector3d& tip,
    double radius,
    const color_cast::Color& color)
{
  double len;
  Eigen::Vector3d dir;
  Ogre::Quaternion quat;
  calcOrientationFromEndpoints(base, tip, len, dir, quat);

  addCylinderOrCone(true, base, dir, len, radius, quat, color);
}

// Assumes length extends from origin towards the z axis
void moveit_rviz_plugin::ShapesDisplay::addArrow(
      const Eigen::Vector3d& base,
      const Eigen::Vector3d& tip,
      const color_cast::Color& color,
      double max_cone_diameter,
      double min_cylinder_diameter)
{
  double len;
  Eigen::Vector3d dir;
  Ogre::Quaternion quat;
  calcOrientationFromEndpoints(base, tip, len, dir, quat);

  // determine the size
  double cyl_diameter = max_cone_diameter * 0.5;
  double cone_diameter = max_cone_diameter;
  double cone_len = max_cone_diameter * 2.0;
  double cyl_len = len - cone_len;

  if (len < 3 * cone_len)
  {
    cone_len = len / 3.0;
    cyl_len = len - cone_len;
    cone_diameter = cone_len * 0.5;
    cyl_diameter = cone_diameter * 0.5;
  }
  if (cyl_diameter < min_cylinder_diameter)
  {
    cyl_diameter = min_cylinder_diameter;
    if (cone_diameter < cyl_diameter * 1.1)
      cone_diameter = cyl_diameter * 1.1;
    if (cone_diameter > max_cone_diameter)
      cone_diameter = max_cone_diameter;
  }

  addCylinderOrCone(
        false, 
        base, 
        dir, 
        cyl_len, 
        cyl_diameter * 0.5, 
        quat, 
        color);

  addCylinderOrCone(
        true,
        base + cyl_len * dir, 
        dir, 
        cone_len, 
        cone_diameter * 0.5, 
        quat, 
        color);
}

void moveit_rviz_plugin::ShapesDisplay::addAxis(
      const Eigen::Affine3d& frame,
      double size)
{
  addArrow(frame.translation(),
           frame.translation() + frame.linear().col(0) * size,
           Eigen::Vector4f(1,0,0,color_.w()));
  addArrow(frame.translation(),
           frame.translation() + frame.linear().col(1) * size,
           Eigen::Vector4f(0,1,0,color_.w()));
  addArrow(frame.translation(),
           frame.translation() + frame.linear().col(2) * size,
           Eigen::Vector4f(0,0,1,color_.w()));
}

void moveit_rviz_plugin::ShapesDisplay::addAxis(
      const Eigen::Quaterniond& orientation,
      const Eigen::Vector3d& position,
      double size)
{
  Eigen::Affine3d frame = Eigen::Translation3d(position) * orientation;
  addAxis(frame, size);
}

void moveit_rviz_plugin::ShapesDisplay::setDefaultColor(
      const color_cast::Color& color)
{
  color_ = color.getColorf();
}
