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
 *   * Neither the name of Willow Garage nor the names of its
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

#ifndef COLLISION_DISTANCE_FIELD_DISPLAY__SHAPES_DISPLAY_H
#define COLLISION_DISTANCE_FIELD_DISPLAY__SHAPES_DISPLAY_H

#include <Eigen/Geometry>
#include <vector>
#include <eigen_stl_containers/eigen_stl_vector_container.h>
#include <collision_distance_field_display/color_cast.h>


namespace Ogre
{
class SceneNode;
class Quaternion;
}

namespace rviz
{
class Shape;
class PointCloud;
}

namespace moveit_rviz_plugin
{

/// Represents some cylinders to draw in rviz.
class ShapesDisplay
{
public:
  ShapesDisplay(Ogre::SceneNode* parent,
             const Eigen::Vector4f& color = Eigen::Vector4f(1,1,1,1),
             double point_size = 0.005);
  ~ShapesDisplay();

  // remove all shapes
  void clear();

  // POINTS

  // add a point
  // color can be an Eigen::Vector4f, QColor, Ogre::ColourValue, std_msgs::ColorRGBA, etc
  void addPoint(const Eigen::Vector3d& point,
                const color_cast::Color& color = color_cast::Color::getDefault());

  // add several points
  // color can be an Eigen::Vector4f, QColor, Ogre::ColourValue, std_msgs::ColorRGBA, etc
  void addPoints(const EigenSTL::vector_Vector3d& points,
                 const color_cast::Color& color = color_cast::Color::getDefault());

  // SPHERES

  // add a sphere
  // color can be an Eigen::Vector4f, QColor, Ogre::ColourValue, std_msgs::ColorRGBA, etc
  void addSphere(const Eigen::Vector3d& center,
                 double radius,
                 const color_cast::Color& color = color_cast::Color::getDefault());

  // add several spheres
  // color can be an Eigen::Vector4f, QColor, Ogre::ColourValue, std_msgs::ColorRGBA, etc
  void addSpheres(const EigenSTL::vector_Vector3d& centers,
                  std::vector<double> radii,
                  const color_cast::Color& color = color_cast::Color::getDefault());

  // add several spheres, all sharing the same radius
  // color can be an Eigen::Vector4f, QColor, Ogre::ColourValue, std_msgs::ColorRGBA, etc
  void addSpheres(const EigenSTL::vector_Vector3d& centers,
                  double radius,
                  const color_cast::Color& color = color_cast::Color::getDefault());

  // BOXES/CUBES

  // add a box.
  // color can be an Eigen::Vector4f, QColor, Ogre::ColourValue, std_msgs::ColorRGBA, etc
  void addBox(
    const Eigen::Affine3d& pose,
    const Eigen::Vector3d& size,
    const color_cast::Color& color = color_cast::Color::getDefault());

  // add a cube.
  void addBox(
    const Eigen::Affine3d& pose,
    double size,
    const color_cast::Color& color = color_cast::Color::getDefault());

  // add a cube unrotated.
  void addBox(
    const Eigen::Vector3d& position,
    double size,
    const color_cast::Color& color = color_cast::Color::getDefault());

  // CYLINDERS

  // add a cylinder.  Endpoints are on z axis.  Center is at origin.
  // color can be an Eigen::Vector4f, QColor, Ogre::ColourValue, std_msgs::ColorRGBA, etc
  void addZCylinder(
    const Eigen::Affine3d& pose,
    double radius,
    double length,
    const color_cast::Color& color = color_cast::Color::getDefault());

  // add a cylinder given 2 endpoints.
  // color can be an Eigen::Vector4f, QColor, Ogre::ColourValue, std_msgs::ColorRGBA, etc
  void addCylinder(
    const Eigen::Vector3d& end0,
    const Eigen::Vector3d& end1,
    double radius,
    const color_cast::Color& color = color_cast::Color::getDefault());

  // CONES

  // add a cone
  // color can be an Eigen::Vector4f, QColor, Ogre::ColourValue, std_msgs::ColorRGBA, etc
  void addCone(
    const Eigen::Vector3d& base,
    const Eigen::Vector3d& tip,
    double radius,
    const color_cast::Color& color = color_cast::Color::getDefault());

  // ARROWS

  // add an arrow
  // diameter is based on length and can be clamped with the last 2 parameters.
  // color can be an Eigen::Vector4f, QColor, Ogre::ColourValue, std_msgs::ColorRGBA, etc
  void addArrow(const Eigen::Vector3d& base,
                    const Eigen::Vector3d& tip,
                    const color_cast::Color& color = color_cast::Color::getDefault(),
                    double max_cone_diameter = 0.05,
                    double min_cylinder_diameter = 0.001);

  // AXIS

  // add arrow pointing from base to tip
  void addAxis(const Eigen::Affine3d& frame,
               double size = 0.5);

  // add arrow pointing from base to tip
  void addAxis(const Eigen::Quaterniond& orientation,
               const Eigen::Vector3d& position,
               double size = 0.5);

  // set default color for shapes added in the future
  void setDefaultColor(const color_cast::Color& color);

private:

  // ensure points_ is valid
  void addPointCloud();

  // used for drawing cylinders, cones, and arrows
  void addCylinderOrCone(bool cone, // else cylinder
                         const Eigen::Vector3d& base,
                         const Eigen::Vector3d& dir,
                         double length,
                         double radius,
                         const Ogre::Quaternion& quat,
                         const color_cast::Color& color);

  // maintains one instance of a shape.  Used for all shapes except points.
  struct Shape
  {
    rviz::Shape *shape_;
    Ogre::SceneNode* node_;
  };


  Ogre::SceneNode* node_;
  rviz::PointCloud* points_;
  std::vector<Shape> shapes_;
  Eigen::Vector4f color_;
  double point_size_;
};



}

#endif
