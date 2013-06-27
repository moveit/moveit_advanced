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

#ifndef COLLISION_DISTANCE_FIELD_DISPLAY__SHAPES_DISPLAY_H
#define COLLISION_DISTANCE_FIELD_DISPLAY__SHAPES_DISPLAY_H

#include <Eigen/Geometry>
#include <vector>
#include <eigen_stl_containers/eigen_stl_vector_container.h>


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
  void addPoint(const Eigen::Vector3d& point);

  // add a point with color
  void addPoint(const Eigen::Vector3d& point,
                const Eigen::Vector4f& color);

  // add several points
  void addPoints(const EigenSTL::vector_Vector3d& points);

  // add several points with color
  void addPoints(const EigenSTL::vector_Vector3d& points,
                 const Eigen::Vector4f& color);

  // SPHERES

  // add a sphere
  void addSphere(const Eigen::Vector3d& center,
                 double radius);

  // add a sphere with color
  void addSphere(const Eigen::Vector3d& center,
                 double radius,
                 const Eigen::Vector4f& color);

  // add several spheres
  void addSpheres(const EigenSTL::vector_Vector3d& centers,
                  std::vector<double> radii);

  // add several spheres with color
  void addSpheres(const EigenSTL::vector_Vector3d& centers,
                  std::vector<double> radii,
                  const Eigen::Vector4f& color);

  // add several spheres with the same radius
  void addSpheres(const EigenSTL::vector_Vector3d& centers,
                  double radius);

  // add several spheres with color
  void addSpheres(const EigenSTL::vector_Vector3d& centers,
                  double radius,
                  const Eigen::Vector4f& color);

  // BOXES

  // add a box.
  void addCube(
    const Eigen::Affine3d& pose, 
    const Eigen::Vector3d& size);

  void addCube(
    const Eigen::Affine3d& pose, 
    const Eigen::Vector3d& size,
    const Eigen::Vector4f& color);

  // CYLINDERS

  // add a cylinder.  Endpoints are on z axis.  Center is at origin.
  void addZCylinder(
    const Eigen::Affine3d& pose, 
    double radius, 
    double length);

  // add a cylinder with color
  void addZCylinder(
    const Eigen::Affine3d& pose, 
    double radius, 
    double length, 
    const Eigen::Vector4f& color);

  // add a cylinder given 2 endpoints.
  void addCylinder(
    const Eigen::Vector3d& end0,
    const Eigen::Vector3d& end1,
    double radius);

  // add a cylinder with color
  void addCylinder(
    const Eigen::Vector3d& end0,
    const Eigen::Vector3d& end1,
    double radius, 
    const Eigen::Vector4f& color);

  // CONES

  // add a cone
  void addCone(
    const Eigen::Vector3d& base,
    const Eigen::Vector3d& tip,
    double radius);

  // add a cone with color
  void addCone(
    const Eigen::Vector3d& base,
    const Eigen::Vector3d& tip,
    double radius, 
    const Eigen::Vector4f& color);

  // ARROWS

  // add arrow pointing from base to tip
  void addArrow(const Eigen::Vector3d& base,
                const Eigen::Vector3d& tip,
                double max_cone_diameter = 0.05,
                double min_cylinder_diameter = 0.001);

  // add arrow with color
  void addArrow(const Eigen::Vector3d& base,
                    const Eigen::Vector3d& tip,
                    const Eigen::Vector4f& color,
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
                         const Eigen::Vector4f& color);

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
