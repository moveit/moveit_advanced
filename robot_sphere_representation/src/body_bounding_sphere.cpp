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

#include <moveit/robot_sphere_representation/body_bounding_sphere.h>
#include <moveit/robot_sphere_representation/bounding_sphere.h>
#include <geometric_shapes/shape_operations.h>
#include <geometric_shapes/bodies.h>
#include <console_bridge/console.h>
#include <Eigen/LU>

void robot_sphere_representation::findTightBoundingSphere(
        const bodies::Body& body,
        Eigen::Vector3d& center,
        double &radius)
{
  switch(body.getType())
  {
  case shapes::MESH:
    {
      const bodies::ConvexMesh& mesh =
                        dynamic_cast<const bodies::ConvexMesh&>(body);
      const EigenSTL::vector_Vector3d& verts = mesh.getScaledVertices();
      generateBoundingSphere(verts, center, radius);
      break;
    }
  default:
    {
      bodies::BoundingSphere sphere;
      body.computeBoundingSphere(sphere);
      center = sphere.center;
      radius = sphere.radius;
      break;
    }
  }
}

void robot_sphere_representation::findTightBoundingSphere(
        const Eigen::Affine3d& pose,
        const shapes::Shape& shape,
        Eigen::Vector3d& center,
        double &radius)
{
  Eigen::Vector3d unposed_center;
  findTightBoundingSphere(shape, unposed_center, radius);
  center = pose * unposed_center;
}

void robot_sphere_representation::findTightBoundingSphere(
        const shapes::Shape& shape,
        Eigen::Vector3d& center,
        double &radius)
{
  switch(shape.type)
  {
  case shapes::MESH:
    {
      const shapes::Mesh& mesh = static_cast<const shapes::Mesh&>(shape);
      EigenSTL::vector_Vector3d verts;
      verts.resize(mesh.vertex_count);
      for (unsigned int i = 0 ; i < mesh.vertex_count ; ++i)
      {
        verts[i] = Eigen::Vector3d(mesh.vertices[i*3 + 0],
                                   mesh.vertices[i*3 + 1],
                                   mesh.vertices[i*3 + 2]);
      }
      generateBoundingSphere(verts, center, radius);
      break;
    }
  default:
    shapes::computeShapeBoundingSphere(&shape, center, radius);
    break;
  }
}
