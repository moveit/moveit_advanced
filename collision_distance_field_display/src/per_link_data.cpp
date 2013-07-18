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

#include <collision_distance_field_display/collision_distance_field_display.h>
#include <collision_distance_field_display/per_link_object.h>
#include <collision_distance_field_display/df_link.h>
#include <collision_distance_field_display/shapes_display.h>
#include <collision_distance_field_display/color_cast.h>
#include "dfexamine.h"

#include <moveit/robot_sphere_representation/link_sphere_representation.h>
#include <moveit/robot_sphere_representation/body_bounding_sphere.h>

#include <moveit/collision_detection_distance_field/collision_robot_distance_field.h>

#include <rviz/properties/color_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/enum_property.h>
#include <rviz/properties/int_property.h>

#include <mesh_core/bounding_sphere.h>
#include <mesh_core/mesh.h>
#include <mesh_core/geom.h>
#include <mesh_ros/mesh_rviz.h>


namespace moveit_rviz_plugin
{
  // Draw Link Spheres from model
  class LinkObj_ModelLinkSpheres : public PerLinkSubObj
  {
  public:
    LinkObj_ModelLinkSpheres(PerLinkObjBase *base, DFLink *link) :
      PerLinkSubObj(base, link)
    {}

    static void addSelf(rviz::Property *parent, PerLinkObjList& per_link_objects)
    {
      per_link_objects.addVisObject(new PerLinkObj<LinkObj_ModelLinkSpheres>(
                                  parent,
                                  "Show Link Collision Spheres",
                                  "Show spheres used for DistanceField collision detection as stored in the SRDF.",
                                  QColor(0, 0, 255),
                                  0.5,
                                  PerLinkObjBase::SPHERES));
    }

    virtual void getGeom(bool& robot_relative, EigenSTL::vector_Vector3d& centers, std::vector<double>& radii)
    {
      robot_relative = false;
      link_->getLinkSpheres(centers, radii);
    }
  };
}

namespace moveit_rviz_plugin
{
  // Draw Link Spheres from model
  class LinkObj_ModelLinkBSpheres : public PerLinkSubObj
  {
  public:
    LinkObj_ModelLinkBSpheres(PerLinkObjBase *base, DFLink *link) :
      PerLinkSubObj(base, link)
    {}

    static void addSelf(rviz::Property *parent, PerLinkObjList& per_link_objects)
    {
      per_link_objects.addVisObject(new PerLinkObj<LinkObj_ModelLinkBSpheres>(
                                  parent,
                                  "Show Link Bounding Sphere",
                                  "Show Link's single Bounding Sphere.",
                                  QColor(128, 128, 255),
                                  0.5,
                                  PerLinkObjBase::SPHERES));
    }

    virtual void getGeom(bool& robot_relative, EigenSTL::vector_Vector3d& centers, std::vector<double>& radii)
    {
      robot_relative = false;
      centers.resize(1);
      radii.resize(1);
      link_->getLinkBoundingSphere(centers[0], radii[0]);
    }
  };
}

namespace moveit_rviz_plugin
{
  // Draw Link Spheres from RobotSphereRep
  class LinkObj_RepLinkSpheres : public PerLinkSubObj
  {
  public:
    LinkObj_RepLinkSpheres(PerLinkObjBase *base, DFLink *link) :
      PerLinkSubObj(base, link)
    {}

    static void addSelf(rviz::Property *parent, PerLinkObjList& per_link_objects)
    {
      per_link_objects.addVisObject(new PerLinkObj<LinkObj_RepLinkSpheres>(
                                  parent,
                                  "Show Generated Spheres",
                                  "Show spheres generated with RobotSphereRepresentation.",
                                  QColor(100, 100, 255),
                                  0.5,
                                  PerLinkObjBase::SPHERES));
    }

    virtual void getGeom(bool& robot_relative, EigenSTL::vector_Vector3d& centers, std::vector<double>& radii)
    {
      robot_relative = false;
      link_->getSphereRep()->getSpheres(centers, radii);
    }
  };
}

namespace moveit_rviz_plugin
{
  // Draw Link bounding sphere
  class LinkObj_BCyl : public PerLinkSubObj
  {
  public:
    LinkObj_BCyl(PerLinkObjBase *base, DFLink *link) :
      PerLinkSubObj(base, link)
    {}

    static void addSelf(rviz::Property *parent, PerLinkObjList& per_link_objects)
    {
      per_link_objects.addVisObject(new PerLinkObj<LinkObj_BCyl>(
                                  parent,
                                  "Show Link Bounding Cylinder",
                                  "Cylinder enclosing link.",
                                  QColor(128, 0, 255),
                                  0.5,
                                  PerLinkObjBase::CYLINDERS));
    }

    virtual void changed()
    {
      shapes_.reset();
      if (!getBool())
        return;

      robot_relative_ = false;
      shapes_.reset(new ShapesDisplay(getSceneNode(), base_->getColor()));

      bodies::BoundingCylinder cylinder;
      link_->getSphereRep()->getBoundingCylinder(cylinder);
      if (cylinder.radius > 0.0)
        shapes_->addZCylinder(cylinder.pose, cylinder.radius, cylinder.length);
    }
  };
}

namespace moveit_rviz_plugin
{
  // Draw link's static distance field
  class LinkObj_StaticDF : public PerLinkSubObj
  {
  public:
    LinkObj_StaticDF(PerLinkObjBase *base, DFLink *link) :
      PerLinkSubObj(base, link)
    {}

    static void addSelf(rviz::Property *parent, PerLinkObjList& per_link_objects)
    {
      per_link_objects.addVisObject(new PerLinkObj<LinkObj_StaticDF>(
                                  parent,
                                  "Show Static Distance Field",
                                  "Show surface as described by distance field points on or just inside the surface.",
                                  QColor(255, 255, 0),
                                  1.0,
                                  PerLinkObjBase::POINTS,
                                  0.005));
    }

    virtual void changed()
    {
      df_examine_.reset();
      PerLinkSubObjBase::changed();
    }

    virtual void getGeom(bool& robot_relative, EigenSTL::vector_Vector3d& centers, std::vector<double>& radii)
    {
      robot_relative = false;
      radii.clear();
      centers.clear();
      centers.reserve(100);
      const collision_detection::CollisionRobotDistanceField *crobot = link_->getDisplay()->getCollisionRobotDistanceField();
      if (crobot)
      {
        crobot->getStaticDistanceFieldPoints(
                        link_->getName(),
                        centers);
        df_examine_.reset(link_->getDisplay()->examineDF("StaticDF for ",
                                                         link_->getName().c_str(),
                                                         crobot->getStaticDistanceField(link_->getName()),
                                                         link_->getCollisionNode(),
                                                         Eigen::Affine3d::Identity()));
      }
    }
  private:
    boost::shared_ptr<moveit_rviz_plugin::CollisionDistanceFieldDisplay::DFExamine> df_examine_;
  };
}

namespace moveit_rviz_plugin
{
  // Draw link's static distance field points (points used to generate static DF)
  class LinkObj_StaticDFPoints : public PerLinkSubObj
  {
  public:
    LinkObj_StaticDFPoints(PerLinkObjBase *base, DFLink *link) :
      PerLinkSubObj(base, link)
    {}

    static void addSelf(rviz::Property *parent, PerLinkObjList& per_link_objects)
    {
      per_link_objects.addVisObject(new PerLinkObj<LinkObj_StaticDFPoints>(
                                  parent,
                                  "Show Points in link used to generate Static Distance Field",
                                  "Original points inside the link.",
                                  QColor(255, 0, 0),
                                  1.0,
                                  PerLinkObjBase::POINTS,
                                  0.006));
    }

    virtual void getGeom(bool& robot_relative, EigenSTL::vector_Vector3d& centers, std::vector<double>& radii)
    {
      robot_relative = false;
      radii.clear();
      centers.clear();
      const collision_detection::CollisionRobotDistanceField *crobot = link_->getDisplay()->getCollisionRobotDistanceField();
      if (crobot)
      {
        const collision_detection::StaticDistanceField *df = crobot->getStaticDistanceField(link_->getName());
        if (df)
          centers = df->getPoints();
      }
    }
  };
}

namespace moveit_rviz_plugin
{
  // Draw link's bounding sphere based on df points
  class LinkObj_SDFBSphere : public PerLinkSubObj
  {
  public:
    LinkObj_SDFBSphere(PerLinkObjBase *base, DFLink *link)
      : PerLinkSubObj(base, link)
    { }

    static void addSelf(rviz::Property *parent, PerLinkObjList& per_link_objects)
    {
      per_link_objects.addVisObject(new PerLinkObj<LinkObj_SDFBSphere>(
                                  parent,
                                  "Show tight bounding sphere around df points",
                                  "This demonstrates the tight bounding sphere code",
                                  QColor(255, 0, 0),
                                  0.5,
                                  PerLinkObjBase::SPHERES));
    }

    virtual void changed()
    {
      shapes_.reset();
      centers_.clear();
      radii_.clear();

      if (!getBool())
        return;

      robot_relative_ = false;
      shapes_.reset(new ShapesDisplay(getSceneNode(), base_->getColor(), base_->getSize()));

      const collision_detection::CollisionRobotDistanceField *crobot = link_->getDisplay()->getCollisionRobotDistanceField();
      if (crobot)
      {
        const collision_detection::StaticDistanceField *df = crobot->getStaticDistanceField(link_->getName());
        if (df)
        {
          EigenSTL::vector_Vector3d points;
          crobot->getStaticDistanceFieldPoints(
                          link_->getName(),
                          points);

          Eigen::Vector3d center;
          double radius;
          mesh_core::generateBoundingSphere(
            points,
            center,
            radius);
          shapes_->addSphere(center, radius);

        }
      }
    }
  };
}

namespace moveit_rviz_plugin
{
  // Draw link's bounding sphere based on vertices
  class LinkObj_VertBSphere : public PerLinkSubObj
  {
  public:
    LinkObj_VertBSphere(PerLinkObjBase *base, DFLink *link)
      : PerLinkSubObj(base, link)
    { }

    static void addSelf(rviz::Property *parent, PerLinkObjList& per_link_objects)
    {
      per_link_objects.addVisObject(new PerLinkObj<LinkObj_VertBSphere>(
                                  parent,
                                  "Show tight bounding sphere around link vertices",
                                  "This demonstrates the tight bounding sphere code",
                                  QColor(255, 0, 0),
                                  0.5,
                                  PerLinkObjBase::SPHERES));
    }

    virtual void changed()
    {
      shapes_.reset();
      centers_.clear();
      radii_.clear();

      if (!getBool())
        return;

      robot_relative_ = false;

      const robot_model::LinkModel* lm = link_->getLinkModel();
      const shapes::ShapeConstPtr& shape = lm->getShape();
      if (shape)
      {
        double bounding_radius;
        Eigen::Vector3d bounding_center;
        robot_sphere_representation::findTightBoundingSphere(*shape, bounding_center, bounding_radius);
        shapes_.reset(new ShapesDisplay(getSceneNode(), base_->getColor(), base_->getSize()));
        shapes_->addSphere(bounding_center, bounding_radius);
      }
    }
  };
}

namespace moveit_rviz_plugin
{
  // Fit plane to link vertices and draw plane as an axis
  class LinkObj_LinkVerts : public PerLinkSubObj
  {
  public:
    LinkObj_LinkVerts(PerLinkObjBase *base, DFLink *link)
      : PerLinkSubObj(base, link)
    { }

    static void addSelf(rviz::Property *parent, PerLinkObjList& per_link_objects)
    {
      per_link_objects.addVisObject(new PerLinkObj<LinkObj_LinkVerts>(
                                  parent,
                                  "Show Link Vertices",
                                  "Show vertices of the link collision geometry mesh (if it is a mesh)",
                                  QColor(0, 0, 255),
                                  1.0,
                                  PerLinkObjBase::POINTS,
                                  0.005));
    }

    virtual void changed()
    {
      shapes_.reset();
      centers_.clear();
      radii_.clear();

      if (!getBool())
        return;

      robot_relative_ = false;

      const robot_model::LinkModel* lm = link_->getLinkModel();
      const shapes::ShapeConstPtr& shape = lm->getShape();
      if (shape)
      {
        const shapes::Mesh *mesh_shape = dynamic_cast<const shapes::Mesh*>(shape.get());
        if (mesh_shape)
        {
          EigenSTL::vector_Vector3d points;
          mesh_core::appendPoints(points, mesh_shape->vertex_count, mesh_shape->vertices);

          shapes_.reset(new ShapesDisplay(getSceneNode(), base_->getColor(), base_->getSize()));
          shapes_->addPoints(points);
        }
        else
        {
          ROS_WARN("link %s is not a mesh!",lm->getName().c_str());
        }
      }
    }
  };
}

extern int acorn_db_slice_showclip;
extern int acorn_db_slice_current;
extern Eigen::Vector3d acorn_db_slice_in;
extern Eigen::Vector3d acorn_db_slice_out;
extern Eigen::Vector3d acorn_db_slice_clip;
extern EigenSTL::vector_Vector3d acorn_db_slice_pts_clip;
extern EigenSTL::vector_Vector3d acorn_db_slice_pts_0;
extern EigenSTL::vector_Vector3d acorn_db_slice_pts_in;
extern EigenSTL::vector_Vector3d acorn_db_slice_pts_out;
extern bool acorn_debug_ear_state;
extern bool acorn_closest_debug;
extern bool ACORN_DEBUG_MESH_getInsidePoints;


namespace moveit_rviz_plugin
{
  struct DebugMesh
  {
    DebugMesh(const mesh_core::Mesh& mesh, 
              const std::string& name)
      : mesh_(mesh)
      , name_(name)
    {}

    std::string name_;
    mesh_core::Mesh mesh_;
  };
}


namespace moveit_rviz_plugin
{

  mesh_core::Mesh::SphereRepNode *findSphereTreeNode(
        mesh_core::Mesh::SphereRepNode *node,
        int id)
  {
    if (node->id_ == id)
      return node;
    if (!node->first_child_)
      return NULL;
    mesh_core::Mesh::SphereRepNode *nn = findSphereTreeNode(node->first_child_, id);
    ACORN_ASSERT(node->first_child_->next_sibling_);
    if (nn)
      return nn;
    return findSphereTreeNode(node->first_child_->next_sibling_, id);
  }
  void showSphereTree(
        const mesh_core::Mesh::SphereRepNode *node, 
        const mesh_core::Mesh::SphereRepNode *mark,
        char *name = NULL,
        char *prefix = NULL,
        int depth = 0)
  {
    static char name0[1000];
    static char prefix0[1000];
    if (!name)
    {
      name = name0;
      name0[0] = 0;
      prefix = prefix0;
      prefix0[0] = '+';
      prefix0[1] = '-';
      prefix0[2] = 0;
    }
    int l = strlen(name);
    int lp = strlen(prefix);

    prefix[lp - 2] = (!node->parent_ || node->next_sibling_ == node->parent_->first_child_) ? '`' : '+';
    prefix[lp - 1] = '-';

    logInform("  %8x=%10d %s %s%08lx %s",
      node->id_,
      node->id_,
      node==mark?"*":" ",
      prefix,
      long(node),
      name);

    prefix[lp - 2] = (prefix[lp - 2] == '+') ? '|' : ' ';
    prefix[lp - 1] = ' ';
    prefix[lp + 0] = '+';
    prefix[lp + 1] = '-';
    prefix[lp + 2] = 0;

    name[l+1] = 0;
    if (node->first_child_)
    {
      name[l] = 'L';
      prefix[lp] = 'L';
      showSphereTree(node->first_child_, mark, name, prefix, depth+1);
      ACORN_ASSERT(node->first_child_->next_sibling_);
      ACORN_ASSERT(node->first_child_->next_sibling_ != node->first_child_);
      ACORN_ASSERT(node->first_child_->next_sibling_->next_sibling_ == node->first_child_);
      name[l] = 'R';
      showSphereTree(node->first_child_->next_sibling_, mark, name, prefix, depth+1);
    }
    name[l] = 0;
    prefix[lp] = 0;
  }




  // Fit plane to link vertices and draw plane as an axis
  class LinkObj_LinkMesh : public PerLinkSubObj
  {
  public:
    LinkObj_LinkMesh(PerLinkObjBase *base, DFLink *link)
      : PerLinkSubObj(base, link)
    { }

    static void addSelf(rviz::Property *parent, PerLinkObjList& per_link_objects)
    {
      PerLinkObj<LinkObj_LinkMesh>* obj = new PerLinkObj<LinkObj_LinkMesh>(
                                  parent,
                                  "Show Link Mesh",
                                  "Show mesh for link, or subdivided partial mesh",
                                  QColor(0, 255, 0),
                                  1.0,
                                  PerLinkObjBase::POINTS,
                                  0.005);

#if 0
      obj->addIntProperty("WhichHalf", 0, "which half of split to show 0=all 1=left 2=right");
      obj->addIntProperty("WhichClip", -1, "Show one triangle and its clip result");
#endif
      obj->addBoolProperty("ShowBoundingSphere", false, "Show bounding sphere for mesh?");
      obj->addBoolProperty("ShowAABB", false, "Show AABB for mesh?");
      obj->addFloatProperty("SphereRepTolerance", 0.01, "Tolerance for filling spheres");
      obj->addIntProperty("SphereRepMaxDepth", 3, "calculate spheres to this depth");
      obj->addIntProperty("ShowSphereRepSpheresLevel", -2, "Show spheres up to this level");
      obj->addBoolProperty("ShowPoints", false, "Show all internal points in mesh");
      obj->addFloatProperty("PointResolution", 0.1, "resolution for ShowPoints");
      obj->addBoolProperty("DebugShowPoints", false, "printfs in ShowPoints");

      obj->addBoolProperty("PrintTree", false, "print SphereRep tree?");
      obj->addIntProperty("SphereRepIndex", -1, "Show mesh used for this sphere (-1 to disable)");
      obj->addIntProperty("WhichGap", -1, "show one filled gap (-1 to disable)");
      obj->addBoolProperty("PrintEarClipDebug", false, "printfs for ear clip of current gap");
      obj->addIntProperty("ShowGapLoopPoint", -2, "show one or all (-1) loop points for the current gap (-2 to disable)");
      obj->addIntProperty("NTris", -1, "Number of tris to show (-1 for all)");
      obj->addIntProperty("FirstTri", 0, "first tri to show");
      obj->addFloatProperty("ClosestAngle", 0, "if >0 show closest distance for features with this max_angle");

      per_link_objects.addVisObject(obj);
    }

    virtual void changed()
    {
      shapes_.reset();
      centers_.clear();
      radii_.clear();
      mesh_shape_.reset();
      mesh_sphere_.reset();
      neigbor_mesh_shape_.reset();

      link_->getDebugMeshes().clear();

      if (!getBool())
        return;

      robot_relative_ = false;

      const robot_model::LinkModel* lm = link_->getLinkModel();
      const shapes::ShapeConstPtr& shape = lm->getShape();
      if (shape)
      {
        const shapes::Mesh *mesh_shape = dynamic_cast<const shapes::Mesh*>(shape.get());
        if (mesh_shape)
        {
          mesh_core::Mesh::enableDebugging(true);
          mesh_core::Mesh mesh;
          mesh.add(mesh_shape->triangle_count, (int*)mesh_shape->triangles, mesh_shape->vertices);
          mesh.fillGaps();


          shapes_.reset(new ShapesDisplay(getSceneNode(), base_->getColor(), base_->getSize()));


#if 0
          acorn_db_slice_showclip = base_->getIntProperty("WhichClip")->getInt();
#endif

          mesh.fillGaps();

          const mesh_core::Mesh *mp = &mesh;


          mesh_core::Mesh::SphereRepNode *sphere_tree = NULL;
          int sphere_rep_index = base_->getIntProperty("SphereRepIndex")->getInt();
          int sphere_rep_level = base_->getIntProperty("ShowSphereRepSpheresLevel")->getInt();
          if (sphere_rep_index >= 0 || sphere_rep_level >= -1)
          {
            double tolerance = base_->getFloatProperty("SphereRepTolerance")->getFloat();
            EigenSTL::vector_Vector3d sphere_centers;
            std::vector<double> sphere_radii;
            int sphere_rep_max_depth = base_->getIntProperty("SphereRepMaxDepth")->getInt();
            logInform("Generate sphere tree to depth %d", sphere_rep_max_depth);

            if (base_->getBoolProperty("PrintEarClipDebug")->getBool())
            {
              mesh.setDebugValues(
                      sphere_rep_index,
                      base_->getIntProperty("WhichGap")->getInt());
            }

            mp->getSphereRep(tolerance, sphere_centers, sphere_radii, &sphere_tree, sphere_rep_max_depth);

            mesh.setDebugValues(-1,-1);

            if (sphere_rep_level >= -1)
            {
              sphere_centers.clear();
              sphere_radii.clear();
              mp->collectSphereRepSpheres(sphere_tree, sphere_centers, sphere_radii, sphere_rep_level);
ACORN_ASSERT(sphere_centers.size() == sphere_radii.size());
              shapes_->addSpheres(sphere_centers, sphere_radii, Eigen::Vector4f(1,1,0,0.5));
            }

            if (sphere_rep_index >= 0)
            {
#if 0
              int bits = sphere_rep_index;
              mesh_core::Mesh::SphereRepNode *node = sphere_tree;
              std::stringstream ss;
              while(bits > 1 && node->first_child_)
              {
                int right = (bits & 1);

                Eigen::Vector3d center;
                double radius;
                node->mesh_->getBoundingSphere(center, radius);
                shapes_->addArrow(
                  center,
                  center + (right ? -1.0 : 1.0) * radius * node->plane_.getNormal(),
                  Eigen::Vector4f(1,0,1,0.5));

                node = right ? node->first_child_->next_sibling_ : node->first_child_;
                ss << (right ? "R" : "L");
                bits >>= 1;
              }
#endif
              mesh_core::Mesh::SphereRepNode *node = findSphereTreeNode(sphere_tree, sphere_rep_index);

              if (base_->getBoolProperty("PrintTree")->getBool())
                showSphereTree(sphere_tree, node);

              if (node)
              {
                if (node->parent_)
                {
                  mesh_core::Mesh::SphereRepNode *n = node;
                  while (n->parent_)
                  {
                    mesh_core::Mesh::SphereRepNode *p = n->parent_;
                    bool is_left = n == p->first_child_;

                    Eigen::Vector3d center;
                    double radius;
                    p->mesh_->getBoundingSphere(center, radius);
                    shapes_->addArrow(
                      center,
                      center + (is_left ? 1.0 : -1.0) * radius * p->plane_.getNormal(),
                      Eigen::Vector4f(1,0,1,0.5));

                    n = p;
                  }

                  if (node->parent_->plane_points_.size() > 0)
                    shapes_->addPoint(node->parent_->plane_points_[0], Eigen::Vector4f(0.5,0,0,1));
                  if (node->parent_->plane_points_.size() > 1)
                    shapes_->addPoint(node->parent_->plane_points_[1], Eigen::Vector4f(0,0.5,0,1));
                  if (node->parent_->plane_points_.size() > 2)
                    shapes_->addPoint(node->parent_->plane_points_[2], Eigen::Vector4f(0,0,0.5,1));
                  if (node->parent_->plane_points_.size() > 3)
                    shapes_->addPoint(node->parent_->plane_points_[3], Eigen::Vector4f(0.5,0.5,0.5,1));
                  shapes_->addPoint(node->parent_->closest_point, Eigen::Vector4f(0,0,0,1));
                  shapes_->addPoint(node->parent_->farthest_point_, Eigen::Vector4f(1,1,1,1));
                  if (node->parent_->closes_triangle_.size() == 3)
                  {
                    shapes_->addSpheres(node->parent_->closes_triangle_, 0.001, Eigen::Vector4f(1,1,0,1));

#if 0
Eigen::Vector3d center;
Eigen::Vector3d closest;
double radius;
node->parent_->mesh_->getBoundingSphere(center, radius);
acorn_closest_debug = true;
mesh_core::closestPointOnTriangle(
  node->parent_->closes_triangle_[0],
  node->parent_->closes_triangle_[1],
  node->parent_->closes_triangle_[2],
  center,
  closest,
  1000.0);
acorn_closest_debug = false;
#endif
                  }
                }

                mp = node->mesh_;
              }
            }
          }
          
          
#if 0
          mesh_core::Mesh a(0.00001);
          mesh_core::Mesh b(0.00001);
          int which_half = base_->getIntProperty("WhichHalf")->getInt();
          if (which_half >= 1 && which_half <= 2)
          {
            mp->slice(plane, a, b);
            mp = which_half == 1 ? &a : &b;


            if (acorn_db_slice_showclip>=0 && acorn_db_slice_showclip < acorn_db_slice_current)
            {
              logInform("Show clip %d of %d",acorn_db_slice_showclip,acorn_db_slice_current);
              //shapes_->addPoint(acorn_db_slice_in, Eigen::Vector4f(1,0,0,1));
              //shapes_->addPoint(acorn_db_slice_out, Eigen::Vector4f(0,1,0,1));
              //shapes_->addPoint(acorn_db_slice_clip, Eigen::Vector4f(0,0,1,1));
      
              shapes_->addPoints(acorn_db_slice_pts_clip, Eigen::Vector4f(1,0,1,1));
              shapes_->addPoints(acorn_db_slice_pts_in, Eigen::Vector4f(0,0,1,1));
              shapes_->addPoints(acorn_db_slice_pts_0, Eigen::Vector4f(1,1,1,1));
              shapes_->addPoints(acorn_db_slice_pts_out, Eigen::Vector4f(1,0,0,1));
            }
          }
#endif

          double closest_max_angle = base_->getFloatProperty("ClosestAngle")->getFloat();
          if (closest_max_angle > 0.0 && closest_max_angle < 90.0)
          {
            int tria, trib;
            double dist;
            
            mp->findThinnestFeature(dist, tria, trib, closest_max_angle);
            logInform("Closest dist=%f  tria=%d  trib=%d for max_angle=%4.1f",dist, tria, trib, closest_max_angle);
          }

#if 0
          // draw a mesh sphere
          if (1)
          {
            mesh_core::Mesh sphere_mesh;
            sphere_mesh.addSphere(
                          Eigen::Vector3d(2,0,0),
                          0.5,
                          0.05);
            mesh_sphere_.reset(new mesh_ros::RvizMeshShape(
                                            link_->getDisplay()->getDisplayContext(), 
                                            getSceneNode(), 
                                            &sphere_mesh,
                                            base_->getColor()));
          }
#endif

          // draw mesh points
          if (base_->getBoolProperty("ShowPoints")->getBool())
          {
            ACORN_DEBUG_MESH_getInsidePoints = base_->getBoolProperty("DebugShowPoints")->getBool();

            double res = base_->getFloatProperty("PointResolution")->getFloat();
            EigenSTL::vector_Vector3d points;
            mp->getInsidePoints(points, res);
            shapes_->addPoints(points, Eigen::Vector4f(0,1,1,1));
            ACORN_DEBUG_MESH_getInsidePoints = false;
          }

          {
            boost::shared_ptr<DebugMesh> dbmesh( new DebugMesh(*mp, "CurrentMesh"));
            link_->getDebugMeshes().push_back(dbmesh);
          }

          // draw the mesh
          mesh_shape_.reset(new mesh_ros::RvizMeshShape(
                                            link_->getDisplay()->getDisplayContext(), 
                                            getSceneNode(), 
                                            NULL,
                                            base_->getColor()));

          int which_gap = base_->getIntProperty("WhichGap")->getInt();
          if (which_gap >= 0)
          {
            const std::vector<mesh_core::Mesh::GapDebugInfo>& gdi_list = mp->getGapDebugInfo();
            if (which_gap < gdi_list.size())
            {
              const mesh_core::Mesh::GapDebugInfo& gdi = gdi_list[which_gap];

              ROS_INFO("draw gap %d/%d with %d verts",which_gap, int(gdi_list.size()), int(gdi.verts_.size()));

              neigbor_mesh_shape_.reset(new mesh_ros::RvizMeshShape(
                                            link_->getDisplay()->getDisplayContext(), 
                                            getSceneNode(), 
                                            NULL,
                                            Eigen::Vector4f(1,1,0,1)));
              neigbor_mesh_shape_->reset(
                            mp,
                            gdi.neigbor_tris_,
                            base_->getIntProperty("FirstTri")->getInt(),
                            base_->getIntProperty("NTris")->getInt());

              if (base_->getIntProperty("NTris")->getInt() == 1)
              {
                int t = base_->getIntProperty("FirstTri")->getInt();
                int num = gdi.neigbor_tris_.size();
                logInform("XXXXXXXX Show neigbor tri %d of %d which is tri index %d",
                  t,
                  num,
                  (t>=0 && t<num) ? gdi.neigbor_tris_[t] : -1);
                  
              }

              mesh_shape_->reset(
                          mp,
                          gdi.gap_tris_,
                          base_->getIntProperty("FirstTri")->getInt(),
                          base_->getIntProperty("NTris")->getInt());

              if (base_->getIntProperty("NTris")->getInt() == 1)
              {
                int t = base_->getIntProperty("FirstTri")->getInt();
                int num = gdi.gap_tris_.size();
                logInform("XXXXXXXX Show gap tri %d of %d which is tri index %d",
                  t,
                  num,
                  (t>=0 && t<num) ? gdi.gap_tris_[t] : -1);
                  
              }

              int showpt = base_->getIntProperty("ShowGapLoopPoint")->getInt();
              if (showpt >=0 && showpt < gdi.points_.size())
              {
                shapes_->addPoint(gdi.points_[showpt], Eigen::Vector4f(1,0,1,1));
              }
              else if (showpt == -1)
              {
                shapes_->addPoints(gdi.points_, Eigen::Vector4f(1,0,1,1));
              }



            }
          }
          else
          {
            ROS_INFO("draw mesh with %d tris %d verts",mp->getTriCount(), mp->getVertCount());
            mesh_shape_->reset(
                          mp,
                          base_->getIntProperty("FirstTri")->getInt(),
                          base_->getIntProperty("NTris")->getInt());
          }

          if (base_->getBoolProperty("ShowBoundingSphere")->getBool())
          {
            Eigen::Vector3d center;
            double radius;
            mp->getBoundingSphere(center, radius);
            shapes_->addSphere(center, radius, Eigen::Vector4f(0,0,1,0.5));
          }

          if (base_->getBoolProperty("ShowAABB")->getBool())
          {
            Eigen::Vector3d min, max;
            mp->getAABB(min, max);
            shapes_->addBox(
                  Eigen::Affine3d(Eigen::Translation3d((min+max)*0.5)),
                  max-min,
                  Eigen::Vector4f(0,0,1,0.5));
          }

          mesh_core::Mesh::deleteSphereRepTree(sphere_tree);
        }
        else
        {
          ROS_WARN("link %s is not a mesh!",lm->getName().c_str());
        }
      }
    }
  private:
    boost::shared_ptr<mesh_ros::RvizMeshShape> mesh_shape_;
    boost::shared_ptr<mesh_ros::RvizMeshShape> neigbor_mesh_shape_;
    boost::shared_ptr<mesh_ros::RvizMeshShape> mesh_sphere_;
  };
}


namespace moveit_rviz_plugin
{
  // Fit plane to link vertices and draw plane as an axis
  class LinkObj_MeshDebug : public PerLinkSubObj
  {
  public:
    LinkObj_MeshDebug(PerLinkObjBase *base, DFLink *link)
      : PerLinkSubObj(base, link)
    { }

    static void addSelf(rviz::Property *parent, PerLinkObjList& per_link_objects)
    {
      PerLinkObj<LinkObj_MeshDebug>* obj = new PerLinkObj<LinkObj_MeshDebug>(
                                  parent,
                                  "DebugMesh",
                                  "debug existing meshes",
                                  QColor(0, 0, 255),
                                  1.0,
                                  PerLinkObjBase::POINTS,
                                  0.005);

      obj->addIntProperty("NMeshes", -1, "Number of meshes available");
      obj->getIntProperty("NMeshes")->setReadOnly(true);
      obj->addIntProperty("Mesh", -1, "Which mesh to debug");
      obj->addIntProperty("NGaps", -1, "Number of gaps in this mesh");
      obj->getIntProperty("NGaps")->setReadOnly(true);
      obj->addIntProperty("Gap", -1, "Which gap to show");

      obj->addIntProperty("ShowGapLoopPoint", -2, "show one (or -1 for all) loop points for the current gap");


      obj->addIntProperty("NTris", -1, "Number of tris to show");
      obj->addIntProperty("FirstTri", 0, "first tri to show");
#if 0
      obj->addIntProperty("WhichHalf", 0, "which half of split to show 0=all 1=left 2=right");
      obj->addIntProperty("WhichGap", -1, "show one filled gap");
      obj->addIntProperty("WhichClip", -1, "Show one triangle and its clip result");
      obj->addBoolProperty("ShowBoundingSphere", false, "Show bounding sphere for mesh?");
      obj->addBoolProperty("ShowAABB", false, "Show AABB for mesh?");
      obj->addIntProperty("SphereRepIndex", -1, "Show results of sphere fitting");
      obj->addFloatProperty("SphereRepTolerance", 0.01, "Tolerance for filling spheres");
      obj->addIntProperty("SphereRepMaxDepth", 3, "calculate spheres to this depth");
      obj->addIntProperty("ShowSphereRepSpheresLevel", -2, "Show spheres up to this level");
      obj->addBoolProperty("PrintTree", false, "print SphereRep tree?");
      obj->addFloatProperty("PointResolution", 0.1, "resolution for ShowPoints");
      obj->addBoolProperty("DebugShowPoints", false, "printfs in ShowPoints");
      obj->addBoolProperty("ShowPoints", false, "Show all internal points in mesh");
#endif
      obj->addFloatProperty("PointX", 0.0, "Show point at this X position");
      obj->addFloatProperty("PointY", 0.0, "Show point at this Y position");
      obj->addFloatProperty("PointZ", 0.0, "Show point at this Z position");

      per_link_objects.addVisObject(obj);
    }

    virtual void changed()
    {
      shapes_.reset();
      centers_.clear();
      radii_.clear();
      mesh_shape_.reset();
      mesh_sphere_.reset();
      neigbor_mesh_shape_.reset();

      if (!getBool() || !link_)
        return;

      robot_relative_ = false;

      int nmeshes = link_->getDebugMeshes().size();
      base_->getIntProperty("NMeshes")->setValue(nmeshes);

      int meshid = base_->getIntProperty("Mesh")->getInt();
      if (meshid < 0 || meshid >= nmeshes)
      {
        base_->getIntProperty("NGaps")->setValue(0);
        return;
      }

      shapes_.reset(new ShapesDisplay(getSceneNode(), base_->getColor(), base_->getSize()));

      DebugMesh& dbmesh = *link_->getDebugMeshes()[meshid];
      logInform("Showing dbmesh[%d] = %s",
        meshid, dbmesh.name_.c_str());
      
      mesh_core::Mesh *mesh = &dbmesh.mesh_;

      base_->getIntProperty("NGaps")->setValue(int(mesh->getGapDebugInfo().size()));

      {
        Eigen::Vector3d point(
                            base_->getFloatProperty("PointX")->getFloat(),
                            base_->getFloatProperty("PointY")->getFloat(),
                            base_->getFloatProperty("PointZ")->getFloat());
        shapes_->addPoint(point, Eigen::Vector4f(1,0,0,1));
      }


      // draw the mesh
      mesh_shape_.reset(new mesh_ros::RvizMeshShape(
                                        link_->getDisplay()->getDisplayContext(), 
                                        getSceneNode(), 
                                        NULL,
                                        base_->getColor()));

      int which_gap = base_->getIntProperty("Gap")->getInt();
      if (which_gap >= 0)
      {
        const std::vector<mesh_core::Mesh::GapDebugInfo>& gdi_list = mesh->getGapDebugInfo();
        if (which_gap < gdi_list.size())
        {
          const mesh_core::Mesh::GapDebugInfo& gdi = gdi_list[which_gap];

          ROS_INFO("draw gap %d/%d with %d verts",which_gap, int(gdi_list.size()), int(gdi.verts_.size()));

          neigbor_mesh_shape_.reset(new mesh_ros::RvizMeshShape(
                                        link_->getDisplay()->getDisplayContext(), 
                                        getSceneNode(), 
                                        NULL,
                                        Eigen::Vector4f(1,1,0,1)));
          neigbor_mesh_shape_->reset(
                        mesh,
                        gdi.neigbor_tris_,
                        base_->getIntProperty("FirstTri")->getInt(),
                        base_->getIntProperty("NTris")->getInt());

          if (base_->getIntProperty("NTris")->getInt() == 1)
          {
            int t = base_->getIntProperty("FirstTri")->getInt();
            int num = gdi.neigbor_tris_.size();
            logInform("XXXXXXXX Show neigbor tri %d of %d which is tri index %d",
              t,
              num,
              (t>=0 && t<num) ? gdi.neigbor_tris_[t] : -1);
              
          }

          mesh_shape_->reset(
                      mesh,
                      gdi.gap_tris_,
                      base_->getIntProperty("FirstTri")->getInt(),
                      base_->getIntProperty("NTris")->getInt());

          if (base_->getIntProperty("NTris")->getInt() == 1)
          {
            int t = base_->getIntProperty("FirstTri")->getInt();
            int num = gdi.gap_tris_.size();
            logInform("XXXXXXXX Show gap tri %d of %d which is tri index %d",
              t,
              num,
              (t>=0 && t<num) ? gdi.gap_tris_[t] : -1);
              
          }

          int showpt = base_->getIntProperty("ShowGapLoopPoint")->getInt();
          if (showpt >=0 && showpt < gdi.points_.size())
          {
            shapes_->addPoint(gdi.points_[showpt], Eigen::Vector4f(1,0,1,1));
          }
          else if (showpt == -1)
          {
            shapes_->addPoints(gdi.points_, Eigen::Vector4f(1,0,1,1));
          }

        }
      }
      else
      {
        ROS_INFO("draw mesh with %d tris %d verts",mesh->getTriCount(), mesh->getVertCount());
        mesh_shape_->reset(
                      mesh,
                      base_->getIntProperty("FirstTri")->getInt(),
                      base_->getIntProperty("NTris")->getInt());
      }
    }
  private:
    boost::shared_ptr<mesh_ros::RvizMeshShape> mesh_shape_;
    boost::shared_ptr<mesh_ros::RvizMeshShape> neigbor_mesh_shape_;
    boost::shared_ptr<mesh_ros::RvizMeshShape> mesh_sphere_;
  };
}

namespace moveit_rviz_plugin
{
  // Fit plane to link vertices and draw plane as an axis
  class LinkObj_VertPlane : public PerLinkSubObj
  {
  public:
    LinkObj_VertPlane(PerLinkObjBase *base, DFLink *link)
      : PerLinkSubObj(base, link)
    { }

    static void addSelf(rviz::Property *parent, PerLinkObjList& per_link_objects)
    {
      per_link_objects.addVisObject(new PerLinkObj<LinkObj_VertPlane>(
                                  parent,
                                  "Show plane through link vertices",
                                  "Axis of a plane through the middle of the link vertices, calculated by least"
                                  " squares fit. This is for debugging Line and Plane code",
                                  QColor(255, 0, 0),
                                  0.5,
                                  PerLinkObjBase::SPHERES,
                                  0.005,
                                  true));
    }

    virtual void changed()
    {
      shapes_.reset();
      centers_.clear();
      radii_.clear();

      if (!getBool())
        return;

      robot_relative_ = true;

      const robot_model::LinkModel* lm = link_->getLinkModel();
      const shapes::ShapeConstPtr& shape = lm->getShape();
      if (shape)
      {
        const shapes::Mesh *mesh_shape = dynamic_cast<const shapes::Mesh*>(shape.get());
        if (mesh_shape)
        {
          const Eigen::Affine3d& link_xform = link_->getLinkState()->getGlobalCollisionBodyTransform();
          EigenSTL::vector_Vector3d points;
          mesh_core::appendPointsTransformed(points, link_xform, mesh_shape->vertex_count, mesh_shape->vertices);
          

          mesh_core::PlaneProjection proj(points);
          shapes_.reset(new ShapesDisplay(getSceneNode(), base_->getColor(), base_->getSize()));
          shapes_->addAxis(proj.getOrientation(), proj.getOrigin());

          Eigen::Vector3d a = link_->getRobotState()->getLinkState("l_gripper_l_finger_tip_link")->getGlobalCollisionBodyTransform().translation();
          Eigen::Vector3d b = link_->getRobotState()->getLinkState("r_gripper_l_finger_tip_link")->getGlobalCollisionBodyTransform().translation();
          Eigen::Vector3d c = link_->getRobotState()->getLinkState("head_pan_link")->getGlobalCollisionBodyTransform().translation();
          Eigen::Vector3d d = link_->getRobotState()->getLinkState("base_laser_link")->getGlobalCollisionBodyTransform().translation();

          shapes_->addPoint(a, Eigen::Vector4f(1,0,0,1));
          shapes_->addPoint(b, Eigen::Vector4f(1,0,0,1));
          shapes_->addPoint(c, Eigen::Vector4f(1,0,0,1));
          shapes_->addPoint(d, Eigen::Vector4f(1,0,0,1));
          shapes_->addArrow(a,b, Eigen::Vector4f(1,0,0,1));
          shapes_->addArrow(c,d, Eigen::Vector4f(1,0,0,1));

          Eigen::Vector2d a2 = proj.project(a);
          Eigen::Vector2d b2 = proj.project(b);
          Eigen::Vector2d c2 = proj.project(c);
          Eigen::Vector2d d2 = proj.project(d);

          mesh_core::LineSegment2D line0(a2,b2);
          mesh_core::LineSegment2D line1(c2,d2);

          bool parallel;
          Eigen::Vector2d intersection;
          bool hit = line0.intersect(line1, intersection, parallel);

          if (!parallel)
          {
            Eigen::Vector3d intersection3 = proj.extract(intersection);
            
            shapes_->addSphere(intersection3,
                               0.05,
                               hit ? Eigen::Vector4f(1,0,0,1) : Eigen::Vector4f(0,1,0,1));
          }
          

          hit = line1.intersect(line0, intersection, parallel);

          if (!parallel)
          {
            Eigen::Vector3d intersection3 = proj.extract(intersection);
            
            shapes_->addBox(intersection3,
                            0.07,
                            hit ? Eigen::Vector4f(1,1,0,1) : Eigen::Vector4f(0,1,1,1));
            shapes_->addBox(
                        Eigen::Vector3d(intersection.x(), intersection.y(), 0),
                        0.07,
                        hit ? Eigen::Vector4f(1,1,0,1) : Eigen::Vector4f(0,1,1,1));
          }
          

          Eigen::Vector3d a23 = proj.extract(a2);
          Eigen::Vector3d b23 = proj.extract(b2);
          Eigen::Vector3d c23 = proj.extract(c2);
          Eigen::Vector3d d23 = proj.extract(d2);
          shapes_->addArrow(a23,b23, Eigen::Vector4f(0,1,1,1));
          shapes_->addArrow(c23,d23, Eigen::Vector4f(0,1,1,1));


          shapes_->addArrow(
                  Eigen::Vector3d(a2.x(), a2.y(), 0),
                  Eigen::Vector3d(b2.x(), b2.y(), 0),
                  Eigen::Vector4f(0,1,1,1));
          shapes_->addArrow(
                  Eigen::Vector3d(c2.x(), c2.y(), 0),
                  Eigen::Vector3d(d2.x(), d2.y(), 0),
                  Eigen::Vector4f(0,1,1,1));


        }
        else
        {
          ROS_WARN("link %s is not a mesh!",lm->getName().c_str());
        }
      }
    }
  };
}


void moveit_rviz_plugin::CollisionDistanceFieldDisplay::addPerLinkData()
{
  per_link_objects_.reset(new PerLinkObjList());

  LinkObj_StaticDF::addSelf(collision_detection_category_, *per_link_objects_);
  LinkObj_StaticDFPoints::addSelf(collision_detection_category_, *per_link_objects_);
  LinkObj_ModelLinkSpheres::addSelf(collision_detection_category_, *per_link_objects_);
  LinkObj_ModelLinkBSpheres::addSelf(collision_detection_category_, *per_link_objects_);

  addSphereGenProperties(sphere_gen_category_);

  LinkObj_RepLinkSpheres::addSelf(sphere_gen_category_, *per_link_objects_);
  LinkObj_BCyl::addSelf(sphere_gen_category_, *per_link_objects_);
  LinkObj_SDFBSphere::addSelf(sphere_gen_category_, *per_link_objects_);
  LinkObj_VertBSphere::addSelf(sphere_gen_category_, *per_link_objects_);

  LinkObj_LinkVerts::addSelf(mesh_vis_category_, *per_link_objects_);
  LinkObj_LinkMesh::addSelf(mesh_vis_category_, *per_link_objects_);
  LinkObj_MeshDebug::addSelf(mesh_vis_category_, *per_link_objects_);
  LinkObj_VertPlane::addSelf(mesh_vis_category_, *per_link_objects_);
}


