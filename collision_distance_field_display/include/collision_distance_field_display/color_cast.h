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

// Convert between the following RGBA color types
//    Eigen::Vector4f
//    std_msgs::ColorRGBA
//    QColor
//    Ogre::ColourValue
//    rviz::ColorProperty, rviz::FloatProperty  (as source only)


#ifndef COLLISION_DISTANCE_FIELD_DISPLAY_COLOR_CAST_H
#define COLLISION_DISTANCE_FIELD_DISPLAY_COLOR_CAST_H

#include <Eigen/Core>
#include <std_msgs/ColorRGBA.h>

class QColor;
namespace Ogre
{
  class ColourValue;
}
namespace rviz
{
  class ColorProperty;
  class FloatProperty;
}


namespace color_cast
{
  // Conversions to Eigen::Vector4f
  Eigen::Vector4f getColorf(const Eigen::Vector4f& colorf);
  Eigen::Vector4f getColorf(const std_msgs::ColorRGBA& colorRGBA);
  Eigen::Vector4f getColorf(const QColor& qcolor);
  Eigen::Vector4f getColorf(const Ogre::ColourValue& oc);
  Eigen::Vector4f getColorf(rviz::ColorProperty *color_prop = NULL, rviz::FloatProperty *alpha_prop = NULL);

  // Conversions to std_msgs::ColorRGBA
  std_msgs::ColorRGBA getColorRGBA(const Eigen::Vector4f colorf);
  std_msgs::ColorRGBA getColorRGBA(const QColor& qcolor);
  std_msgs::ColorRGBA getColorRGBA(const Ogre::ColourValue& oc);
  std_msgs::ColorRGBA getColorRGBA(rviz::ColorProperty *color_prop = NULL, rviz::FloatProperty *alpha_prop = NULL);

  // Conversions to Qt QColor
  QColor getQColor(const Eigen::Vector4f color);
  QColor getQColor(const QColor& qcolor);
  QColor getQColor(const Ogre::ColourValue& oc);
  QColor getQColor(rviz::ColorProperty *color_prop = NULL, rviz::FloatProperty *alpha_prop = NULL);

  // Conversions to Ogre::ColourValue
  Ogre::ColourValue getOColor(const Eigen::Vector4f colorf);
  Ogre::ColourValue getOColor(const QColor& qcolor);
  Ogre::ColourValue getOColor(const Ogre::ColourValue& oc);
  Ogre::ColourValue getOColor(rviz::ColorProperty *color_prop = NULL, rviz::FloatProperty *alpha_prop = NULL);

  class Color
  {
  public:
    Color(float r, float g, float b, float a = 1.0);
    Color(const Eigen::Vector4f& color);
    Color(const Eigen::Vector3f& color);
    Color(const Eigen::Vector4d& color);
    Color(const Eigen::Vector3d& color);
    Color(const std_msgs::ColorRGBA& color);
    Color(const QColor& color);
    Color(const Ogre::ColourValue& color);
    Color(rviz::ColorProperty *color_prop = NULL, rviz::FloatProperty *alpha_prop = NULL);

    // Get the default color.  Always white.  Use isDefault() to check for default.
    static const Color& getDefault();

    // true if this is the default color
    bool isDefault() const;

    const Eigen::Vector4f& getColorf() const { return color_; }
    std_msgs::ColorRGBA getColorRGBA() const;
    QColor getQColor() const;
    Ogre::ColourValue getOColor() const;

    float r() const { return color_.x(); }
    float g() const { return color_.y(); }
    float b() const { return color_.z(); }
    float a() const { return color_.w(); }

  private:
    Eigen::Vector4f color_;
  };
}

inline color_cast::Color::Color(const Eigen::Vector4f& color)
  : color_(color)
{}

inline color_cast::Color::Color(float r, float g, float b, float a)
  : color_(r,g,b,a)
{}

inline color_cast::Color::Color(const Eigen::Vector3f& color)
  : color_(color.x(), color.y(), color.z(), 1.0)
{}

inline color_cast::Color::Color(const Eigen::Vector4d& color)
  : color_(color.x(), color.y(), color.z(), color.w())
{}

inline color_cast::Color::Color(const Eigen::Vector3d& color)
  : color_(color.x(), color.y(), color.z(), 1.0)
{}

#endif


