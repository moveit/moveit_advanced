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

#include <collision_distance_field_display/color_cast.h>

#include <OGRE/OgreColourValue.h>
#include <QColor>
#include <rviz/properties/color_property.h>
#include <rviz/properties/float_property.h>

Eigen::Vector4f color_cast::getColorf(const std_msgs::ColorRGBA& colorRGBA)
{
  return Eigen::Vector4f(colorRGBA.r, colorRGBA.g, colorRGBA.b, colorRGBA.a);
}

Eigen::Vector4f color_cast::getColorf(const QColor& qcolor)
{
  return Eigen::Vector4f(qcolor.redF(), qcolor.greenF(), qcolor.blueF(), qcolor.alphaF());
}

Eigen::Vector4f color_cast::getColorf(const Ogre::ColourValue& oc)
{
  return Eigen::Vector4f(oc.r, oc.g, oc.b, oc.a);
}

Eigen::Vector4f color_cast::getColorf(rviz::ColorProperty *color_prop, rviz::FloatProperty *alpha_prop)
{
  Eigen::Vector4f result(1,1,1,1);
  if (alpha_prop)
  {
    double a = alpha_prop->getFloat();
    if (a>1.0) a=1.0;
    if (a<0.0) a=0.0;
    result.w() = a;
  }
  if (color_prop)
  {
    Ogre::ColourValue oc = color_prop->getOgreColor();
    result.x() = oc.r;
    result.y() = oc.g;
    result.z() = oc.b;
  }
  return result;
}

std_msgs::ColorRGBA color_cast::getColorRGBA(const Eigen::Vector4f colorf)
{
  std_msgs::ColorRGBA result;
  result.r = colorf.x();
  result.g = colorf.y();
  result.b = colorf.z();
  result.a = colorf.w();
  return result;
}

QColor color_cast::getQColor(const Eigen::Vector4f color)
{
  int r = color.x() * 255.0;
  int g = color.y() * 255.0;
  int b = color.z() * 255.0;
  int a = color.w() * 255.0;
  return QColor(
            std::min(255, std::max(0, r)),
            std::min(255, std::max(0, g)),
            std::min(255, std::max(0, b)),
            std::min(255, std::max(0, a)));
}

Ogre::ColourValue color_cast::getOColor(const Eigen::Vector4f colorf)
{
  return Ogre::ColourValue(colorf.x(), colorf.y(), colorf.z(), colorf.w());
}


// Conversions to std_msgs::ColorRGBA (derived from above)
std_msgs::ColorRGBA color_cast::getColorRGBA(const QColor& qcolor)
{
  return getColorRGBA(getColorf(qcolor));
}
std_msgs::ColorRGBA color_cast::getColorRGBA(const Ogre::ColourValue& oc)
{
  return getColorRGBA(getColorf(oc));
}
std_msgs::ColorRGBA color_cast::getColorRGBA(rviz::ColorProperty *color_prop, rviz::FloatProperty *alpha_prop)
{
  return getColorRGBA(getColorf(color_prop, alpha_prop));
}

// Conversions to Qt QColor (derived from above)
QColor color_cast::getQColor(const QColor& qcolor)
{
  return getQColor(getColorf(qcolor));
}
QColor color_cast::getQColor(const Ogre::ColourValue& oc)
{
  return getQColor(getColorf(oc));
}
QColor color_cast::getQColor(rviz::ColorProperty *color_prop, rviz::FloatProperty *alpha_prop)
{
  return getQColor(getColorf(color_prop, alpha_prop));
}

// Conversions to Ogre::ColourValue (derived from above)
Ogre::ColourValue color_cast::getOColor(const QColor& qcolor)
{
  return getOColor(getColorf(qcolor));
}
Ogre::ColourValue color_cast::getOColor(const Ogre::ColourValue& oc)
{
  return getOColor(getColorf(oc));
}
Ogre::ColourValue color_cast::getOColor(rviz::ColorProperty *color_prop, rviz::FloatProperty *alpha_prop)
{
  return getOColor(getColorf(color_prop, alpha_prop));
}

color_cast::Color::Color(const std_msgs::ColorRGBA& color)
  : color_(color.r, color.g, color.b, color.a)
{}

color_cast::Color::Color(const QColor& color)
  : color_(color.redF(), color.greenF(), color.blueF(), color.alphaF())
{}

color_cast::Color::Color(const Ogre::ColourValue& color)
  : color_(color.r, color.g, color.b, color.a)
{}

color_cast::Color::Color(rviz::ColorProperty *color_prop, rviz::FloatProperty *alpha_prop)
  : color_(1,1,1,1)
{
  if (alpha_prop)
  {
    double a = alpha_prop->getFloat();
    if (a>1.0) a=1.0;
    if (a<0.0) a=0.0;
    color_.w() = a;
  }
  if (color_prop)
  {
    Ogre::ColourValue oc = color_prop->getOgreColor();
    color_.x() = oc.r;
    color_.y() = oc.g;
    color_.z() = oc.b;
  }
}

std_msgs::ColorRGBA color_cast::Color::getColorRGBA() const
{
  std_msgs::ColorRGBA result;
  result.r = color_.x();
  result.g = color_.y();
  result.b = color_.z();
  result.a = color_.w();
  return result;
}

QColor color_cast::Color::getQColor() const
{
  int r = color_.x() * 255.0;
  int g = color_.y() * 255.0;
  int b = color_.z() * 255.0;
  int a = color_.w() * 255.0;
  return QColor(
            std::min(255, std::max(0, r)),
            std::min(255, std::max(0, g)),
            std::min(255, std::max(0, b)),
            std::min(255, std::max(0, a)));
}

Ogre::ColourValue color_cast::Color::getOColor() const
{
  return Ogre::ColourValue(color_.x(), color_.y(), color_.z(), color_.w());
}

