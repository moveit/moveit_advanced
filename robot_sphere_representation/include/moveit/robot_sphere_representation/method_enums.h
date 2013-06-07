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

#ifndef MOVEIT_ROBOT_SPHERE_REPRESENTATION_METHOD_ENUMS_
#define MOVEIT_ROBOT_SPHERE_REPRESENTATION_METHOD_ENUMS_

#include <moveit/robot_sphere_representation/enum_string.h>

namespace robot_sphere_representation
{
// GenMethod is the method to use to generate spheres.
#define robot_sphere_representation__GenMethod__Values(VD,VS,VV,VVS,VA) \
  VS(SRDF,                      "From SRDF") \
  VS(SRDF_EXT,                  "From external SRDF") \
  VS(BOUNDING_SPHERES,          "Use bounding sphere") \
  VD(THIN_LIMITGREEDY_GRADIENT) \
  VD(THIN_GREEDY_GRADIENT) \
  VD(THIN_GREEDY) \
  VD(THIN_GRADIENT) \
  VD(THIN_GOBBLE) \
  VD(LIMITGREEDY_GRADIENT) \
  VD(GREEDY_GRADIENT) \
  VD(GREEDY) \
  VD(GRADIENT) \
  VD(GOBBLE) \
  VD(CLUSTERING) \
  VD(ONE_SPHERE) \
  VD(ZERO_SPHERES) \
  VA(DEFAULT,SRDF) \

ENUM_STRING_DECLARE(GenMethod,robot_sphere_representation__GenMethod__Values);
}

namespace robot_sphere_representation
{
// QualMethod is the method to use to calculate quality of a sphere representation.
#define robot_sphere_representation__QualMethod__Values(VD,VS,VV,VVS,VA) \
  VS(MAX_DIST, "Max distance from link") \
  VS(BADCOUNT, "Count of weighted points inside spheres but outside link") \
  VS(RADIUS,   "How much the radius sticks out") \
  VA(DEFAULT,MAX_DIST) \

ENUM_STRING_DECLARE(QualMethod,robot_sphere_representation__QualMethod__Values);
}




#endif

