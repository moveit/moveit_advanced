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

/** \author Acorn Pooley */

#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/transforms.h>
#include <moveit/collision_detection_distance_field/collision_world_distance_field.h>
#include <moveit/collision_detection_distance_field/collision_robot_distance_field.h>

#include <urdf_parser/urdf_parser.h>
#include <geometric_shapes/shape_operations.h>

#include <gtest/gtest.h>
#include <sstream>
#include <algorithm>
#include <ctype.h>
#include <fstream>

#include <boost/filesystem.hpp>

#include <resources/config.h>

//static std::string urdf_file("test/urdf/robot.xml");
//static std::string srdf_file("test/srdf/robot.xml");

// MOVEIT_RESOURCES_DIR and TEST_RESOURCES_DIR defined in config.h by CMakelist.txt
#if 0
static const boost::filesystem::path moveit_resources(MOVEIT_RESOURCES_DIR);
static const boost::filesystem::path urdf_file("test/urdf/robot.xml");
static const boost::filesystem::path srdf_file("test/srdf/robot.xml");
#else
static const boost::filesystem::path moveit_resources(TEST_RESOURCES_DIR);
static const boost::filesystem::path urdf_file("pr2.urdf");
static const boost::filesystem::path srdf_file("pr2.srdf");
#endif


class CollisionDetectionDistanceFieldTester : public testing::Test{

protected:

  virtual void SetUp() 
  {
    console_bridge::setLogLevel(console_bridge::LOG_INFO);

    boost::filesystem::path urdf_path = moveit_resources / urdf_file;
    boost::filesystem::path srdf_path = moveit_resources / srdf_file;

    srdf_model_.reset(new srdf::Model());
    std::string xml_string;
    std::fstream xml_file(urdf_path.c_str(), std::fstream::in);

    if (xml_file.is_open())
    {
      while ( xml_file.good() )
      {
        std::string line;
        std::getline( xml_file, line);
        xml_string += (line + "\n");
      }
      xml_file.close();
      urdf_model_ = urdf::parseURDF(xml_string);
      urdf_ok_ = urdf_model_;
    }
    else
    {
      EXPECT_EQ("FAILED TO OPEN FILE", urdf_path);
      urdf_ok_ = false;
    }
    srdf_ok_ = srdf_model_->initFile(*urdf_model_, srdf_path.string());

    robot_model_.reset(new robot_model::RobotModel(urdf_model_, srdf_model_));

    acm_.reset(new collision_detection::AllowedCollisionMatrix(robot_model_->getLinkModelNames(), true));

    crobot_.reset(new collision_detection::CollisionRobotDistanceField(robot_model_));
    cworld_.reset(new collision_detection::CollisionWorldDistanceField());
  }

  virtual void TearDown()
  {

  }

protected:

  bool urdf_ok_;
  bool srdf_ok_;

  boost::shared_ptr<urdf::ModelInterface>  urdf_model_;
  boost::shared_ptr<srdf::Model>           srdf_model_;
  
  robot_model::RobotModelPtr             robot_model_;
  
  boost::shared_ptr<collision_detection::CollisionRobot>        crobot_;
  boost::shared_ptr<collision_detection::CollisionWorld>        cworld_;
  
  collision_detection::AllowedCollisionMatrixPtr acm_;

};


TEST_F(CollisionDetectionDistanceFieldTester, InitOK)
{
  ASSERT_TRUE(urdf_ok_);
  ASSERT_TRUE(srdf_ok_);
}

TEST_F(CollisionDetectionDistanceFieldTester, DefaultNotInCollision)
{
  robot_state::RobotState robot_state(robot_model_);
  robot_state.setToDefaultValues();

  collision_detection::CollisionRequest req;
  collision_detection::CollisionResult res;
  crobot_->checkSelfCollision(req, res, robot_state, *acm_);
  ASSERT_FALSE(res.collision);
}


int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
