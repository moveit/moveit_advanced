#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2013, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# * Redistributions of source code must retain the above copyright
# notice, this list of conditions and the following disclaimer.
# * Redistributions in binary form must reproduce the above
# copyright notice, this list of conditions and the following
# disclaimer in the documentation and/or other materials provided
# with the distribution.
# * Neither the name of Willow Garage, Inc. nor the names of its
# contributors may be used to endorse or promote products derived
# from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

# Author: Vincent Rabaud, Ioan Sucan

import actionlib
import rospy
import sys
import collections
import copy
from threading import Lock
from object_recognition_msgs.msg import ObjectRecognitionAction, ObjectRecognitionGoal, RecognizedObjectArray
from object_recognition_msgs.srv import GetObjectInformation

from moveit_msgs.msg import CollisionObject, PlanningScene
from shape_msgs.msg import SolidPrimitive, Mesh, MeshTriangle
from geometry_msgs.msg import Point
from std_msgs.msg import Bool


class ObjectDetector:
    """ Listen to recognized objects over a topic or using an action server. Trigger a callback when objects are found """

    def __init__(self, on_object_found):
        self._on_object_found_callback = on_object_found
        if not hasattr(on_object_found, '__call__'):
            raise RuntimeError("Callable object must be supplied to constructor")
        self._action_client = None

    def detected_object(self, ob):
        if self._on_object_found_callback is not None:
            self._on_object_found_callback(ob)

    def start_action_client(self, name = 'recognize_objects'):
        rospy.loginfo("Starting %s action" % name)
        self._action_client = actionlib.SimpleActionClient(name, ObjectRecognitionAction)
        self._action_client.wait_for_server()

    def start_continuous_monitor_client(self, topic = '/recognized_object_array'):
        self._subscriber = rospy.Subscriber(topic, RecognizedObjectArray, self.on_topic_data)

    def trigger_detection(self):
        if self._action_client is None:
            self.start_action_client()
        goal = ObjectRecognitionGoal()
        self._action_client.send_goal(goal, done_cb=self.on_action_result)

    def wait_for_detection(self):
        if self._action_client is not None:
            self._action_client.wait_for_result()

    def on_action_result(self, status, result):
        self.detected_object(result.recognized_objects.objects)
    
    def on_topic_data(self, msg):
        self.detected_object(msg.objects)

class ObjectBroadcaster:
    """ Given a detected object, request more information about it and publish the result as a CollisionObject """

    def __init__(self, topic = '/collision_object', diff_topic = '/recognized_object_diff'):
        self._publisher = rospy.Publisher(topic, CollisionObject)
        self._diff_publisher = rospy.Publisher(diff_topic, PlanningScene)
        self._index = 1
        self._min_confidence = 0.5
        self._lock = Lock()
        self._get_object_info = rospy.ServiceProxy('get_object_info', GetObjectInformation)

    def broadcast_diff(self, objects):
        planning_scene = PlanningScene()
        planning_scene.is_diff = True
        if isinstance(objects, collections.Iterable):
            for ob in objects:
                co = self._create_collision_object(ob)
                planning_scene.world.collision_objects.append(co)
        else:
            co = self._create_collision_object(objects)
            planning_scene.world.collision_objects.append(co)
        self._diff_publisher.publish(planning_scene)



    def _create_collision_object(self, ob):
        if ob.confidence < self._min_confidence:
            rospy.loginfo("Not publishing object of type %s because confidence %s < %s" % (ob.type.key, str(ob.confidence), str(self._min_confidence)))
            return

        info = None
        try:
            info = self._get_object_info(ob.type).information
        except rospy.ServiceException, e:
            rospy.logwarn("Unable to retrieve object information for object of type\n%s" % str(ob.type))

        co = CollisionObject()
        co.type = ob.type
        co.operation = CollisionObject.ADD
        co.header = ob.pose.header

        if info:
            if len(info.name) > 0:
                co.id = info.name + '_' + str(self._bump_index())
            else:
                co.id = ob.type.key + '_' + str(self._bump_index())
            if len(info.ground_truth_mesh.triangles) > 0:
                co.meshes = [info.ground_truth_mesh]
            else:
                co.meshes = [ob.bounding_mesh]
            co.mesh_poses = [ob.pose.pose.pose]            
        else:
            rospy.loginfo("Did not find information for object %s:" % (ob.type.key))
            co.id = ob.type.key + '_' + str(self._bump_index())
            co.meshes = [ob.bounding_mesh]
            co.mesh_poses = [ob.pose.pose.pose]
        if len(co.meshes[0].triangles) > 0:
            rospy.loginfo("Publishing collision object %s with confidence %s" % (co.id, str(ob.confidence)))

            # hack to turn the mesh into a box (aabb)
            #co.primitive_poses = co.mesh_poses
            #co.mesh_poses = []
            min_x = 1000000
            min_y = 1000000
            min_z = 1000000
            max_x = -1000000
            max_y = -1000000
            max_z = -1000000
            for v in co.meshes[0].vertices:
                if v.x > max_x:
                    max_x = v.x
                if v.y > max_y:
                    max_y = v.y
                if v.z > max_z:
                    max_z = v.z
                if v.x < min_x:
                    min_x = v.x
                if v.y < min_y:
                    min_y = v.y
                if v.z < min_z:
                    min_z = v.z

            box_co = copy.deepcopy(co)
            box_co.meshes[0].vertices = []
            box_co.meshes[0].triangles = []

            p = Point ()
            p.x = min_x;
            p.y = min_y;
            p.z = min_z;
            box_co.meshes[0].vertices.append(p);

            p = Point ()
            p.x = max_x;
            p.y = min_y;
            p.z = min_z;
            box_co.meshes[0].vertices.append(p);

            p = Point ()
            p.x = max_x;
            p.y = min_y;
            p.z = max_z;
            box_co.meshes[0].vertices.append(p);

            p = Point ()
            p.x = min_x;
            p.y = min_y;
            p.z = max_z;
            box_co.meshes[0].vertices.append(p);

            p = Point ()
            p.x = min_x;
            p.y = max_y;
            p.z = max_z;
            box_co.meshes[0].vertices.append(p);

            p = Point ()
            p.x = min_x;
            p.y = max_y;
            p.z = min_z;
            box_co.meshes[0].vertices.append(p);

            p = Point ()
            p.x = max_x;
            p.y = max_y;
            p.z = max_z;
            box_co.meshes[0].vertices.append(p);

            p = Point ()
            p.x = max_x;
            p.y = max_y;
            p.z = min_z;
            box_co.meshes[0].vertices.append(p);

            t = MeshTriangle ()
            t.vertex_indices [0] = 0
            t.vertex_indices [1] = 1
            t.vertex_indices [2] = 2
            box_co.meshes[0].triangles.append(t)

            t = MeshTriangle ()
            t.vertex_indices [0] = 2
            t.vertex_indices [1] = 3
            t.vertex_indices [2] = 0
            box_co.meshes[0].triangles.append(t)

            t = MeshTriangle ()
            t.vertex_indices [0] = 4
            t.vertex_indices [1] = 3
            t.vertex_indices [2] = 2
            box_co.meshes[0].triangles.append(t)

            t = MeshTriangle ()
            t.vertex_indices [0] = 2
            t.vertex_indices [1] = 6
            t.vertex_indices [2] = 4
            box_co.meshes[0].triangles.append(t)

            t = MeshTriangle ()
            t.vertex_indices [0] = 7
            t.vertex_indices [1] = 6
            t.vertex_indices [2] = 2
            box_co.meshes[0].triangles.append(t)

            t = MeshTriangle ()
            t.vertex_indices [0] = 2
            t.vertex_indices [1] = 1
            t.vertex_indices [2] = 7
            box_co.meshes[0].triangles.append(t)

            t = MeshTriangle ()
            t.vertex_indices [0] = 3
            t.vertex_indices [1] = 4
            t.vertex_indices [2] = 5
            box_co.meshes[0].triangles.append(t)

            t = MeshTriangle ()
            t.vertex_indices [0] = 5
            t.vertex_indices [1] = 0
            t.vertex_indices [2] = 3
            box_co.meshes[0].triangles.append(t)

            t = MeshTriangle ()
            t.vertex_indices [0] = 0
            t.vertex_indices [1] = 5
            t.vertex_indices [2] = 7
            box_co.meshes[0].triangles.append(t)

            t = MeshTriangle ()
            t.vertex_indices [0] = 7
            t.vertex_indices [1] = 1
            t.vertex_indices [2] = 0
            box_co.meshes[0].triangles.append(t)

            t = MeshTriangle ()
            t.vertex_indices [0] = 7
            t.vertex_indices [1] = 5
            t.vertex_indices [2] = 4
            box_co.meshes[0].triangles.append(t)

            t = MeshTriangle ()
            t.vertex_indices [0] = 4
            t.vertex_indices [1] = 6
            t.vertex_indices [2] = 7
            box_co.meshes[0].triangles.append(t)

            return box_co

    def broadcast_one(self, ob):
        if ob.confidence < self._min_confidence:
            rospy.loginfo("Not publishing object of type %s because confidence %s < %s" % (ob.type.key, str(ob.confidence), str(self._min_confidence)))
            return

        info = None
        try:
            info = self._get_object_info(ob.type).information
        except rospy.ServiceException, e:
            rospy.logwarn("Unable to retrieve object information for object of type\n%s" % str(ob.type))

        co = CollisionObject()
        co.type = ob.type
        co.operation = CollisionObject.ADD
        co.header = ob.pose.header

        if info:
            if len(info.name) > 0:
                co.id = info.name + '_' + str(self._bump_index())
            else:
                co.id = ob.type.key + '_' + str(self._bump_index())
            if len(info.ground_truth_mesh.triangles) > 0:
                co.meshes = [info.ground_truth_mesh]
            else:
                co.meshes = [ob.bounding_mesh]
            co.mesh_poses = [ob.pose.pose.pose]            
        else:
            rospy.loginfo("Did not find information for object %s:" % (ob.type.key))
            co.id = ob.type.key + '_' + str(self._bump_index())
            co.meshes = [ob.bounding_mesh]
            co.mesh_poses = [ob.pose.pose.pose]
        if len(co.meshes[0].triangles) > 0:
            rospy.loginfo("Publishing collision object %s with confidence %s" % (co.id, str(ob.confidence)))

            # hack to turn the mesh into a box (aabb)
            #co.primitive_poses = co.mesh_poses
            #co.mesh_poses = []
            min_x = 1000000
            min_y = 1000000
            min_z = 1000000
            max_x = -1000000
            max_y = -1000000
            max_z = -1000000
            for v in co.meshes[0].vertices:
                if v.x > max_x:
                    max_x = v.x
                if v.y > max_y:
                    max_y = v.y
                if v.z > max_z:
                    max_z = v.z
                if v.x < min_x:
                    min_x = v.x
                if v.y < min_y:
                    min_y = v.y
                if v.z < min_z:
                    min_z = v.z

            box_co = copy.deepcopy(co)
            box_co.meshes[0].vertices = []
            box_co.meshes[0].triangles = []

            p = Point ()
            p.x = min_x;
            p.y = min_y;
            p.z = min_z;
            box_co.meshes[0].vertices.append(p);

            p = Point ()
            p.x = max_x;
            p.y = min_y;
            p.z = min_z;
            box_co.meshes[0].vertices.append(p);

            p = Point ()
            p.x = max_x;
            p.y = min_y;
            p.z = max_z;
            box_co.meshes[0].vertices.append(p);

            p = Point ()
            p.x = min_x;
            p.y = min_y;
            p.z = max_z;
            box_co.meshes[0].vertices.append(p);

            p = Point ()
            p.x = min_x;
            p.y = max_y;
            p.z = max_z;
            box_co.meshes[0].vertices.append(p);

            p = Point ()
            p.x = min_x;
            p.y = max_y;
            p.z = min_z;
            box_co.meshes[0].vertices.append(p);

            p = Point ()
            p.x = max_x;
            p.y = max_y;
            p.z = max_z;
            box_co.meshes[0].vertices.append(p);

            p = Point ()
            p.x = max_x;
            p.y = max_y;
            p.z = min_z;
            box_co.meshes[0].vertices.append(p);

            t = MeshTriangle ()
            t.vertex_indices [0] = 0
            t.vertex_indices [1] = 1
            t.vertex_indices [2] = 2
            box_co.meshes[0].triangles.append(t)

            t = MeshTriangle ()
            t.vertex_indices [0] = 2
            t.vertex_indices [1] = 3
            t.vertex_indices [2] = 0
            box_co.meshes[0].triangles.append(t)

            t = MeshTriangle ()
            t.vertex_indices [0] = 4
            t.vertex_indices [1] = 3
            t.vertex_indices [2] = 2
            box_co.meshes[0].triangles.append(t)

            t = MeshTriangle ()
            t.vertex_indices [0] = 2
            t.vertex_indices [1] = 6
            t.vertex_indices [2] = 4
            box_co.meshes[0].triangles.append(t)

            t = MeshTriangle ()
            t.vertex_indices [0] = 7
            t.vertex_indices [1] = 6
            t.vertex_indices [2] = 2
            box_co.meshes[0].triangles.append(t)

            t = MeshTriangle ()
            t.vertex_indices [0] = 2
            t.vertex_indices [1] = 1
            t.vertex_indices [2] = 7
            box_co.meshes[0].triangles.append(t)

            t = MeshTriangle ()
            t.vertex_indices [0] = 3
            t.vertex_indices [1] = 4
            t.vertex_indices [2] = 5
            box_co.meshes[0].triangles.append(t)

            t = MeshTriangle ()
            t.vertex_indices [0] = 5
            t.vertex_indices [1] = 0
            t.vertex_indices [2] = 3
            box_co.meshes[0].triangles.append(t)

            t = MeshTriangle ()
            t.vertex_indices [0] = 0
            t.vertex_indices [1] = 5
            t.vertex_indices [2] = 7
            box_co.meshes[0].triangles.append(t)

            t = MeshTriangle ()
            t.vertex_indices [0] = 7
            t.vertex_indices [1] = 1
            t.vertex_indices [2] = 0
            box_co.meshes[0].triangles.append(t)

            t = MeshTriangle ()
            t.vertex_indices [0] = 7
            t.vertex_indices [1] = 5
            t.vertex_indices [2] = 4
            box_co.meshes[0].triangles.append(t)

            t = MeshTriangle ()
            t.vertex_indices [0] = 4
            t.vertex_indices [1] = 6
            t.vertex_indices [2] = 7
            box_co.meshes[0].triangles.append(t)

            self._publisher.publish(box_co)
        
    def broadcast(self, objects):
        self.broadcast_diff(objects)
#        if isinstance(objects, collections.Iterable):
#            self._reset_index()
#            for ob in objects:
#                self.broadcast_one(ob)
#        else:
#            self.broadcast_one(objects)

    def set_minimum_confidence(self, min_confidence):
        self._min_confidence = min_confidence

    def _bump_index(self):
        self._lock.acquire()
        index = self._index
        self._index = self._index + 1
        self._lock.release()
        return index

    def _reset_index(self):
        self._lock.acquire()
        self._index = 1
        self._lock.release()
