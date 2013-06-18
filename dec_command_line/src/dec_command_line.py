#!/usr/bin/env python
import sys
from time import time
import functools
import threading
import roslib; roslib.load_manifest('dec_command_line')
import rospy
from dec_msgs.msg import Object
import dec_msgs.msg
import roslib.packages
from dec_world_state.world_state import WorldState
from visualization_msgs.msg import Marker
from tf import transformations
import math
import tf
import shelve

import numpy

def main():
  rospy.init_node('DecCommandLine', anonymous=True)
  # object_poses = dict()

  # create publishers
  global mesh_pub
  mesh_pub = rospy.Publisher('/DECCommandLine/visualization_marker', Marker)

  global world_state
  world_state = WorldState()
  global object_poses
  object_poses = world_state.object_poses

  print 'Initialized dec_command_line.'

def euler_to_quat_deg(roll, pitch, yaw):
    roll = roll * (math.pi / 180.0)
    pitch = pitch * (math.pi / 180.0)
    yaw = yaw * (math.pi / 180.0)
    xaxis = (1, 0, 0)
    yaxis = (0, 1, 0)
    zaxis = (0, 0, 1)
    qx = transformations.quaternion_about_axis(roll, xaxis)
    qy = transformations.quaternion_about_axis(pitch, yaxis)
    qz = transformations.quaternion_about_axis(yaw, zaxis)
    q = transformations.quaternion_multiply(qx, qy)
    q = transformations.quaternion_multiply(q, qz)
    print q

""" get relative pose from object_a to object_b (object_a * rel_pose = object_b) """
def get_relative_pose(object_a, object_b):
    object_a_mat = get_object_pose_as_matrix(object_a)
    object_b_mat = get_object_pose_as_matrix(object_b)
    rel_mat = numpy.linalg.inv(object_a_mat).dot(object_b_mat)
    trans = transformations.translation_from_matrix(rel_mat)
    rot = transformations.quaternion_from_matrix(rel_mat)
    rot = [rot[3], rot[0], rot[1], rot[2]]
    print 'pose: [%f, %f, %f, %f, %f, %f, %f]' % (trans[0], trans[1], trans[2], rot[0], rot[1], rot[2], rot[3])
    return (trans, rot)

def set_object_from_tf(frame):
    (trans, rot) = get_tf(frame)
    set_object_pose(frame, trans[0], trans[1], trans[2], rot[3], rot[0], rot[1], rot[2])

""" get relative pose from frame1 to frame2 (frame_1 * rel_pose = frame2) """
def get_tf_to_tf(frame1, frame2):
    (trans1, rot1) = get_tf(frame1)
    (trans2, rot2) = get_tf(frame2)
    get_trans_to_trans(trans1, rot1, trans2, rot2)

def get_trans_to_tf(trans1, rot1, frame2):
    (trans2, rot2) = get_tf(frame2)
    get_trans_to_trans(trans1, rot1, trans2, rot2)

def convert_rel_pose_to_trans(rel_pose):
    trans = [rel_pose[0], rel_pose[1], rel_pose[2]]
    rot = [rel_pose[4], rel_pose[5], rel_pose[6], rel_pose[3]]
    return (trans, rot)

def convert_object_pose_to_trans(object_name):
    pose = object_poses[object_name].pose.pose.pose
    trans = [pose.position.x, pose.position.y, pose.position.z]
    rot = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
    return (trans, rot)

""" get relative pose from frame1 to frame2 (frame_1 * rel_pose = frame2) """
def get_trans_to_trans(trans1, rot1, trans2, rot2):
    quat_mat1 = transformations.quaternion_matrix(rot1)
    quat_mat2 = transformations.quaternion_matrix(rot2)
    trans_mat1 = transformations.translation_matrix(trans1)
    trans_mat2 = transformations.translation_matrix(trans2)
    mat1 = transformations.concatenate_matrices(trans_mat1, quat_mat1)
    mat2 = transformations.concatenate_matrices(trans_mat2, quat_mat2)
    rel_mat = numpy.linalg.inv(mat1).dot(mat2)
    rel_trans = transformations.translation_from_matrix(rel_mat)
    rel_rot = transformations.quaternion_from_matrix(rel_mat)
    rel_rot = [rel_rot[3], rel_rot[0], rel_rot[1], rel_rot[2]]
    print 'rel_pose: [%f, %f, %f, %f, %f, %f, %f]' % (rel_trans[0], rel_trans[1], rel_trans[2], rel_rot[0], rel_rot[1], rel_rot[2], rel_rot[3])

def get_object_pose_as_matrix(object_name):
    global object_poses
    obj = object_poses[object_name]
    obj_trans = (obj.pose.pose.pose.position.x,
                 obj.pose.pose.pose.position.y,
                 obj.pose.pose.pose.position.z)
    obj_quat = (obj.pose.pose.pose.orientation.x,
                obj.pose.pose.pose.orientation.y,
                obj.pose.pose.pose.orientation.z,
                obj.pose.pose.pose.orientation.w)
    obj_quat_mat = transformations.quaternion_matrix(obj_quat)
    obj_trans_mat = transformations.translation_matrix(obj_trans)
    obj_mat = transformations.concatenate_matrices(obj_trans_mat, obj_quat_mat)
    return obj_mat

def set_object_pose(object_name, x, y, z, qw, qx, qy, qz, real_object_name=''):
  if real_object_name == '':
    real_object_name = object_name
  global object_poses
  global world_state
  obj = Object()
  obj.name = real_object_name
  obj.pose.header.frame_id = '/BASE'
  obj.pose.pose.pose.position.x = x
  obj.pose.pose.pose.position.y = y
  obj.pose.pose.pose.position.z = z
  obj.pose.pose.pose.orientation.w = qw
  obj.pose.pose.pose.orientation.x = qx
  obj.pose.pose.pose.orientation.y = qy
  obj.pose.pose.pose.orientation.z = qz
  world_state.add_object(obj)
  # draw_object(object_name, obj)

def get_scene_filename(name):
    return roslib.packages.get_pkg_dir('dec_command_line') + '/scenes/' + name

def save_scene(name):
    global object_poses
    filename = get_scene_filename(name)
    d = shelve.open(filename)
    d['object_poses'] = object_poses
    d.close()

def load_scene(name):
    global object_poses
    global world_state
    filename = get_scene_filename(name)
    d = shelve.open(filename)
    my_object_poses = d['object_poses']
    d.close()
    for obj in my_object_poses.values():
        world_state.add_object(obj)
    # draw_all_objects()

def edit_object(object_name):
    global world_state
    world_state.edit_object(object_name)

def finish_edit_object(object_name):
    global world_state
    world_state.finish_edit_object(object_name)

if __name__ == "__main__":
  main()

def draw_object(object_name, object):
    marker = Marker()
    marker.header.frame_id = '/BASE'
    marker.header.stamp = rospy.Time.now()
    marker.ns = object_name
    marker.id = 0
    marker.type = Marker.MESH_RESOURCE
    marker.action = Marker.ADD
    marker.mesh_resource = 'package://dec_object_models/objects/%s/%s.obj' % (object_name, object.name)
    marker.pose = object.pose.pose.pose
    marker.scale.x, marker.scale.y, marker.scale.z = 1.0, 1.0, 1.0
    marker.color.r, marker.color.g, marker.color.b, marker.color.a = (1.0, 0.0, 0.0, 0.5)
    marker.lifetime = rospy.Duration()
    mesh_pub.publish(marker)

