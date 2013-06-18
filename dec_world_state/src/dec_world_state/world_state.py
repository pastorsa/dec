'''
Holds the world state and state of all objects

2013.01.08 Mrinal Kalakrishnan
'''

import rospy
from dec_world_state.msg import ObjectState
from dec_msgs.msg import Object

class WorldState:
    def __init__(self):
        self.object_sub = rospy.Subscriber('/world_state/objects', ObjectState, self.object_callback)
        self.object_pub = rospy.Publisher('/world_state/objects', ObjectState)
        self.object_poses = dict()

    def object_callback(self, object_state):
        #print 'callback for object ' + object_state.object.name
        if object_state.operation == object_state.ADD:
            self.object_poses[object_state.object.name] = object_state.object
        elif object_state.operation == object_state.REMOVE:
            del self.object_poses[object_state.object.name]

    def publish_object(self, object, operation):
        state = ObjectState()
        state.object = object
        state.operation = operation
        self.object_pub.publish(state)
        return True
        
    def add_object(self, object):
        self.publish_object(object, ObjectState.ADD)
        return True

    def set_object(self, object_name, object_pose = [0.0, 1.0, 0.8, 1.0, 0.0, 0.0, 0.0]):
        object = Object()
        object.name = object_name
        object.pose.header.stamp = rospy.Time.now()
        object.pose.header.frame_id = "/BASE"
        object.pose.pose.pose.position.x = object_pose[0]
        object.pose.pose.pose.position.y = object_pose[1]
        object.pose.pose.pose.position.z = object_pose[2]
        object.pose.pose.pose.orientation.w = object_pose[3]
        object.pose.pose.pose.orientation.x = object_pose[4]
        object.pose.pose.pose.orientation.y = object_pose[5]
        object.pose.pose.pose.orientation.z = object_pose[6]
        self.add_object(object)

    def remove_object(self, object_name):
        obj = Object()
        obj.name = object_name
        self.publish_object(obj, ObjectState.REMOVE)
        return True
    
    def remove_all_objects(self):
        for key in self.object_poses.keys():
            self.remove_object(key)
        return True
    