#!/usr/bin/env python

import rospy
import tf
from collections import defaultdict

from geometry_msgs.msg._Point import Point
from geometry_msgs.msg._Quaternion import Quaternion
from knowrob_beliefstate.srv._DirtyObject import DirtyObject, DirtyObjectResponse
from multiprocessing import Lock
from std_msgs.msg._ColorRGBA import ColorRGBA
from std_srvs.srv._Trigger import Trigger, TriggerResponse
from visualization_msgs.msg._Marker import Marker
from json_prolog import json_prolog


class PerceivedObject(object):
    def __init__(self):
        self.transform = None
        self.mesh_path = ''
        self.color = ColorRGBA()
        self.initialized = False

    def update_color(self, r, g, b, a):
        self.color = ColorRGBA()
        self.color.r = float(r)
        self.color.g = float(g)
        self.color.b = float(b)
        self.color.a = float(a)

    def update_transform(self, ref_frame, object_name, translation, rotation):
        self.ref_frame = str(ref_frame)
        self.object_name = str(object_name)
        self.transform = [self.ref_frame, self.object_name, translation, rotation]

    def get_marker(self):
        marker = Marker()
        marker.header.frame_id = self.ref_frame
        marker.type = marker.MESH_RESOURCE
        marker.action = Marker.ADD
        marker.id = 1337
        marker.ns = self.object_name
        marker.color = self.color

        marker.scale.x = 1
        marker.scale.y = 1
        marker.scale.z = 1
        marker.frame_locked = True
        marker.pose.position = Point(*self.transform[-2])
        marker.pose.orientation = Quaternion(*self.transform[-1])
        if len(self.mesh_path)>2 and self.mesh_path[0] == "'":
            marker.mesh_resource = self.mesh_path[1:-1]
        else:
            marker.mesh_resource = self.mesh_path
        return marker

    def get_del_marker(self):
        marker = Marker()
        marker.action = Marker.DELETE
        marker.id = 1337
        marker.ns = self.object_name
        return marker

class ObjectStatePublisher(object):
    def __init__(self, tf_frequency):
        rospy.wait_for_service('/json_prolog/query')
        self.tf_frequency = tf_frequency
        self.prolog = json_prolog.Prolog()
        self.tf_broadcaster = tf.TransformBroadcaster()
        self.marker_publisher = rospy.Publisher('/visualization_marker', Marker, queue_size=10)
        self.dirty_object_srv = rospy.Service('~mark_dirty_object', DirtyObject, self.dirty_cb)
        self.dirty_lock = Lock()
        self.update_positions_srv = rospy.Service('~update_object_positions', Trigger, self.update_object_positions_cb)
        self.objects = defaultdict(lambda: PerceivedObject())
        rospy.loginfo('object state publisher is running')

    def update_object_positions_cb(self, trigger):
        self.load_objects()
        r = TriggerResponse()
        r.success = True
        return r

    def dirty_cb(self, srv_msg):
        with self.dirty_lock:
            rospy.loginfo('got dirty object request {}'.format(srv_msg))
            r = DirtyObjectResponse()
            r.error_code = r.SUCCESS
            self.load_object_ids()
            for object_id in srv_msg.object_ids:
                if not self.load_object(object_id):
                    rospy.logdebug("object '{}' unknown".format(object_id))
                    r.error_code = r.UNKNOWN_OBJECT
                else:
                    rospy.loginfo("object '{}' updated".format(object_id))
            self.publish_object_frames()
            self.publish_object_markers()
            return r

    def prolog_query(self, q):
        query = self.prolog.query(q)
        solutions = [x for x in query.solutions()]
        if len(solutions) > 1:
            rospy.logwarn('{} returned more than one result'.format(q))
        elif len(solutions) == 0:
            rospy.logwarn('{} returned nothing'.format(q))
        query.finish()
        return solutions

    def load_objects(self):
        self.load_object_ids()
        for object_id in self.objects.keys():
            self.load_object(object_id)
        self.publish_object_frames()
        self.publish_object_markers()

    def load_object(self, object_id):
        if object_id in self.objects.keys():
            self.load_object_color(object_id)
            self.load_object_mesh(object_id)
            self.load_object_transform(object_id)
            self.objects[object_id].initialized = True
            self.objects[object_id].object_name = object_id
            return True
        rospy.logwarn("object with id:'{}' not found in database".format(object_id))
        return False

    def load_object_ids(self):
        q = 'belief_existing_objects(A)'
        solutions = self.prolog_query(q)
        for object_id in solutions[0]['A']:
            if object_id not in self.objects.keys():
                self.objects[object_id] = PerceivedObject()
        for object_id in self.objects.keys():
            if object_id not in solutions[0]['A']:
                self.marker_publisher.publish(self.objects[object_id].get_del_marker())
                self.objects.pop(object_id)
        rospy.loginfo('Loaded object ids: {}'.format([str(x) for x in self.objects.keys()]))

    def load_object_transform(self, object_id):
        q = "belief_at('{}', A)".format(object_id)
        solutions = self.prolog_query(q)
        if len(solutions) > 0:
            self.objects[object_id].update_transform(*solutions[0]['A'])
            rospy.logdebug("'{}' has transform: {}".format(object_id, self.objects[object_id].transform))
        else:
            self.objects[object_id].update_transform(object_id, 'map', [0,0,0],[0,0,0,1])
            rospy.logerr("'{}' has no active transform!".format(object_id))

    def load_object_color(self, object_id):
        q = "object_color('{}', A)".format(object_id)
        solutions = self.prolog_query(q)
        self.objects[object_id].update_color(*solutions[0]['A'])
        rospy.logdebug("'{}' has color: {}".format(object_id, self.objects[object_id].color))

    def load_object_mesh(self, object_id):
        q = "object_mesh_path('{}', A)".format(object_id)
        solutions = self.prolog_query(q)
        if len(solutions) > 0:
            self.objects[object_id].mesh_path = str(solutions[0]['A'])
            rospy.logdebug("'{}' has mesh path: {}".format(object_id, self.objects[object_id].mesh_path))

    def publish_object_markers(self):
        r = rospy.Rate(10)
        for object_id, v in self.objects.items():
            if v.initialized:
                self.marker_publisher.publish(v.get_marker())
                r.sleep()

    def publish_object_frames(self):
        for object_id, perceived_object in self.objects.items():
            if perceived_object.initialized:
                ref_frame, object_frame, translation, rotation = perceived_object.transform
                self.tf_broadcaster.sendTransform(translation,
                                                  rotation,
                                                  rospy.Time.now(),
                                                  object_frame,
                                                  ref_frame)

    def loop(self):
        self.load_objects()
        rate = rospy.Rate(self.tf_frequency)
        while not rospy.is_shutdown():
            self.publish_object_frames()
            rate.sleep()


if __name__ == '__main__':
    rospy.init_node('object_state_publisher')
    hz = rospy.get_param('~hz', default='1')
    object_state_publisher = ObjectStatePublisher(int(hz))
    object_state_publisher.loop()
