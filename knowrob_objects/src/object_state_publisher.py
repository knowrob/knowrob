#!/usr/bin/env python

import rospy
import tf
from collections import defaultdict

from geometry_msgs.msg import Vector3
from geometry_msgs.msg._Point import Point
from geometry_msgs.msg._Quaternion import Quaternion
from knowrob_objects.srv._DirtyObject import DirtyObject, DirtyObjectResponse
from multiprocessing import Queue
from std_msgs.msg._ColorRGBA import ColorRGBA
from std_srvs.srv._Trigger import Trigger, TriggerResponse
from visualization_msgs.msg import MarkerArray
from visualization_msgs.msg._Marker import Marker
from json_prolog import json_prolog


class PerceivedObject(object):
    def __init__(self):
        self.transform = None
        self.mesh_path = ''
        self.color = ColorRGBA(0,0,0,1)
        self.initialized = False
        self.visualize = False
        self.scale = Vector3(0.05, 0.05, 0.05)

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

    def update_dimensions(self, depth, width, height):
        self.scale = Vector3(width, depth, height)

    def get_marker(self):
        marker = Marker()
        marker.header.frame_id = self.ref_frame
        marker.action = Marker.ADD
        marker.id = 1337
        marker.ns = self.object_name
        marker.color = self.color

        marker.scale = Vector3(1,1,1) # don't use self.scale because the meshes are already scaled.
        marker.frame_locked = True
        marker.pose.position = Point(*self.transform[-2])
        marker.pose.orientation = Quaternion(*self.transform[-1])
        if self.mesh_path != '':
            marker.type = marker.MESH_RESOURCE
            marker.mesh_resource = self.mesh_path.replace('\'', '')
        else:
            rospy.logdebug('{} has no mesh'.format(self.object_name))
            marker.type = Marker.CUBE
            marker.scale = self.scale
        return marker

    def get_del_marker(self):
        marker = Marker()
        marker.action = Marker.DELETE
        marker.id = 1337
        marker.ns = self.object_name
        return marker

class ObjectStatePublisher(object):
    def __init__(self, tf_frequency, object_types):
        rospy.wait_for_service('/json_prolog/query')
        self.tf_frequency = tf_frequency
        self.object_types = object_types
        self.prolog = json_prolog.Prolog()
        self.tf_broadcaster = tf.TransformBroadcaster()
        self.marker_publisher = rospy.Publisher('/visualization_marker', Marker, queue_size=10)
        self.marker_array_publisher = rospy.Publisher('/visualization_marker_array', MarkerArray, queue_size=10)
        self.dirty_object_srv = rospy.Service('~mark_dirty_object', DirtyObject, self.dirty_cb)
        self.update_positions_srv = rospy.Service('~update_object_positions', Trigger, self.update_object_positions_cb)
        self.objects = defaultdict(lambda: PerceivedObject())
        self.dirty_timer = rospy.Timer(rospy.Duration(.01), self.dirty_timer_cb)
        self.dirty_object_requests = Queue()
        rospy.loginfo('object state publisher is running')

    def update_object_positions_cb(self, trigger):
        self.load_objects()
        r = TriggerResponse()
        r.success = True
        return r

    def dirty_timer_cb(self, _):
        # This is done in a different thread to prevent deadlocks in json prolog
        srv_msg = self.dirty_object_requests.get()
        rospy.logdebug('got dirty object request {}'.format(srv_msg))
        self.load_object_ids()
        for object_id in srv_msg.object_ids:
            if not self.load_object(object_id):
                rospy.logdebug("object '{}' unknown".format(object_id))
            else:
                rospy.logdebug("object '{}' updated".format(object_id))
        self.publish_object_frames()
        self.publish_object_markers()

    def dirty_cb(self, srv_msg):
        self.dirty_object_requests.put(srv_msg)
        r = DirtyObjectResponse()
        r.error_code = r.SUCCESS
        return r

    def prolog_query(self, q, silent=False):
        query = self.prolog.query(q)
        solutions = [x for x in query.solutions()]
        if not silent and len(solutions) > 1:
            rospy.logwarn('{} returned more than one result'.format(q))
        elif not silent and len(solutions) == 0:
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
            self.load_object_transform(object_id)
            self.objects[object_id].visualize = self.object_has_visual(object_id)
            if self.objects[object_id].visualize:
              self.load_object_color(object_id)
              self.load_object_mesh(object_id)
              self.load_object_dimensions(object_id)
            self.objects[object_id].initialized = True
            self.objects[object_id].object_name = object_id
            return True
        rospy.logwarn("object with id:'{}' not found in database".format(object_id))
        return False

    def object_has_visual(self, object_id):
        q = "not(rdf_has('{}', knowrob:'hasVisual', literal(type(_,false))))".format(object_id)
        solutions = self.prolog_query(q,silent=True)
        if len(solutions) > 0: return True
        else:                  return False

    def load_object_ids(self):
        q = 'belief_existing_objects(A,['+','.join(self.object_types)+'])'
        solutions = self.prolog_query(q)
        for object_id in solutions[0]['A']:
            if object_id not in self.objects.keys():
                self.objects[object_id] = PerceivedObject()
        for object_id in self.objects.keys():
            if object_id not in solutions[0]['A']:
                self.marker_publisher.publish(self.objects[object_id].get_del_marker())
                self.objects.pop(object_id)
        rospy.logdebug('Loaded object ids: {}'.format([str(x) for x in self.objects.keys()]))

    def load_object_transform(self, object_id):
        q = "belief_at_id('{}', A)".format(object_id)
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
        q = "object_mesh_path('{}', A),!".format(object_id)
        solutions = self.prolog_query(q)
        if len(solutions) > 0:
            self.objects[object_id].mesh_path = str(solutions[0]['A'])
            rospy.logdebug("'{}' has mesh path: {}".format(object_id, self.objects[object_id].mesh_path))

    def load_object_dimensions(self, object_id):
        q = "object_dimensions('{}', D, W, H)".format(object_id)
        solutions = self.prolog_query(q)
        if len(solutions) > 0:
            self.objects[object_id].update_dimensions(depth=solutions[0]['D'],
                                                      width=solutions[0]['W'],
                                                      height=solutions[0]['H'])

    def publish_object_markers(self):
        ma = MarkerArray()
        for object_id, v in self.objects.items():
            if v.initialized and v.visualize:
                ma.markers.append(v.get_marker())
        self.marker_array_publisher.publish(ma)

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
    object_types = rospy.get_param('~object_types', default="knowrob:'EnduringThing-Localized'")
    object_state_publisher = ObjectStatePublisher(int(hz), object_types.split(','))
    object_state_publisher.loop()