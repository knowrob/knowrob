#!/usr/bin/env python

import rospy
from collections import defaultdict

from geometry_msgs.msg import Vector3, Transform, TransformStamped
from geometry_msgs.msg._Point import Point
from geometry_msgs.msg._Quaternion import Quaternion
#from knowrob_objects.srv._DirtyObject import DirtyObject, DirtyObjectResponse
from knowrob_objects.srv._UpdateObjectState import UpdateObjectState, UpdateObjectStateResponse
from knowrob_objects.msg._ObjectState import ObjectState
from multiprocessing import Queue
from std_msgs.msg._ColorRGBA import ColorRGBA
from std_srvs.srv._Trigger import Trigger, TriggerResponse
from tf2_msgs.msg import TFMessage
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from visualization_msgs.msg import MarkerArray
from visualization_msgs.msg._Marker import Marker
from json_prolog import json_prolog
from random import random

class PerceivedObject(object):
    def __init__(self):
        self.transform = None
        self.mesh_path = ''
        self.object_name = ''
        self.ref_frame = ''
        self.color = ColorRGBA(0, 0, 0, 1)
        self.initialized = False
        self.visualize = False
        self.scale = Vector3(0.05, 0.05, 0.05)
        self.static_transforms = dict()
        self.static_transforms_loaded = False
        self.marker_id = int(random()*1000000000)
        self.marker_ns = 'belief_state'
        self.marker = Marker()
        self.marker.id = self.marker_id
        self.marker.ns = self.marker_ns
        self.marker.action = Marker.ADD
        self.marker.scale = Vector3(1, 1, 1) # don't use self.scale because the meshes are already scaled.
        # TODO: do we need to republish frame locked markers at all?
        #       maybe only facings because they may change visual appearance
        self.marker.frame_locked = True

    def update_color(self, r, g, b, a):
        self.color = ColorRGBA()
        self.color.r = float(r)
        self.color.g = float(g)
        self.color.b = float(b)
        self.color.a = float(a)

    def set_static_transform(self, ref_frame, affordance_name, translation, rotation):
        self.static_transforms[affordance_name] = [ref_frame, affordance_name, translation, rotation]

    def update_transform(self, ref_frame, object_name, translation, rotation):
        self.ref_frame = str(ref_frame)
        self.object_name = str(object_name)
        self.transform = [self.ref_frame, self.object_name, translation, rotation]
        self.marker.header.frame_id = self.ref_frame
        self.marker.pose.position = Point(*self.transform[-2])
        self.marker.pose.orientation = Quaternion(*self.transform[-1])

    def update_dimensions(self, depth, width, height):
        self.scale = Vector3(width, depth, height)

    def get_marker(self):
        if self.mesh_path != '' and self.mesh_path != "''":
            self.marker.type = self.marker.MESH_RESOURCE
            self.marker.mesh_resource = self.mesh_path.replace('\'', '')
            self.marker.mesh_use_embedded_materials = True
            if self.marker.mesh_resource.endswith('.stl'):
                # TODO I think we need this special case for stl meshes, try to remove this line if they are buggy
                self.marker.color = self.color
        else:
            self.marker.color = self.color
            self.marker.type = Marker.CUBE
            self.marker.scale = self.scale
        return self.marker

    def get_del_marker(self):
        marker = Marker()
        marker.action = Marker.DELETE
        marker.id = self.marker_id
        marker.ns = self.marker_ns
        return marker

    def __repr__(self):
        return 'obj('+str(self.visualize)+','+\
                      str(self.object_name)+','+\
                      str(self.mesh_path)+','+\
                      str([self.scale.x,self.scale.y,self.scale.z])+','+\
                      str([self.color.r,self.color.g,self.color.b,self.color.a])+')'


class ObjectStatePublisher(object):
    def __init__(self, tf_frequency, object_types):
        rospy.wait_for_service('/json_prolog/query')
        self.tf_frequency = tf_frequency
        self.object_types = object_types
        self.dirty_object_requests = Queue()
        self.prolog = json_prolog.Prolog()
        self.objects = defaultdict(lambda: PerceivedObject())
        self.tf_broadcaster = rospy.Publisher("/tf", TFMessage, queue_size=100)
        self.tf_static = rospy.Publisher("/tf_static", TFMessage, queue_size=100, latch=True)
        self.marker_publisher = rospy.Publisher('/visualization_marker', Marker, queue_size=100)
        self.marker_array_publisher = rospy.Publisher('/visualization_marker_array', MarkerArray, queue_size=100)
        #self.dirty_object_srv = rospy.Service('~mark_dirty_object', DirtyObject, self.dirty_cb)
        self.dirty_object_srv = rospy.Service('~update_object_states', UpdateObjectState, self.update_state_cb)
        self.update_positions_srv = rospy.Service('~update_object_positions', Trigger, self.update_object_positions_cb)
        self.dirty_timer = rospy.Timer(rospy.Duration(.01), self.dirty_timer_cb)
        rospy.loginfo('object state publisher is running')
        rospy.sleep(1)

    def update_object_positions_cb(self, trigger):
        self.load_objects()
        r = TriggerResponse()
        r.success = True
        return r

    def update_state_cb(self, srv_msg):
        self.dirty_object_requests.put(srv_msg)
        r = UpdateObjectStateResponse()
        r.error_code = r.SUCCESS
        return r

    def dirty_timer_cb(self, _):
        # This is done in a different thread to prevent deadlocks in json prolog
        if self.dirty_object_requests.empty():
            return True
        object_states = list()
        while not self.dirty_object_requests.empty():
            srv_msg = self.dirty_object_requests.get()
            object_states += list(srv_msg.object_states)
        self.load_objects(object_states)

    #def dirty_timer_cb(self, _):
        ## This is done in a different thread to prevent deadlocks in json prolog
        #if self.dirty_object_requests.empty():
            #return True
        #object_ids = set()
        #while not self.dirty_object_requests.empty():
            #srv_msg = self.dirty_object_requests.get()
            #object_ids |= set(srv_msg.object_ids)
        #self.load_objects(object_ids)

    #def dirty_cb(self, srv_msg):
    #    self.dirty_object_requests.put(srv_msg)
    #    r = DirtyObjectResponse()
    #    r.error_code = r.SUCCESS
    #    return r

    def prolog_query(self, q, verbose=False):
        query = self.prolog.query(q)
        solutions = [x for x in query.solutions()]
        if verbose:
            if len(solutions) > 1:
                rospy.logwarn('{} returned more than one result'.format(q))
            elif len(solutions) == 0:
                rospy.logwarn('{} returned nothing'.format(q))
        query.finish()
        return solutions

    def load_objects(self, object_ids=None):
        #self.load_object_ids()
        if object_ids is None:
            self.load_object_ids()
            object_ids = self.objects.keys()
        self.load_object_information(object_ids)
        # TODO: do we need to do this here? loop() is doing it anyway...
        self.publish_object_frames()
        self.publish_object_markers(object_ids)
        # TODO: I guess these don't need to be republished once published?
        self.publish_static_transforms()

    def load_object_ids(self):
        # TODO: replace with service calls telling the publisher that objects were created/destroyed.
        #       then this lookup has only to be done once initially
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

    def load_object_information(self, objects):
        if len(objects)>0 and isinstance(objects[0],ObjectState):
            self.load_object_state(objects)
        else:
            self.query_object_information(objects)

    def load_object_state(self, object_states):
        rospy.loginfo('Updated object state')
        for obj_state in object_states:
            if obj_state.object_id not in self.objects.keys():
                self.objects[obj_state.object_id] = PerceivedObject()
            obj = self.objects[obj_state.object_id]
            obj.marker_ns = (str(obj_state.object_type).replace('\'', ''))
            obj.visualize = bool(obj_state.has_visual)
            ###
            position = obj_state.pose.pose.position
            orientation = obj_state.pose.pose.orientation
            obj.update_transform(
                obj_state.pose.header.frame_id,
                obj_state.frame_name,
                [position.x,position.y,position.z],
                [orientation.x,orientation.y,orientation.z,orientation.w])
            #for static_transform in x['StaticTransforms']:
            #    obj.set_static_transform(*static_transform)
            if obj.visualize:
                obj.update_color(obj_state.color.r,obj_state.color.g,obj_state.color.b,obj_state.color.a)
                obj.mesh_path = obj_state.mesh_path
                obj.update_dimensions(obj_state.size.x,obj_state.size.y,obj_state.size.z)
            obj.initialized = True
            obj.object_name = obj_state.object_id
            rospy.logdebug('Updated object: {}'.format(str(obj)))

    def query_object_information(self, object_ids):
        rospy.loginfo('Query object information')
        q = "member(Obj,['"+"','".join(object_ids)+"']),object_information(Obj,Type,HasVisual,Color,Mesh,[D,W,H],Pose,StaticTransforms)"
        #solutions = self.prolog_query(q, verbose=False)
        for x in self.prolog.query(q).solutions():
            object_id = str(x['Obj']).replace('\'', '')
            obj = self.objects[object_id]
            obj.marker_ns = (str(x['Type']).replace('\'', ''))
            obj.visualize = (str(x['HasVisual'])=="'true'")
            obj.update_transform(*x['Pose'])
            for static_transform in x['StaticTransforms']:
                obj.set_static_transform(*static_transform)
            if obj.visualize:
                obj.update_color(*x['Color'])
                obj.mesh_path = str(x['Mesh'])
                obj.update_dimensions(depth=x['D'],width=x['W'],height=x['H'])
            obj.initialized = True
            obj.object_name = object_id
            rospy.logdebug('Updated object: {}'.format(str(obj)))

    def publish_object_markers(self, objects):
        ma = MarkerArray()
        for o in objects:
            if isinstance(o,ObjectState):
                object_id = o.object_id
            else:
                object_id = o
            v = self.objects[object_id]
            if v.initialized and v.visualize:
                ma.markers.append(v.get_marker())
        self.marker_array_publisher.publish(ma)

    def publish_object_frames(self):
        tf_msgs = []
        stamp = rospy.get_rostime()
        for object_id, perceived_object in self.objects.items():
            if perceived_object.initialized:
                ref_frame, object_frame, translation, rotation = perceived_object.transform
                msg = TransformStamped()
                msg.header.stamp = stamp
                msg.header.frame_id = ref_frame
                msg.child_frame_id = object_frame
                msg.transform.translation = Vector3(*translation)
                msg.transform.rotation = Quaternion(*rotation)
                tf_msgs.append(msg)
        self.tf_broadcaster.publish(TFMessage(tf_msgs))

    def publish_static_transforms(self):
        # we have to publish all tf static msg in one msg otherwise shit doesn't work for some reason
        tf_msgs = []
        stamp = rospy.get_rostime()
        for obj in self.objects.values():
            for [ref_frame, object_frame, translation, rotation] in obj.static_transforms.values():
                msg = TransformStamped()
                msg.header.stamp = stamp
                msg.header.frame_id = ref_frame
                msg.child_frame_id = object_frame
                msg.transform.translation = Vector3(*translation)
                msg.transform.rotation = Quaternion(*rotation)
                tf_msgs.append(msg)
        if len(tf_msgs)>0:
            self.tf_static.publish(TFMessage(tf_msgs))

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
