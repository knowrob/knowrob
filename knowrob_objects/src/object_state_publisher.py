#!/usr/bin/env python

import rospy
from collections import defaultdict

from geometry_msgs.msg import Vector3, Transform, TransformStamped
from geometry_msgs.msg._Point import Point
from geometry_msgs.msg._Quaternion import Quaternion
from knowrob_objects.msg._ObjectState import ObjectState
from knowrob_objects.msg._ObjectStateArray import ObjectStateArray
from multiprocessing import Queue
from std_msgs.msg._ColorRGBA import ColorRGBA
from std_srvs.srv._Trigger import Trigger, TriggerResponse
from tf2_msgs.msg import TFMessage
from tf2_ros import StaticTransformBroadcaster
from visualization_msgs.msg import MarkerArray
from visualization_msgs.msg._Marker import Marker
from json_prolog import json_prolog
from random import random

# fix to allow multiple latched publisher on the same topic
class LatchPublisher(rospy.Publisher, rospy.SubscribeListener):
    def __init__(self, name, data_class, tcp_nodelay=False, headers=None, queue_size=None):
        super(LatchPublisher, self).__init__(name, data_class=data_class, tcp_nodelay=tcp_nodelay, headers=headers, queue_size=queue_size, subscriber_listener=self, latch=False)
        self.message = None

    def publish(self, msg):
        self.message = msg
        super(LatchPublisher, self).publish(msg)

    def peer_subscribe(self, resolved_name, publish, publish_single):
        if self.message is not None:
            publish_single(self.message)

def transform_to_prolog(transform_stamped):
    """
    :type transform_stamped: TransformStamped
    :rtype: list
    """
    return [transform_stamped.header.frame_id,
            transform_stamped.child_frame_id,
            [transform_stamped.transform.translation.x,
             transform_stamped.transform.translation.y,
             transform_stamped.transform.translation.z],
            [transform_stamped.transform.rotation.x,
             transform_stamped.transform.rotation.y,
             transform_stamped.transform.rotation.z,
             transform_stamped.transform.rotation.w]]


def prolog_to_transform(frame_id, child_frame_id, translation, rotation):
    """
    :type frame_id: str
    :type child_frame_id: str
    :type translation: list
    :type rotation: list
    :rtype: TransformStamped
    """
    if child_frame_id==None or translation==None or rotation==None: return None
    t = TransformStamped()
    t.header.frame_id = frame_id.encode('utf-8')
    t.child_frame_id = child_frame_id.encode('utf-8')
    t.transform.translation = Point(*translation)
    t.transform.rotation = Quaternion(*rotation)
    return t


class PerceivedObject(object):
    __transform = None
    __mesh_path = ''
    __object_name = ''
    __scale = None
    __color = None
    __initialized = False
    __has_visual = False
    __marker = None

    def __init__(self):
        self.__static_transforms = {}
        self.__static_transform_publisher = LatchPublisher('/tf_static', TFMessage, queue_size=500)

    # @profile
    def update_information(self, object_id, marker_ns, visualize, transform, static_transforms, mesh, color,
                           depth, width, height):
        """
        :type marker_ns: str
        :type visualize: bool
        :type transform: TransformStamped
        :param static_transforms: [TransformStamped]
        :type static_transforms: list
        :param color: [r, g, b, a]
        :type color: list
        :type mesh: str
        :type depth: float
        :type width: float
        :type height: float
        """
        self.__marker_ns = marker_ns
        self.__has_visual = visualize
        self.update_transform(transform)
        for static_transform in static_transforms:
            self.set_static_transform(static_transform)
        if visualize:
            self.update_color(*color)
            self.__mesh_path = mesh
            self.update_dimensions(depth, width, height)
        else:
            self.__color = ColorRGBA(0, 0, 0, 1)
            self.update_dimensions(1, 1, 1)
        self.__initialized = True
        self.__object_name = object_id
        # self.publish_static_transforms()

    def is_initialized(self):
        return self.__initialized

    def has_visual(self):
        return self.__has_visual

    def update_color(self, r, g, b, a):
        self.__color = ColorRGBA()
        self.__color.r = r
        self.__color.g = g
        self.__color.b = b
        self.__color.a = a

    def set_static_transform(self, static_transform):
        """
        :type static_transform: TransformStamped
        """
        self.__static_transforms[static_transform.child_frame_id] = static_transform

    def update_transform(self, transform):
        """
        :type transform: TransformStamped
        """
        self.__transform = transform

    def update_dimensions(self, depth, width, height):
        self.__scale = Vector3(width, depth, height)


    def get_marker(self):
        if self.is_initialized():
            if self.__marker is None:
                self.__marker = Marker()
                self.__marker.id = int(random() * 1000000000) # FIXME find a better way to generate ids
                self.__marker.ns = 'belief_state'
                self.__marker.action = Marker.ADD
                self.__marker.frame_locked = True
            self.__marker.header.frame_id = self.__transform.header.frame_id
            self.__marker.pose.position = self.__transform.transform.translation
            self.__marker.pose.orientation = self.__transform.transform.rotation

            if self.__mesh_path != '' and self.__mesh_path != "''":
                self.__marker.type = self.__marker.MESH_RESOURCE
                self.__marker.mesh_resource = self.__mesh_path.replace('\'', '')
                self.__marker.mesh_use_embedded_materials = True
                self.__marker.scale = Vector3(1, 1, 1)
                if self.__marker.mesh_resource.endswith('.stl'):
                    # TODO I think we need this special case for stl meshes, try to remove this line if they are buggy
                    self.__marker.color = self.__color
            else:
                self.__marker.color = self.__color
                self.__marker.type = Marker.CUBE
                self.__marker.scale = self.__scale
            return self.__marker

    def get_del_marker(self):
        marker = Marker()
        marker.action = Marker.DELETE
        marker.id = self.__marker.id
        marker.ns = self.__marker.ns
        return marker

    def get_tf_msg(self):
        if self.is_initialized():
            return self.__transform

    def publish_static_transforms(self):
        if len(self.__static_transforms) > 0:
            self.__static_transform_publisher.publish(TFMessage(self.__static_transforms.values()))

    def __repr__(self):
        return 'obj(' + str(self.__has_visual) + ',' + \
               str(self.__object_name) + ',' + \
               str(self.__mesh_path) + ',' + \
               str([self.__scale.x, self.__scale.y, self.__scale.z]) + ',' + \
               str([self.__color.r, self.__color.g, self.__color.b, self.__color.a]) + ')'


class ObjectStatePublisher(object):
    def __init__(self, tf_frequency, object_types):
        rospy.wait_for_service('/json_prolog/query')
        self.tf_frequency = tf_frequency
        self.object_types = object_types
        self.dirty_object_requests = Queue()
        self.prolog = json_prolog.Prolog()
        self.objects = defaultdict(lambda: PerceivedObject())
        self.tf_broadcaster = rospy.Publisher("/tf", TFMessage, queue_size=100)
        self.marker_publisher = rospy.Publisher('/visualization_marker', Marker, queue_size=100)
        self.marker_array_publisher = rospy.Publisher('/visualization_marker_array', MarkerArray, queue_size=100)
        self.object_state_subscriber = rospy.Subscriber('/object_state', ObjectStateArray, self.update_state_cb)
        self.update_positions_srv = rospy.Service('~update_object_positions', Trigger, self.update_object_positions_cb)
        self.dirty_timer = rospy.Timer(rospy.Duration(.01), self.dirty_timer_cb)
        rospy.loginfo('object state publisher is running')
        rospy.sleep(1)

    def update_object_positions_cb(self, trigger):
        self.load_objects_from_prolog()
        r = TriggerResponse()
        r.success = True
        return r

    def update_state_cb(self, srv_msg):
        self.dirty_object_requests.put(srv_msg)

    def dirty_timer_cb(self, _):
        # This is done in a different thread to prevent deadlocks in json prolog
        if self.dirty_object_requests.empty():
            return True
        object_states = []
        while not self.dirty_object_requests.empty():
            srv_msg = self.dirty_object_requests.get()
            object_states.extend(srv_msg.object_states)
        self.update_objects(object_states)

    def prolog_query(self, q):
        return self.prolog.all_solutions(q)

    # @profile
    def update_objects(self, object_states):
        object_ids = []
        for obj_state in object_states:
            object_ids.append(obj_state.object_id)
            marker_ns = (str(obj_state.object_type).replace('\'', ''))
            transform = TransformStamped()
            transform.header.frame_id = obj_state.pose.header.frame_id
            transform.child_frame_id = obj_state.frame_name
            transform.transform.translation = obj_state.pose.pose.position
            transform.transform.rotation = obj_state.pose.pose.orientation

            color = [obj_state.color.r, obj_state.color.g, obj_state.color.b, obj_state.color.a]
            self.objects[obj_state.object_id].update_information(obj_state.object_id,
                                                                 marker_ns,
                                                                 obj_state.has_visual,
                                                                 transform,
                                                                 obj_state.static_transforms,
                                                                 obj_state.mesh_path,
                                                                 color,
                                                                 obj_state.size.x, obj_state.size.y, obj_state.size.z)
            rospy.logdebug('Updated object: {}'.format(str(self.objects[obj_state.object_id])))
        self.publish_object_markers(object_ids)
        self.publish_static_transforms(object_ids)

    # @profile
    def load_objects_from_prolog(self):
        q = "belief_existing_objects(Objects,[{}])," \
            "belief_republish_objects(Objects)".format(','.join(self.object_types))
        self.prolog_query(q)

    def publish_object_markers(self, object_ids):
        ma = MarkerArray()
        for object_id in object_ids:
            v = self.objects[object_id]
            if v.is_initialized() and v.has_visual():
                ma.markers.append(v.get_marker())
        self.marker_array_publisher.publish(ma)

    # @profile
    def publish_object_frames(self, object_ids=None):
        if object_ids is None:
            object_ids = self.objects.keys()
        tf_msgs = []
        stamp = rospy.get_rostime()
        for object_id in object_ids:  # type: (str, PerceivedObject)
            tf_msg = self.objects[object_id].get_tf_msg()
            tf_msg.header.stamp = stamp
            if tf_msg is not None:
                tf_msgs.append(tf_msg)
        self.tf_broadcaster.publish(TFMessage(tf_msgs))

    def publish_static_transforms(self, object_ids):
        for object_id in object_ids:
            self.objects[object_id].publish_static_transforms()

    def loop(self):
        self.load_objects_from_prolog()
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
