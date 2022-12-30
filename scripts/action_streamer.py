#!/usr/bin/env python

# A simple script to tests sending Event tokens
# for the activity parser and its GUI.

import sys
import rospy
import time
#import rosprolog_client as rosprolog

from knowrob.msg._EventToken import EventToken

class ActionStreamer(object):
    def __init__(self):
        self.token_publisher = rospy.Publisher('/parser/token', EventToken, queue_size=1)
        # HACK: need to wait until publisher is running
        rospy.sleep(0.5)
        rospy.loginfo('token streamer is running')

    def support(self,obj,location):
        self.begin_('Supporting',[location,obj])

    def pick(self,obj,using,location):
        self.begin_('GraspMotion',[using])
        time.sleep(0.2)
        self.begin_('Touching',[using,obj])
        time.sleep(0.1)
        self.end_('GraspMotion',[using])
        time.sleep(0.4)
        self.end_('Supporting',[location,obj])

    def place(self,obj,using,location):
        self.begin_('Supporting',[location,obj])
        time.sleep(0.2)
        self.begin_('ReleaseMotion',[using])
        time.sleep(0.1)
        self.end_('Touching',[using,obj])
        time.sleep(0.4)
        self.end_('ReleaseMotion',[using])

    def get_iri_(self,x):
        # TODO rather use concepts from EASE ontology
        return 'http://knowrob.org/kb/parser-test.owl#'+x

    def make_token_(self,tok_polarization,tok_type,tok_objs):
        return EventToken(
            timestamp=time.time() * 1000.0,
            polarization=tok_polarization,
            event_type=self.get_iri_(tok_type),
            participants=list(map(self.get_iri_,tok_objs))
        )

    def begin_(self,tok_type,tok_objs):
        self.send_token_(self.make_token_(EventToken.EVENT_BEGIN,tok_type,tok_objs))

    def end_(self,tok_type,tok_objs):
        self.send_token_(self.make_token_(EventToken.EVENT_END,tok_type,tok_objs))

    def send_token_(self,tok):
        self.token_publisher.publish(tok)


if __name__ == '__main__':
    rospy.init_node('parser_stream_test')
    streamer = ActionStreamer()
    [method_name,rest] = sys.argv[1].split('(')
    method_args = dict(map(lambda x: x.split("="),rest[:-1].split(',')))
    getattr(streamer, method_name)(**method_args)
