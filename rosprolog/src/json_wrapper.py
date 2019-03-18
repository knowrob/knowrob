#!/usr/bin/env python

import rospy
import json
import importlib

from rosprolog.srv._JSONWrapper import JSONWrapper, JSONWrapperResponse

# global handle to all imported ROS modules
ros_modules = {}
ros_services = {}

# dynamic import of ROS modules by type string, e.g. 'std_srvs/Trigger'
def get_ros_module(type_string, import_path):
    try:
        module = ros_modules[type_string]
    except:
        module = importlib.import_module(import_path)
        ros_modules[type_string] = module
    return module

def get_service_module(type_string):
    x = type_string.split('/')
    return get_ros_module(type_string, '.'.join([x[0],'srv','_'+x[1]]))

def get_message_module(type_string):
    x = type_string.split('/')
    return get_ros_module(type_string, '.'.join([x[0],'msg','_'+x[1]]))
    
def get_service(srv_name, srv_type):
    try:
        srv = ros_services[srv_name]
    except:
        srv = rospy.ServiceProxy(srv_name, srv_type)
        ros_services[srv_name] = srv
    return srv

class JSONWrapperService(object):
    def __init__(self):
        self.update_positions_srv = rospy.Service('~json_wrapper', JSONWrapper, self.json_wrapper_cb)
        rospy.loginfo('json_wrapper service is running')
    
    def json_wrapper_cb(self, wrapper_request):
        # TODO: also include a status field in response
        rospy.logdebug('received: ' + wrapper_request.json_data)
        request_data = json.loads(wrapper_request.json_data)
        srv_module   = get_service_module(wrapper_request.service_path)
        module_name  = wrapper_request.service_path.split('/')[-1]
        srv_cls = getattr(srv_module,module_name)
        req_cls = getattr(srv_module,module_name+'Request')
        res_cls = getattr(srv_module,module_name+'Response')
        # instantiate the message
        request = req_cls()
        self.assign_slots(request, request_data)
        # get a service handle
        srv = get_service(wrapper_request.service_name, srv_cls)
        # send it
        response = JSONWrapperResponse()
        try:
            srv_response = srv(request)
            response.json_data = self.read_slots(res_cls,srv_response)
        except:
            response.json_data = ''
        rospy.logdebug('send: ' + response.json_data)
        # return the json encoded response
        return response
    
    #######################
    ######### Type checking
    
    def is_primitive_type(self,type_path):
        return type_path in ['bool',
                         'float32','float64',
                         'int8','int16','int32','int64',
                         'uint8','uint16','uint32','uint64',
                         'string']
    
    def is_message_array_type(self,type_path):
        if not type_path.startswith('array('):
            return False
        array_type = type_path[6:-1]
        return not self.is_primitive_type(array_type)
    
    def is_primitive_array_type(self,type_path):
        if not type_path.startswith('array('):
            return False
        array_type = type_path[6:-1]
        return self.is_primitive_type(array_type)
    
    def is_message_type(self,type_path):
        if type_path.startswith('array('):
            return False
        return not self.is_primitive_type(type_path)
    
    #######################
    ######### Decoding JSON into ROS messages
    
    def decode_json_value(self, json_value):
        [type_path,value] = json_value
        if self.is_primitive_type(type_path) or self.is_primitive_array_type(type_path):
            return value
        elif self.is_message_array_type(type_path, element_type):
            return self.decode_json_message_array(element_type, value)
        else:
            return self.decode_json_message(type_path,value)
    
    def decode_json_message(self, type_path, msg_json):
        msg_module  = get_message_module(type_path)
        module_name = type_path.split('/')[-1]
        msg         = getattr(msg_module,module_name)()
        self.assign_slots(msg, msg_json)
        return msg
    
    def decode_json_message_array(self, type_path, array_json):
        out = []
        for x in array_json:
            out.append(self.decode_json_message(type_path,x))
        return out
    
    def assign_slots(self, request_message, json_data):
        for name in json_data:
            value = self.decode_json_value(json_data[name])
            setattr(request_message, name, value)
    
    #######################
    ######### Encoding of ROS message into JSON
    
    def read_slots(self, response_cls, response_message):
        attributes = [attr for attr in dir(response_cls)
                      if not callable(getattr(response_cls, attr))
                      and not attr.startswith("_")]
        out = {}
        for a in attributes:
            out[a] = getattr(response_message,a)
        return json.dumps(out)

if __name__ == '__main__':
    rospy.init_node('json_wrapper')
    json_wrapper = JSONWrapperService()
    rospy.spin()
