# Inspired by https://github.com/code-iai/iai_common_msgs/blob/master/designator_integration_msgs/msg/KeyValuePair.msg
# Data
int32 TYPE_STRING=0
int32 TYPE_FLOAT=1
int32 TYPE_INT=2
int32 TYPE_LONG=3
int32 TYPE_VARIABLE=4
int32 TYPE_PREDICATE=5
int32 TYPE_LIST=6

int32 type

string key
string value_string
float64 value_float
int32 value_int
int64 value_long
string value_variable
string value_predicate
# List can be nested and with varying
# datatypes in KnowRob. This can't be
# represented in ROS Messages, so the
# use would have to parse the answer
string value_list