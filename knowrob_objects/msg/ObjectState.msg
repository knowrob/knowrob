
# object IRI
string object_id
string frame_name
# object type IRI
string object_type
# create a marker for this object?
bool has_visual
# path to mesh if any
string mesh_path
# primary object color (or (0,0,0,0))
std_msgs/ColorRGBA color
# bounding box
geometry_msgs/Vector3 size
# current pose
geometry_msgs/PoseStamped pose
# static transforms of affordances, etc.
geometry_msgs/PoseStamped static_transforms
