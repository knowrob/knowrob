
uint8 ARROW=0
uint8 CUBE=1
uint8 SPHERE=2
uint8 CYLINDER=3

string object_id                # ID of the object
string frame_name               # Frame name in TF tree
string object_type              # Object type IRI
int32 shape                     # Approximate shape
string mesh_path                # Path to mesh or empty string
std_msgs/ColorRGBA color        # Material color [0.0-1.0]
geometry_msgs/Vector3 size      # Bounding box
geometry_msgs/PoseStamped pose  # Pose of the object
# static transforms of affordances, etc.
geometry_msgs/TransformStamped[] static_transforms
