# Semantic map object message type, Moritz Tenorth, tenorth@cs.tum.edu
int32 id          # unique object identifier
string type       # object class: {oven, cupboard, refrigerator, table, countertop, door, hinge, handle, knob, sink}
float32 width     # object width
float32 depth     # object width
float32 height    # object width
float32[] pose    # 4x4 pose matrix, row-based
int32   partOf    # ID of the parent object (consistent with the ID on the first line)
