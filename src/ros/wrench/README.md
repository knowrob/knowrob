Force/torque integration
=======

KnowRob supports storing and loading subsymbolic data about wrenches (forces & torques) acting on objects. This works exactly like the mechanism for loading and storing /tf.

Wrenches are always associated with the object they act on. The wrench is defined relative to that object's local coordinate system. See `wrench.plt` for examples.

Unlike tf data, wrench data cannot be transformed into other coordinate systems by KnowRob (yet). Support for wrench transformations as well as gravity/lever compensation is planned for the future.
