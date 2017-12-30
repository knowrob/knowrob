The Semantic Robot Description Language (SRDL) extends KnowRob with representations for robot hardware,
robot software and robot capabilities. The hardware models can automatically be
[[imported from a URDF description][http://knowrob.org/doc/create_srdl_model]],
and are annotated with additional semantic information.
For example, one can annotate semantically meaningful groups of links such
as the left arm or the right gripper (see image on the right). SRDL integrates
this information into KnowRob, allowing the system to reason about the robot's configuration.
In addition, SRDL provides inference mechanisms that operate on the robot model and
are able to check which dependencies of
[[action descriptions][http://knowrob.org/doc/doc/modeling_tasks_and_actions]]
are available on the robot. This allows to identify missing components or capabilities.