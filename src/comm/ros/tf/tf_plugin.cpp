
#include <knowrob/comm/ros/tf/memory.h>
#include <knowrob/comm/ros/tf/logger.h>
#include <knowrob/comm/ros/tf/publisher.h>

static ros::NodeHandle node;
static TFMemory memory;
static TFPublisher pub(memory);
static TFLogger logger(node,memory);

// tf_logger_set_db_name(DBName)
PREDICATE(tf_logger_set_db_name, 1) {
	std::string db_name((char*)PL_A1);
	logger.set_db_name(db_name);
	return true;
}

//
PREDICATE(tf_logger_set_time_threshold, 1) {
	logger.set_time_threshold((double)PL_A1);
	return true;
}
PREDICATE(tf_logger_set_vectorial_threshold, 1) {
	logger.set_vectorial_threshold((double)PL_A1);
	return true;
}
PREDICATE(tf_logger_set_angular_threshold, 1) {
	logger.set_angular_threshold((double)PL_A1);
	return true;
}

//
PREDICATE(tf_logger_get_time_threshold, 1) {
	PL_A1=logger.get_time_threshold();
	return true;
}
PREDICATE(tf_logger_get_vectorial_threshold, 1) {
	PL_A1=logger.get_vectorial_threshold();
	return true;
}
PREDICATE(tf_logger_get_angular_threshold, 1) {
	PL_A1=logger.get_angular_threshold();
	return true;
}

// tf_mem_set_pose(ObjFrame,PoseData,Since)
PREDICATE(tf_mem_set_pose, 3) {
	std::string frame((char*)PL_A1);
	double stamp = (double)PL_A3;
	return memory.set_pose_term(frame,PL_A2,stamp);
}

// tf_mem_get_pose(ObjFrame,PoseData,Since)
PREDICATE(tf_mem_get_pose, 3) {
	std::string frame((char*)PL_A1);
	double stamp;
	PlTerm pose_term;
	if(memory.get_pose_term(frame,&pose_term,&stamp)) {
		PL_A2 = pose_term;
		PL_A3 = stamp;
		return true;
	}
	return false;
}

// tf_mng_store(ObjFrame,PoseData,Since)
PREDICATE(tf_mng_store, 3) {
	std::string frame((char*)PL_A1);
	double stamp = (double)PL_A3;
	geometry_msgs::TransformStamped ts;
	memory.create_transform(&ts,frame,PL_A2,stamp);
	logger.store(ts);
	return true;
}
