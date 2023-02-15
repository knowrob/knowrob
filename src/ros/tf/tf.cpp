
#include <knowrob/ros/tf/memory.h>
#include <knowrob/ros/tf/logger.h>
#include <knowrob/ros/tf/publisher.h>
#include <knowrob/ros/tf/republisher.h>

static ros::NodeHandle node;
static TFMemory memory;
static TFPublisher pub(memory);
static TFLogger *tf_logger=NULL;

// TF logger parameter
double vectorial_threshold=0.001;
double angular_threshold=0.001;
double time_threshold=-1.0;
std::string logger_db_name="roslog";

TFRepublisher& get_republisher() {
	static TFRepublisher republisher;
	return republisher;
}

// tf_republish_set_goal(DBName,CollectionName,Time0,Time1)
PREDICATE(tf_republish_set_goal, 4) {
	std::string db_name((char*)PL_A1);
	std::string coll_name((char*)PL_A2);
	double time_min = (double)PL_A3;
	double time_max = (double)PL_A4;
	get_republisher().set_db_uri(db_name);
	get_republisher().set_db_collection(coll_name);
	get_republisher().set_goal(time_min,time_max);
	return true;
}

PREDICATE(tf_republish_set_time, 1) {
	double time = (double)PL_A1;
	get_republisher().set_now(time);
	return true;
}

PREDICATE(tf_republish_set_progress, 1) {
	double percent = (double)PL_A1;
	get_republisher().set_progress(percent);
	return true;
}

PREDICATE(tf_republish_clear, 0) {
	get_republisher().clear();
	return true;
}

// tf_republish_set_loop(RealtimeFactor)
PREDICATE(tf_republish_set_loop, 1) {
	get_republisher().set_loop((int)PL_A1);
	return true;
}

// tf_republish_set_realtime_factor(RealtimeFactor)
PREDICATE(tf_republish_set_realtime_factor, 1) {
	double realtime_factor = (double)PL_A1;
	get_republisher().set_realtime_factor(realtime_factor);
	return true;
}

// tf_logger_enable
PREDICATE(tf_logger_enable, 0) {
	if(tf_logger) {
		delete tf_logger;
	}

	tf_logger = new TFLogger(node,memory);
	tf_logger->set_db_name(logger_db_name);
	tf_logger->set_time_threshold(time_threshold);
	tf_logger->set_vectorial_threshold(vectorial_threshold);
	tf_logger->set_angular_threshold(angular_threshold);
	return true;
}

// tf_logger_disable
PREDICATE(tf_logger_disable, 0) {
	if(tf_logger) {
		delete tf_logger;
		tf_logger = NULL;
	}
	return true;
}

// tf_logger_set_db_name(DBName)
PREDICATE(tf_logger_set_db_name, 1) {
	std::string db_name((char*)PL_A1);
	logger_db_name = db_name;
	return true;
}

//
PREDICATE(tf_logger_set_time_threshold, 1) {
	time_threshold = (double)PL_A1;
	return true;
}
PREDICATE(tf_logger_set_vectorial_threshold, 1) {
	vectorial_threshold = (double)PL_A1;
	return true;
}
PREDICATE(tf_logger_set_angular_threshold, 1) {
	angular_threshold = (double)PL_A1;
	return true;
}

//
PREDICATE(tf_logger_get_time_threshold, 1) {
	PL_A1 = time_threshold;
	return true;
}
PREDICATE(tf_logger_get_vectorial_threshold, 1) {
	PL_A1 = vectorial_threshold;
	return true;
}
PREDICATE(tf_logger_get_angular_threshold, 1) {
	PL_A1 = angular_threshold;
	return true;
}


// Clear the tf memory to remove cached transforms
PREDICATE(tf_mem_clear, 0) {
	memory.clear();
	return true;
}

// tf_mem_set_pose(ObjFrame,PoseData,Since)
PREDICATE(tf_mem_set_pose, 3) {
	std::string frame((char*)PL_A1);
	double stamp = (double)PL_A3;
	memory.set_pose_term(frame,PL_A2,stamp);
	return true;
}

// tf_republish_set_pose(ObjFrame,PoseData)
PREDICATE(tf_republish_set_pose, 2) {
	std::string frame((char*)PL_A1);
	get_republisher().memory().set_pose_term(frame,PL_A2,-1.0);
	return true;
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
	if(tf_logger) {
		tf_logger->store(ts);
	}
	return true;
}
