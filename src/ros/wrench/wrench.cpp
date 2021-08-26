
#include <knowrob/ros/wrench/memory.h>
#include <knowrob/ros/wrench/logger.h>

static ros::NodeHandle node;
static WrenchMemory memory;
static WrenchLogger *wrench_logger=NULL;

// wrench logger parameter
double force_threshold=0.001;
double torque_threshold=0.001;
double time_threshold=-1.0;
std::string logger_db_name="roslog";

// Clear the wrench memory to remove cached wrenches
PREDICATE(wrench_mem_clear, 0) {
	memory.clear();
	return true;
}

// wrench_mem_set(ObjFrame,WrenchData,Since)
PREDICATE(wrench_mem_set, 3) {
	std::string frame((char*)PL_A1);
	double stamp = (double)PL_A3;
	memory.set_wrench_term(frame,PL_A2,stamp);
	return true;
}

// wrench_mem_get(ObjFrame,WrenchData,Since)
PREDICATE(wrench_mem_get, 3) {
	std::string frame((char*)PL_A1);
	double stamp;
	PlTerm wrench_term;
	if(memory.get_wrench_term(frame,&wrench_term,&stamp)) {
		PL_A2 = wrench_term;
		PL_A3 = stamp;
		return true;
	}
	return false;
}

// wrench_logger_enable
PREDICATE(wrench_logger_enable, 0) {
	if(wrench_logger) {
		delete wrench_logger;
	}

	wrench_logger = new WrenchLogger(node,memory);
	wrench_logger->set_db_name(logger_db_name);
	wrench_logger->set_time_threshold(time_threshold);
	wrench_logger->set_force_threshold(force_threshold);
	wrench_logger->set_torque_threshold(torque_threshold);
	return true;
}

// wrench_logger_disable
PREDICATE(wrench_logger_disable, 0) {
	if(wrench_logger) {
		delete wrench_logger;
		wrench_logger = NULL;
	}
	return true;
}

// wrench_logger_set_db_name(DBName)
PREDICATE(wrench_logger_set_db_name, 1) {
	std::string db_name((char*)PL_A1);
	logger_db_name = db_name;
	return true;
}

//
PREDICATE(wrench_logger_set_time_threshold, 1) {
	time_threshold = (double)PL_A1;
	return true;
}
PREDICATE(wrench_logger_set_force_threshold, 1) {
	force_threshold = (double)PL_A1;
	return true;
}
PREDICATE(wrench_logger_set_torque_threshold, 1) {
	torque_threshold = (double)PL_A1;
	return true;
}

//
PREDICATE(wrench_logger_get_time_threshold, 1) {
	PL_A1 = time_threshold;
	return true;
}
PREDICATE(wrench_logger_get_force_threshold, 1) {
	PL_A1 = force_threshold;
	return true;
}
PREDICATE(wrench_logger_get_torque_threshold, 1) {
	PL_A1 = torque_threshold;
	return true;
}

// wrench_mng_store(ObjFrame,WrenchData,Since)
PREDICATE(wrench_mng_store, 3) {
	std::string frame((char*)PL_A1);
	double stamp = (double)PL_A3;
	geometry_msgs::WrenchStamped ws;
	memory.create_wrench(&ws,frame,PL_A2,stamp);
	if(wrench_logger) {
		wrench_logger->store(ws);
	}
	return true;
}