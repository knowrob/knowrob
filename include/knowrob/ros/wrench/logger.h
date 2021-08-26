#ifndef __KNOWROB_WRENCH_LOGGER__
#define __KNOWROB_WRENCH_LOGGER__

#include <string>

// MONGO
#include <mongoc.h>
// ROS
#include <ros/ros.h>
#include <geometry_msgs/WrenchStamped.h>
// SWI Prolog
#define PL_SAFE_ARG_MACROS
#include <SWI-cpp.h>

#include <knowrob/ros/wrench/memory.h>

/**
 * A wrench listener that stores messages in MongoDB.
 */
class WrenchLogger
{
public:
	WrenchLogger(ros::NodeHandle &node,
			WrenchMemory &memory,
			const std::string &topic="wrench");
	~WrenchLogger();

	void set_db_name(const std::string &db_name)
	{ db_name_ = db_name; }

	const std::string& get_db_name()
	{ return db_name_; }

	void set_time_threshold(double v)
	{ timeThreshold_ = v; }

	double get_time_threshold() const
	{ return timeThreshold_; }

	void set_force_threshold(double v)
	{ forceThreshold_ = v; }

	double get_force_threshold() const
	{ return forceThreshold_; }

	void set_torque_threshold(double v)
	{ torqueThreshold_ = v; }

	double get_torque_threshold() const
	{ return torqueThreshold_; }

	void store(const geometry_msgs::WrenchStamped &ws);

protected:
	ros::Subscriber subscriber_;
	WrenchMemory &memory_;
	double forceThreshold_;
	double torqueThreshold_;
	double timeThreshold_;
	std::string db_name_;
	std::string topic_;

	char buf_[16];
	size_t keylen_;

	void store_document(bson_t *doc);

	bool ignoreWrench(const geometry_msgs::WrenchStamped &ws);

	void callback(const geometry_msgs::WrenchStamped& msg);

	void appendWrench(bson_t *ws_doc, const geometry_msgs::WrenchStamped &ws);
};

#endif //__KNOWROB_WRENCH_LOGGER__
