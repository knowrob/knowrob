#ifndef __KNOWROB_TF_REPUBLISHER__
#define __KNOWROB_TF_REPUBLISHER__

#include <string>

// MONGO
#include <mongoc.h>
// ROS
#include <ros/ros.h>
#include <tf/tfMessage.h>
#include <geometry_msgs/TransformStamped.h>
// SWI Prolog
#define PL_SAFE_ARG_MACROS
#include <SWI-cpp.h>

#include <knowrob/ros/tf/memory.h>
#include <knowrob/ros/tf/publisher.h>

/**
 * A TF publisher that publishes data stored in mongo DB.
 */
class TFRepublisher
{
public:
	TFRepublisher(double frequency=10.0);

	~TFRepublisher();

	void set_realtime_factor(double realtime_factor)
	{ realtime_factor_ = realtime_factor; }

	void set_loop(bool loop)
	{ loop_ = loop; }

	void set_db_name(std::string db_name)
	{ db_name_ = db_name; }

	void set_db_collection(std::string db_collection)
	{ db_collection_ = db_collection; }

	TFMemory& memory()
	{ return memory_; }

	void set_goal(double time_min, double time_max);

protected:
	double realtime_factor_;
	double frequency_;
	bool loop_;
	bool is_running_;
	std::thread thread_;

	double time_min_;
	double time_max_;
	double time_;

	std::string db_name_;
	std::string db_collection_;

	mongoc_cursor_t *cursor_;
	geometry_msgs::TransformStamped ts_;
	bool has_next_;

	TFMemory memory_;
	TFPublisher publisher_;

	void loop();
	void create_cursor();
	void reset_cursor();
	void advance_cursor(ros::Publisher &tick, double dt);
	void read_transform(const bson_t *doc);
};

#endif //__KNOWROB_TF_REPUBLISHER__
