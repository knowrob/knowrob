#ifndef __KNOWROB_WRENCH_MEMORY__
#define __KNOWROB_WRENCH_MEMORY__

#include <string>
#include <set>
#include <map>
#include <mutex>

// MONGO
#include <mongoc.h>
// ROS
#include <ros/ros.h>
#include <geometry_msgs/WrenchStamped.h>
// SWI Prolog
#define PL_SAFE_ARG_MACROS
#include <SWI-cpp.h>

/**
 * A cache of most recent wrenches.
 */
class WrenchMemory
{
public:
	WrenchMemory();

	/**
	 * True if a wrench for a given frame was added before.
	 */
	bool has_wrench(const std::string &frame) const;

	/**
	 * Clears cached wrenches and the list of managed frames.
	 */
	bool clear();

	/**
	 * Clears cached wrenches.
	 */
	bool clear_wrenches_only();

	/**
	 * Get the wrench associated to a frame.
	 */
	const geometry_msgs::WrenchStamped& get_wrench(const std::string &frame, int buffer_index=0) const;

	/**
	 * Read a Prolog wrench term into WrenchStamped.
	 */
	void create_wrench(geometry_msgs::WrenchStamped *ts, const std::string &frame, const PlTerm &term, double stamp);

	/**
	 * Add a wrench, overwriting any previous wrench with same frame.
	 */
	void set_wrench(const geometry_msgs::WrenchStamped &ts);

	/**
	 * Add a wrench, overwriting any previous wrench with same frame.
	 */
	void set_managed_wrench(const geometry_msgs::WrenchStamped &ts);

	/**
	 * Read Prolog wrench term for frame.
	 */
	bool get_wrench_term(const std::string &frame, PlTerm *term, double *stamp);

	/**
	 * Add a wrench, overwriting any previous wrench with same frame.
	 */
	bool set_wrench_term(const std::string &frame, const PlTerm &term, double stamp);

	/**
	 * True for frames which are managed by the KB.
	 */
	bool is_managed_frame(const std::string &frame) const;

protected:
	std::set<std::string> managed_frames_[2];
	std::map<std::string, geometry_msgs::WrenchStamped> wrenches_[2];
	std::mutex wrenches_lock_;
	std::mutex names_lock_;
	int buffer_index_;
};

#endif //__KNOWROB_WRENCH_MEMORY__
