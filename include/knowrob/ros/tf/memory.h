#ifndef __KNOWROB_TF_MEMORY__
#define __KNOWROB_TF_MEMORY__

#include <string>
#include <set>
#include <map>
#include <mutex>

// MONGO
#include <mongoc.h>
// ROS
#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
// SWI Prolog
#define PL_SAFE_ARG_MACROS
#include <SWI-cpp.h>

/**
 * A cache of most recent poses.
 */
class TFMemory
{
public:
	TFMemory();

	/**
	 * True if a transform with given frame was added before.
	 */
	bool has_transform(const std::string &frame) const;

	/**
	 * Get the transform associated to a frame.
	 */
	const geometry_msgs::TransformStamped& get_transform(const std::string &frame);

	/**
	 * Read a Prolog pose term into TransformStamped.
	 */
	void create_transform(geometry_msgs::TransformStamped *ts, const std::string &frame, const PlTerm &term, double stamp);

	/**
	 * Add a transform, overwriting any previous transform with same frame.
	 */
	void set_transform(const geometry_msgs::TransformStamped &ts);

	/**
	 * Read Prolog pose term for frame.
	 */
	bool get_pose_term(const std::string &frame, PlTerm *term, double *stamp);

	/**
	 * Add a transform, overwriting any previous transform with same frame.
	 */
	bool set_pose_term(const std::string &frame, const PlTerm &term, double stamp);

	/**
	 * True for frames which are managed by the KB.
	 */
	bool is_managed_frame(const std::string &frame) const;

	const std::set<std::string>& get_managed_frames() const;

protected:
	std::set<std::string> managed_frames_;
	std::map<std::string, geometry_msgs::TransformStamped> transforms_;
	std::mutex mutex_;
};

#endif //__KNOWROB_TF_MEMORY__
