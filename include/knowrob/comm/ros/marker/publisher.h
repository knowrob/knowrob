#ifndef __KNOWROB_MARKER_PUBLISHER__
#define __KNOWROB_MARKER_PUBLISHER__

#include <thread>
#include <map>

// ROS
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
// SWI Prolog
#define PL_SAFE_ARG_MACROS
#include <SWI-cpp.h>

/**
 * A marker publisher that maps Prolog terms to marker messages.
 */
class MarkerPublisher
{
public:
	MarkerPublisher(ros::NodeHandle &node);

	/**
	 * Publish an array of marker messages.
	 */
	void publish(visualization_msgs::MarkerArray &array_msg);

	/**
	 * Sets the current marker from a Prolog term and returns a reference
	 * to the marker.
	 */
	const visualization_msgs::Marker& setMarker(const PlTerm &term);

protected:
	ros::Publisher pub_;
	visualization_msgs::Marker msg_;
	std::map<std::string,int> idMap_;
	int idCounter_;

	int getID(const std::string &name);
};

#endif //__KNOWROB_MARKER_PUBLISHER__
