#ifndef __ROSPROLOG_KB__PRIV_H__
#define __ROSPROLOG_KB__PRIV_H__

// STD
#include <thread>
// ROS
#include <ros/ros.h>
// rosprolog
#include <rosprolog/rosprolog_node/PrologPool.h>

namespace rosprolog_kb {
	/**
	 * A ROS node that is supposed to be used from
	 * Prolog predicates to interact with ROS.
	 **/
	class KBNode {
	public:
		/**
		 * The node handle.
		 */
		static ros::NodeHandle& node();
		/**
		 * A pool of Prolog engines that can be used
		 * in C++ code to call queries without
		 * going via the *rosprolog_node*.
		 */
		static PrologPool& thread_pool();
	private:
		ros::NodeHandle nh_;
		std::thread thread_;
		PrologPool thread_pool_;
		
		KBNode();
		~KBNode();
		
		KBNode(KBNode const&); // Don't Implement
		void operator=(KBNode const&);     // Don't implement
		
		void run();
		static KBNode& get();
	};
};

#endif
