#include <knowrob/comm/ros/tf/publisher.h>
#include <tf/tfMessage.h>

// TODO: handle static object transforms (e.g. for features, srdl components also have static transforms relative to base link)

TFPublisher::TFPublisher(TFMemory &memory, double frequency) :
		memory_(memory),
		is_running_(true),
		frequency_(frequency),
	    thread_(&TFPublisher::loop, this)
{
}

TFPublisher::~TFPublisher()
{
	is_running_ = false;
	thread_.join();
}

void TFPublisher::loop()
{
	ros::Rate r(frequency_);
	while(ros::ok()) {
		publishTransforms();
		r.sleep();
		if(!is_running_) break;
	}
}

void TFPublisher::publishTransforms()
{
	const std::set<std::string>::const_iterator &begin = memory_.get_managed_frames().begin();
	const std::set<std::string>::const_iterator &end   = memory_.get_managed_frames().end();
	if(begin==end) return;
	//
	tf::tfMessage tf_msg;
	const ros::Time& time = ros::Time::now();
	// loop over all frames
	for(std::set<std::string>::const_iterator it=begin; it!=end; ++it) {
		// TODO: better avoid get_transform here, could memory_ store a list of managed transforms?
		geometry_msgs::TransformStamped tf_transform = memory_.get_transform(*it);
		tf_transform.header.stamp = time;
		tf_msg.transforms.push_back(tf_transform);
	}
	tf_broadcaster_.sendTransform(tf_msg.transforms);
}
