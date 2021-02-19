#include <knowrob/ros/tf/publisher.h>
#include <tf/tfMessage.h>

// TODO: handle static object transforms (e.g. for features, srdl components also have static transforms relative to base link)

TFPublisher::TFPublisher(TFMemory &memory, double frequency, bool clear_after_publish) :
		memory_(memory),
		is_running_(true),
		clear_after_publish_(clear_after_publish),
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
	tf2_ros::TransformBroadcaster tf_broadcaster;
	ros::Rate r(frequency_);
	while(ros::ok()) {
		publishTransforms(tf_broadcaster);
		r.sleep();
		if(!is_running_) break;
	}
}

void TFPublisher::publishTransforms(tf2_ros::TransformBroadcaster &tf_broadcaster)
{
	tf::tfMessage tf_msg;
	memory_.loadTF(tf_msg, clear_after_publish_);
	tf_broadcaster.sendTransform(tf_msg.transforms);
}
