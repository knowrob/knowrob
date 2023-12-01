#include <tf/LinearMath/Quaternion.h>

#include "knowrob/reasoner/tf/logger.h"
#include "knowrob/mongodb/MongoInterface.h"

using namespace knowrob;
using namespace knowrob::mongo;

TFLogger::TFLogger(
		ros::NodeHandle &node,
		TFMemory &memory,
		const std::string &topic) :
		memory_(memory),
		timeThreshold_(-1.0),
		vectorialThreshold_(0.001),
		angularThreshold_(0.001),
		db_name_("roslog"),
		topic_(topic),
		subscriber_static_(node.subscribe("tf_static", 1000, &TFLogger::callback, this)),
		subscriber_(node.subscribe(topic, 1000, &TFLogger::callback, this))
{
}

TFLogger::~TFLogger()
{
}

void TFLogger::store_document(bson_t *doc)
{
	bson_error_t err;
    Document tfMessage(bson_new());
    std::shared_ptr<Collection> collection_(MongoInterface::get().connect(
            db_uri_.c_str(), db_name_.c_str(),topic_.c_str()));
	collection_->storeOne(tfMessage);
}

void TFLogger::store(const geometry_msgs::TransformStamped &ts)
{
	bson_t *doc = bson_new();
	appendTransform(doc, ts);
	BSON_APPEND_DATE_TIME(doc, "__recorded", time(NULL) * 1000);
	BSON_APPEND_UTF8(doc, "__topic", topic_.c_str());
	store_document(doc);
	bson_destroy(doc);
}

void TFLogger::callback(const tf::tfMessage::ConstPtr& msg)
{
	std::vector<geometry_msgs::TransformStamped>::const_iterator it;
	for (it = msg->transforms.begin(); it != msg->transforms.end(); ++it)
	{
		const geometry_msgs::TransformStamped &ts = *it;
		if(!ignoreTransform(ts)) {
			store(ts);
			// NOTE: IMPORTANT: we assign *ts* to its frame in memory
			//       such that we can check for the pose difference in
			//       ignoreTransform.
			//       This is only needed for the frames that are not
			//       managed by knowrob (e.g. robot frames handled by robot
			//       state publisher).
			// TODO: not sure if it is worth doing the check here...
			if(!memory_.is_managed_frame(ts.child_frame_id)) {
				memory_.set_transform(ts);
			}
		}
	}
}

bool TFLogger::ignoreTransform(const geometry_msgs::TransformStamped &ts0)
{
	const std::string &child  = ts0.child_frame_id;
	if(!memory_.has_transform(child)) {
		// it's a new frame
		return false;
	}
	if(memory_.is_managed_frame(child)) {
		// managed frames are asserted into DB back-end directly
		return true;
	}
	const geometry_msgs::TransformStamped &ts1 = memory_.get_transform(child);
	// do not ignore in case parent frame has changed
	const std::string &parent0 = ts0.header.frame_id;
	const std::string &parent1 = ts1.header.frame_id;
	if(parent0.compare(parent1)!=0) {
		return false;
	}
	// tests whether temporal distance exceeds threshold
	if(timeThreshold_>0.0) {
		double timeDistance = (
				(ts0.header.stamp.sec * 1000.0 + ts0.header.stamp.nsec / 1000000.0) -
				(ts1.header.stamp.sec * 1000.0 + ts1.header.stamp.nsec / 1000000.0)) / 1000.0;
		if(timeDistance<0 || timeDistance > timeThreshold_) {
			return false;
		}
	}
	// tests whether vectorial distance exceeds threshold
	const geometry_msgs::Vector3 &pos0 = ts0.transform.translation;
	const geometry_msgs::Vector3 &pos1 = ts1.transform.translation;
	double vectorialDistance = sqrt(
			((pos0.x - pos1.x) * (pos0.x - pos1.x)) +
			((pos0.y - pos1.y) * (pos0.y - pos1.y)) +
			((pos0.z - pos1.z) * (pos0.z - pos1.z)));
	if(vectorialDistance > vectorialThreshold_) {
		return false;
	}
	// tests whether angular distance exceeds threshold
	const geometry_msgs::Quaternion &rot0 = ts0.transform.rotation;
	const geometry_msgs::Quaternion &rot1 = ts1.transform.rotation;
    tf::Quaternion q1(rot0.x, rot0.y, rot0.z, rot0.w);
    tf::Quaternion q2(rot1.x, rot1.y, rot1.z, rot1.w);
    float angularDistance = 2.0 * fabs(q1.angle(q2));
	if(angularDistance > angularThreshold_) {
		return false;
	}
	return true;
}

void TFLogger::appendTransform(bson_t *ts_doc, const geometry_msgs::TransformStamped &ts)
{
	bson_t child, child2;
	// append header
	BSON_APPEND_DOCUMENT_BEGIN(ts_doc, "header", &child); {
		unsigned long long time = (unsigned long long)(
				ts.header.stamp.sec * 1000.0 + ts.header.stamp.nsec / 1000000.0);
		//
		BSON_APPEND_INT32(&child, "seq", ts.header.seq);
		BSON_APPEND_DATE_TIME(&child, "stamp", time);
		BSON_APPEND_UTF8(&child, "frame_id", ts.header.frame_id.c_str());
		bson_append_document_end(ts_doc, &child);
	}
	// append child frame
	BSON_APPEND_UTF8(ts_doc, "child_frame_id", ts.child_frame_id.c_str());
	// append transform
	BSON_APPEND_DOCUMENT_BEGIN(ts_doc, "transform", &child); {
		// append translation
		BSON_APPEND_DOCUMENT_BEGIN(&child, "translation", &child2); {
			BSON_APPEND_DOUBLE(&child2, "x", ts.transform.translation.x);
			BSON_APPEND_DOUBLE(&child2, "y", ts.transform.translation.y);
			BSON_APPEND_DOUBLE(&child2, "z", ts.transform.translation.z);
			bson_append_document_end(&child, &child2);
		}
		// append rotation
		BSON_APPEND_DOCUMENT_BEGIN(&child, "rotation", &child2); {
			BSON_APPEND_DOUBLE(&child2, "x", ts.transform.rotation.x);
			BSON_APPEND_DOUBLE(&child2, "y", ts.transform.rotation.y);
			BSON_APPEND_DOUBLE(&child2, "z", ts.transform.rotation.z);
			BSON_APPEND_DOUBLE(&child2, "w", ts.transform.rotation.w);
			bson_append_document_end(&child, &child2);
		}
		bson_append_document_end(ts_doc, &child);
	}
}
