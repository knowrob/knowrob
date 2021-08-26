#include <knowrob/ros/wrench/logger.h>
#include <knowrob/db/mongo/MongoInterface.h>

WrenchLogger::WrenchLogger(ros::NodeHandle &node, WrenchMemory &memory, const std::string &topic)
	: memory_(memory), timeThreshold_(-1.0), forceThreshold_(0.001), torqueThreshold_(0.001),
	  db_name_("roslog"), topic_(topic), subscriber_(node.subscribe(topic, 1000, &WrenchLogger::callback, this))
{
}

WrenchLogger::~WrenchLogger()
{
}

void WrenchLogger::store_document(bson_t *doc)
{
	bson_error_t err;
	MongoCollection *collection = MongoInterface::get_collection(db_name_.c_str(),topic_.c_str());
	if(!mongoc_collection_insert(
			(*collection)(),MONGOC_INSERT_NONE,doc,NULL,&err))
	{
		ROS_WARN("[WrenchLogger] insert failed: %s.", err.message);
	}
	delete collection;
}

void WrenchLogger::store(const geometry_msgs::WrenchStamped &ws)
{
	bson_t *doc = bson_new();
	appendWrench(doc, ws);
	BSON_APPEND_DATE_TIME(doc, "__recorded", time(NULL) * 1000);
	BSON_APPEND_UTF8(doc, "__topic", topic_.c_str());
	store_document(doc);
	bson_destroy(doc);
}

void WrenchLogger::callback(const geometry_msgs::WrenchStamped& ws)
{
	if(!ignoreWrench(ws)) {
		store(ws);
		if(!memory_.is_managed_frame(ws.header.frame_id)) {
			memory_.set_wrench(ws);
		}
	}
}

bool WrenchLogger::ignoreWrench(const geometry_msgs::WrenchStamped &ws0)
{
	const std::string &frame_id  = ws0.header.frame_id;
	if(!memory_.has_wrench(frame_id)) {
		// it's a new frame
		return false;
	}
	if(memory_.is_managed_frame(frame_id)) {
		// managed frames are asserted into DB back-end directly
		return true;
	}
	const geometry_msgs::WrenchStamped &ws1 = memory_.get_wrench(frame_id);
	// test whether temporal distance exceeds threshold
	if(timeThreshold_>0.0) {
		double timeDistance = (
				(ws0.header.stamp.sec * 1000.0 + ws0.header.stamp.nsec / 1000000.0) -
				(ws1.header.stamp.sec * 1000.0 + ws1.header.stamp.nsec / 1000000.0)) / 1000.0;
		if(timeDistance<0 || timeDistance > timeThreshold_) {
			return false;
		}
	}
	// test whether force distance exceeds threshold
	const geometry_msgs::Vector3 &force0 = ws0.wrench.force;
	const geometry_msgs::Vector3 &force1 = ws1.wrench.force;
	double forceDistance = sqrt(
			((force0.x - force1.x) * (force0.x - force1.x)) +
			((force0.y - force1.y) * (force0.y - force1.y)) +
			((force0.z - force1.z) * (force0.z - force1.z)));
	if(forceDistance > forceThreshold_) {
		return false;
	}
	// test whether torque distance exceeds threshold
	const geometry_msgs::Vector3 &torque0 = ws0.wrench.torque;
	const geometry_msgs::Vector3 &torque1 = ws1.wrench.torque;
	double torqueDistance = sqrt(
			((torque0.x - torque1.x) * (torque0.x - torque1.x)) +
			((torque0.y - torque1.y) * (torque0.y - torque1.y)) +
			((torque0.z - torque1.z) * (torque0.z - torque1.z)));
	if(torqueDistance > torqueThreshold_) {
		return false;
	}
	return true;
}

void WrenchLogger::appendWrench(bson_t *ws_doc, const geometry_msgs::WrenchStamped &ws)
{
	bson_t child, child2;
	// append header
	BSON_APPEND_DOCUMENT_BEGIN(ws_doc, "header", &child); {
		unsigned long long time = (unsigned long long)(
				ws.header.stamp.sec * 1000.0 + ws.header.stamp.nsec / 1000000.0);
		//
		BSON_APPEND_INT32(&child, "seq", ws.header.seq);
		BSON_APPEND_DATE_TIME(&child, "stamp", time);
		BSON_APPEND_UTF8(&child, "frame_id", ws.header.frame_id.c_str());
		bson_append_document_end(ws_doc, &child);
	}
	// append wrench
	BSON_APPEND_DOCUMENT_BEGIN(ws_doc, "wrench", &child); {
		// append force
		BSON_APPEND_DOCUMENT_BEGIN(&child, "force", &child2); {
			BSON_APPEND_DOUBLE(&child2, "x", ws.wrench.force.x);
			BSON_APPEND_DOUBLE(&child2, "y", ws.wrench.force.y);
			BSON_APPEND_DOUBLE(&child2, "z", ws.wrench.force.z);
			bson_append_document_end(&child, &child2);
		}
		// append torque
		BSON_APPEND_DOCUMENT_BEGIN(&child, "torque", &child2); {
			BSON_APPEND_DOUBLE(&child2, "x", ws.wrench.torque.x);
			BSON_APPEND_DOUBLE(&child2, "y", ws.wrench.torque.y);
			BSON_APPEND_DOUBLE(&child2, "z", ws.wrench.torque.z);
			bson_append_document_end(&child, &child2);
		}
		bson_append_document_end(ws_doc, &child);
	}
}
