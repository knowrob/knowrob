#include <knowrob/ros/tf/memory.h>

static inline double get_stamp(const geometry_msgs::TransformStamped &ts)
{
	unsigned long long time = (unsigned long long)(
			ts.header.stamp.sec * 1000.0 + ts.header.stamp.nsec / 1000000.0);
	return (double)(time/1000.0);
}

static geometry_msgs::TransformStamped dummy;

//#define SEND_UNKNOWN_FAR_AWAY
#ifdef SEND_UNKNOWN_FAR_AWAY
static geometry_msgs::TransformStamped far_away;
#endif

TFMemory::TFMemory() :
		buffer_index_(0)
{
#ifdef SEND_UNKNOWN_FAR_AWAY
	far_away.transform.translation.x = 99999.9;
	far_away.transform.translation.y = 99999.9;
	far_away.transform.translation.z = 99999.9;
	far_away.transform.rotation.x = 0.0;
	far_away.transform.rotation.y = 0.0;
	far_away.transform.rotation.z = 0.0;
	far_away.transform.rotation.w = 1.0;
	far_away.header.frame_id = "map";
#endif
}

bool TFMemory::has_transform(const std::string &frame) const
{
	return transforms_[buffer_index_].find(frame) != transforms_[buffer_index_].end();
}

bool TFMemory::is_managed_frame(const std::string &frame) const
{
	return managed_frames_[buffer_index_].find(frame) != managed_frames_[buffer_index_].end();
}

bool TFMemory::clear()
{
	std::lock_guard<std::mutex> guard1(transforms_lock_);
	std::lock_guard<std::mutex> guard2(names_lock_);
	transforms_[buffer_index_].clear();
	managed_frames_[buffer_index_].clear();
	return true;
}

bool TFMemory::clear_transforms_only()
{
	std::lock_guard<std::mutex> guard1(transforms_lock_);
	transforms_[buffer_index_].clear();
	return true;
}

const geometry_msgs::TransformStamped& TFMemory::get_transform(
		const std::string &frame, int buffer_index) const
{
	const std::map<std::string, geometry_msgs::TransformStamped>::const_iterator
		&needle = transforms_[buffer_index].find(frame);
	if(needle != transforms_[buffer_index].end()) {
		return needle->second;
	}
	else {
		return dummy;
	}
}

void TFMemory::set_transform(const geometry_msgs::TransformStamped &ts)
{
	std::lock_guard<std::mutex> guard(transforms_lock_);
	transforms_[buffer_index_][ts.child_frame_id] = ts;
}

void TFMemory::set_managed_transform(const geometry_msgs::TransformStamped &ts)
{
	std::lock_guard<std::mutex> guard1(transforms_lock_);
	std::lock_guard<std::mutex> guard2(names_lock_);
	managed_frames_[buffer_index_].insert(ts.child_frame_id);
	transforms_[buffer_index_][ts.child_frame_id] = ts;
}

bool TFMemory::loadTF(tf::tfMessage &tf_msg, bool clear_memory)
{
	if(managed_frames_[buffer_index_].empty()) {
		return true;
	}

	if(clear_memory) {
		int pong = buffer_index_;
		// ping-pong. pong buffer can then be used without lock.
		// and other threads start writing into ping buffer.
		buffer_index_ = (pong==0 ? 1 : 0);
		// load transforms without locking
		loadTF_internal(tf_msg,pong);
		// clear the pong buffer
		managed_frames_[pong].clear();
		transforms_[pong].clear();
	}
	else {
		// load transforms while locking *managed_frames_*.
		// it is ok though if transforms_ is written to.
		std::lock_guard<std::mutex> guard(names_lock_);
		loadTF_internal(tf_msg,buffer_index_);
	}
	return true;
}

void TFMemory::loadTF_internal(tf::tfMessage &tf_msg, int buffer_index)
{
	const ros::Time& time = ros::Time::now();
	// loop over all frames
	for(std::set<std::string>::const_iterator
			it=managed_frames_[buffer_index].begin();
			it!=managed_frames_[buffer_index].end(); ++it)
	{
		const std::string &name = *it;
		std::map<std::string, geometry_msgs::TransformStamped>::iterator
			needle = transforms_[buffer_index].find(name);
		if(needle != transforms_[buffer_index].end()) {
			geometry_msgs::TransformStamped &tf_transform = needle->second;
			tf_transform.header.stamp = time;
			tf_msg.transforms.push_back(tf_transform);
		}
#ifdef SEND_UNKNOWN_FAR_AWAY
		else {
			far_away.header.stamp = time;
			far_away.child_frame_id = name;
			tf_msg.transforms.push_back(far_away);
		}
#endif
	}
}

bool TFMemory::get_pose_term(const std::string &frame, PlTerm *term, double *stamp)
{
	if(!has_transform(frame)) return false;
	const geometry_msgs::TransformStamped &ts = get_transform(frame);

	PlTail pose_list(*term); {
		pose_list.append(ts.header.frame_id.c_str());
		//
		PlTerm pos_term;
		PlTail pos_list(pos_term); {
			pos_list.append(ts.transform.translation.x);
			pos_list.append(ts.transform.translation.y);
			pos_list.append(ts.transform.translation.z);
			pos_list.close();
		}
		pose_list.append(pos_term);
		//
		PlTerm rot_term;
		PlTail rot_list(rot_term); {
			rot_list.append(ts.transform.rotation.x);
			rot_list.append(ts.transform.rotation.y);
			rot_list.append(ts.transform.rotation.z);
			rot_list.append(ts.transform.rotation.w);
			rot_list.close();
		}
		pose_list.append(rot_term);
		pose_list.close();
	}
	// get unix timestamp
	*stamp = get_stamp(ts);

	return true;
}

bool TFMemory::set_pose_term(const std::string &frame, const PlTerm &term, double stamp)
{
	const geometry_msgs::TransformStamped &ts_old = get_transform(frame);
	// make sure the pose is more recent then the one stored
	if(stamp<0.0) {
		// force setting pose if stamp<0.0
		stamp = 0.0;
	}
	else if(get_stamp(ts_old)>stamp) {
		return false;
	}
	//
	geometry_msgs::TransformStamped ts = ts_old;
	create_transform(&ts,frame,term,stamp);
	set_managed_transform(ts);
	return true;
}

void TFMemory::create_transform(
		geometry_msgs::TransformStamped *ts,
		const std::string &frame,
		const PlTerm &term,
		double stamp)
{
	PlTail tail(term); PlTerm e,j;
	// frame
	ts->child_frame_id = frame;
	// header
	tail.next(e);
	ts->header.frame_id = std::string((char*)e);
	unsigned long long time_ms = ((unsigned long long)(stamp*1000.0));
	ts->header.stamp.sec  = time_ms / 1000;
	ts->header.stamp.nsec = (time_ms % 1000) * 1000 * 1000;
	// translation
	tail.next(e);
	PlTail pos_list(e);
	pos_list.next(j); ts->transform.translation.x =(double)j;
	pos_list.next(j); ts->transform.translation.y =(double)j;
	pos_list.next(j); ts->transform.translation.z =(double)j;
	// rotation
	tail.next(e);
	PlTail rot_list(e);
	rot_list.next(j); ts->transform.rotation.x =(double)j;
	rot_list.next(j); ts->transform.rotation.y =(double)j;
	rot_list.next(j); ts->transform.rotation.z =(double)j;
	rot_list.next(j); ts->transform.rotation.w =(double)j;
}
