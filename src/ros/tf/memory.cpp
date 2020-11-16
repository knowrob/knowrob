#include <knowrob/ros/tf/memory.h>

static double get_stamp(const geometry_msgs::TransformStamped &ts)
{
	unsigned long long time = (unsigned long long)(
			ts.header.stamp.sec * 1000.0 + ts.header.stamp.nsec / 1000000.0);
	return (double)(time/1000.0);
}

TFMemory::TFMemory()
{
}

bool TFMemory::has_transform(const std::string &frame) const
{
	return transforms_.find(frame) != transforms_.end();
}

bool TFMemory::is_managed_frame(const std::string &frame) const
{
	return managed_frames_.find(frame) != managed_frames_.end();
}

const std::set<std::string>& TFMemory::get_managed_frames() const
{
	return managed_frames_;
}

const geometry_msgs::TransformStamped& TFMemory::get_transform(const std::string &frame)
{
	return transforms_[frame];
}

void TFMemory::set_transform(const geometry_msgs::TransformStamped &ts)
{
	transforms_[ts.child_frame_id] = ts;
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
	if(get_stamp(ts_old)>stamp) {
		return false;
	}
	//
	geometry_msgs::TransformStamped ts = ts_old;
	create_transform(&ts,frame,term,stamp);
	managed_frames_.insert(frame);
	set_transform(ts);
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
