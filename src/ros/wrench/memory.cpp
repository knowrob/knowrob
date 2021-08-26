#include <knowrob/ros/wrench/memory.h>

static inline double get_stamp(const geometry_msgs::WrenchStamped &msg)
{
	unsigned long long time = (unsigned long long)(
			msg.header.stamp.sec * 1000.0 + msg.header.stamp.nsec / 1000000.0);
	return (double)(time/1000.0);
}

static geometry_msgs::WrenchStamped dummy;

WrenchMemory::WrenchMemory() :
		buffer_index_(0)
{
}

bool WrenchMemory::has_wrench(const std::string &frame) const
{
	return wrenches_[buffer_index_].find(frame) != wrenches_[buffer_index_].end();
}

bool WrenchMemory::is_managed_frame(const std::string &frame) const
{
	return managed_frames_[buffer_index_].find(frame) != managed_frames_[buffer_index_].end();
}

bool WrenchMemory::clear()
{
	std::lock_guard<std::mutex> guard1(wrenches_lock_);
	std::lock_guard<std::mutex> guard2(names_lock_);
	wrenches_[buffer_index_].clear();
	managed_frames_[buffer_index_].clear();
	return true;
}

bool WrenchMemory::clear_wrenches_only()
{
	std::lock_guard<std::mutex> guard1(wrenches_lock_);
	wrenches_[buffer_index_].clear();
	return true;
}

const geometry_msgs::WrenchStamped& WrenchMemory::get_wrench(
		const std::string &frame, int buffer_index) const
{
	const std::map<std::string, geometry_msgs::WrenchStamped>::const_iterator
		&needle = wrenches_[buffer_index].find(frame);
	if(needle != wrenches_[buffer_index].end()) {
		return needle->second;
	}
	else {
		return dummy;
	}
}

void WrenchMemory::set_wrench(const geometry_msgs::WrenchStamped &ws)
{
	std::lock_guard<std::mutex> guard(wrenches_lock_);
	wrenches_[buffer_index_][ws.header.frame_id] = ws;
}

void WrenchMemory::set_managed_wrench(const geometry_msgs::WrenchStamped &ws)
{
	std::lock_guard<std::mutex> guard1(wrenches_lock_);
	std::lock_guard<std::mutex> guard2(names_lock_);
	managed_frames_[buffer_index_].insert(ws.header.frame_id);
	wrenches_[buffer_index_][ws.header.frame_id] = ws;
}

bool WrenchMemory::get_wrench_term(const std::string &frame, PlTerm *term, double *stamp)
{
	if(!has_wrench(frame)) return false;
	const geometry_msgs::WrenchStamped &ws = get_wrench(frame);

	PlTail wrench_list(*term); {
		wrench_list.append(ws.header.frame_id.c_str());
		//
		PlTerm force_term;
		PlTail force_list(force_term); {
			force_list.append(ws.wrench.force.x);
			force_list.append(ws.wrench.force.y);
			force_list.append(ws.wrench.force.z);
			force_list.close();
		}
		wrench_list.append(force_term);
		//
		PlTerm torque_term;
		PlTail torque_list(torque_term); {
			torque_list.append(ws.wrench.force.x);
			torque_list.append(ws.wrench.force.y);
			torque_list.append(ws.wrench.force.z);
			torque_list.close();
		}
		wrench_list.append(torque_term);
		wrench_list.close();
	}
	// get unix timestamp
	*stamp = get_stamp(ws);

	return true;
}

bool WrenchMemory::set_wrench_term(const std::string &frame, const PlTerm &term, double stamp)
{
	const geometry_msgs::WrenchStamped &ws_old = get_wrench(frame);
	// make sure the wrench is more recent then the one stored
	if(stamp<0.0) {
		// force setting wrench if stamp<0.0
		stamp = 0.0;
	}
	else if(get_stamp(ws_old)>stamp) {
		return false;
	}
	//
	geometry_msgs::WrenchStamped ws = ws_old;
	create_wrench(&ws,frame,term,stamp);
	set_managed_wrench(ws);
	return true;
}

void WrenchMemory::create_wrench(
		geometry_msgs::WrenchStamped *ws,
		const std::string &frame,
		const PlTerm &term,
		double stamp)
{
	PlTail tail(term);
	PlTerm e,j;
	// header
	ws->header.frame_id = frame;
	unsigned long long time_ms = ((unsigned long long)(stamp*1000.0));
	ws->header.stamp.sec  = time_ms / 1000;
	ws->header.stamp.nsec = (time_ms % 1000) * 1000 * 1000;
	// force
	tail.next(e);
	PlTail force_list(e);
	force_list.next(j); ws->wrench.force.x =(double)j;
	force_list.next(j); ws->wrench.force.y =(double)j;
	force_list.next(j); ws->wrench.force.z =(double)j;
	// torque
	tail.next(e);
	PlTail torque_list(e);
	torque_list.next(j); ws->wrench.torque.x =(double)j;
	torque_list.next(j); ws->wrench.torque.y =(double)j;
	torque_list.next(j); ws->wrench.torque.z =(double)j;
}
