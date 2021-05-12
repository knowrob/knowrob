#include <knowrob/ros/tf/republisher.h>
#include <std_msgs/Float64.h>

#define CLEAR_MEMORY_AFTER_PUBLISH 0

// TODO: buffering of poses, i.e. pre-load the next 10s.

TFRepublisher::TFRepublisher(double frequency) :
		realtime_factor_(1.0),
		frequency_(frequency),
		loop_(true),
		has_next_(false),
		has_new_goal_(false),
		is_running_(true),
		reset_(false),
		has_been_skipped_(false),
		skip_reset_(false),
		db_name_("neems"),
		db_collection_("tf"),
		collection_(NULL),
		time_min_(0.0),
		time_max_(0.0),
		time_(0.0),
		cursor_(NULL),
		memory_(),
		publisher_(memory_,frequency,CLEAR_MEMORY_AFTER_PUBLISH),
	    thread_(&TFRepublisher::loop, this),
	    tick_thread_(&TFRepublisher::tick_loop, this)
{
}

TFRepublisher::~TFRepublisher()
{
	is_running_ = false;
	thread_.join();
	tick_thread_.join();
	if(collection_) {
		delete collection_;
		collection_ = NULL;
	}
}

void TFRepublisher::clear()
{
	has_next_ = false;
	time_min_ = 0.0;
	time_max_ = 0.0;
	time_ = 0.0;
	memory_.clear();
}

void TFRepublisher::tick_loop()
{
	ros::NodeHandle node;
	ros::Rate r(frequency_);
	ros::Publisher tick(node.advertise<std_msgs::Float64>("republisher_tick",1));
	std_msgs::Float64 time_msg;
	double last_t = ros::Time::now().toSec();

	while(ros::ok()) {
		double this_t = ros::Time::now().toSec();
		
		if(time_max_ > 0.0 ) {
			double dt = this_t - last_t;
			double next_time = time_ + dt*realtime_factor_;
			// set new time value
			if(next_time > time_max_) {
				time_ = time_min_;
				reset_ = true;
			}
			else if(next_time < time_min_) {
				time_ = time_min_;
				reset_ = true;
			}
			else {
				time_ = next_time;
			}
			// publish time value
			time_msg.data = time_;
			tick.publish(time_msg);
		}
		
		last_t = this_t;
		// sleep to achieve rate of 10Hz
		r.sleep();
		if(!is_running_) break;
	}
}

void TFRepublisher::loop()
{
	ros::Rate r(frequency_);
	while(ros::ok()) {
		if(time_>0.0) {
			advance_cursor();
		}
		r.sleep();
		if(!is_running_) break;
	}
}

void TFRepublisher::set_goal(double time_min, double time_max)
{
	time_min_ = time_min;
	time_max_ = time_max;
	time_ = time_min_;
	has_next_ = false;
	has_new_goal_ = true;
}

void TFRepublisher::set_progress(double percent)
{
	time_ = time_min_ + percent*(time_max_ - time_min_);
	has_been_skipped_ = true;
}

void TFRepublisher::set_now(double time)
{
	time_ = time_min_;
	has_been_skipped_ = true;
}

void TFRepublisher::create_cursor(double start_time)
{
	// ascending order
	bson_t *opts = BCON_NEW(
		"sort", "{", "header.stamp", BCON_INT32 (1), "}"
	);
	// filter documents outside of time interval
	bson_t *filter = BCON_NEW(
		"header.stamp", "{",
			"$gt", BCON_DATE_TIME((unsigned long long)(1000.0*start_time)),
			"$lt", BCON_DATE_TIME((unsigned long long)(1000.0*time_max_)),
		"}"
	);
	// get the cursor
	if(cursor_!=NULL) {
		mongoc_cursor_destroy(cursor_);
	}
	if(collection_) {
		delete collection_;
	}
	collection_ = MongoInterface::get_collection(
		db_name_.c_str(),db_collection_.c_str());
	collection_->appendSession(opts);
	cursor_ = mongoc_collection_find_with_opts(
	    (*collection_)(), filter, opts, NULL /* read_prefs */ );
	// cleanup
	if(filter) {
		bson_destroy(filter);
	}
	if(opts) {
		bson_destroy(opts);
	}
}

void TFRepublisher::set_initial_poses(double unix_time)
{
	unsigned long long mng_time = (unsigned long long)(1000.0*unix_time);
	bson_t *append_opts = bson_new();
	// lookup latest transform of each frame before given time, if any
	bson_t *pipeline = BCON_NEW ("pipeline", "[",
		"{", "$group", "{", "_id", "{", "child_frame_id", BCON_UTF8("$child_frame_id"), "}", "}", "}",
		"{", "$lookup", "{",
			"from", BCON_UTF8(db_collection_.c_str()),
			"as",   BCON_UTF8("tf"),
			"let", "{", "frame", BCON_UTF8("$_id.child_frame_id"), "}",
			"pipeline", "[",
				"{", "$match", "{", "$expr", "{", "$and", "[",
					"{", "$eq", "[", BCON_UTF8("$child_frame_id"), BCON_UTF8("$$frame"),     "]", "}",
					"{", "$lt", "[", BCON_UTF8("$header.stamp"),   BCON_DATE_TIME(mng_time), "]", "}",
				"]", "}", "}", "}",
				"{", "$sort", "{",
					"child_frame_id", BCON_INT32(1),
					"header.stamp",   BCON_INT32(-1),
				"}", "}",
				"{", "$limit", BCON_INT32(1), "}",
			"]",
		"}", "}",
		"{", "$unwind", BCON_UTF8("$tf"), "}",
		"{", "$replaceRoot", "{", "newRoot", BCON_UTF8("$tf"), "}", "}",
	"]");
	// create the cursor
	MongoCollection *collection = MongoInterface::get_collection(
			db_name_.c_str(), db_collection_.c_str());
	collection->appendSession(append_opts);
	mongoc_cursor_t *cursor = mongoc_collection_aggregate(
		(*collection)(), MONGOC_QUERY_NONE, pipeline, NULL, NULL);
	memory_.clear_transforms_only();
	if(cursor!=NULL) {
		const bson_t *doc;
		// iterate the cursor and assign poses in TF memory
		while(mongoc_cursor_next(cursor,&doc)) {
			read_transform(doc);
			memory_.set_transform(ts_);
		}
		mongoc_cursor_destroy(cursor);
	}
	// cleanup
	if(pipeline) {
		bson_destroy(pipeline);
	}
	if(append_opts) {
		bson_destroy(append_opts);
	}
	if(collection) {
		delete collection;
	}
}

void TFRepublisher::reset_cursor()
{
	if(cursor_!=NULL) {
		mongoc_cursor_t *clone = mongoc_cursor_clone(cursor_);
		mongoc_cursor_destroy(cursor_);
		cursor_ = clone;
	}
}

void TFRepublisher::advance_cursor()
{
	double this_time = time_;
	if(has_new_goal_) {
		has_new_goal_ = false;
		has_next_ = false;
		set_initial_poses(time_min_);
		create_cursor(time_min_);
	}
	else if(has_been_skipped_) {
		has_been_skipped_ = false;
		has_next_ = false;
		// special reset mode because cursor needs to be recreated with proper start time
		skip_reset_ = true;
		// load initial poses
		set_initial_poses(this_time);
		// create cursor with documents from this_time to time_max_
		create_cursor(this_time);
	}
	if(reset_) {
		reset_ = false;
		has_next_ = false;
		// load initial poses to avoid problems with objects sticking at the position
		// where they were at the end of the loop.
		set_initial_poses(time_min_);
		if(skip_reset_) {
			skip_reset_ = false;
			create_cursor(time_min_);
		}
		else {
			reset_cursor();
		}
	}
	//
	while(1) {
		// check if the cursor has an error.
		// if this is the case, reset next loop.
		bson_error_t cursor_error;
		if (mongoc_cursor_error (cursor_, &cursor_error)) {
			ROS_ERROR("[TFRepublisher] mongo cursor error: %s. Resetting..", cursor_error.message);
			reset_ = true;
			break;
		}

		if(has_next_) {
			double t_next = (ts_.header.stamp.sec * 1000.0 +
					ts_.header.stamp.nsec / 1000000.0) / 1000.0;
			if(t_next > this_time) {
				// the next transform is too far in the future
				break;
			}
			// push the next transform
#if CLEAR_MEMORY_AFTER_PUBLISH
			memory_.set_managed_transform(ts_);
#else
			memory_.set_transform(ts_);
#endif
		}
		// read the next transform
		const bson_t *doc;
		if(cursor_!=NULL && mongoc_cursor_next(cursor_,&doc)) {
			read_transform(doc);
			has_next_ = true;
		}
		else {
			has_next_ = false;
			break;
		}
	}
}

void TFRepublisher::read_transform(const bson_t *doc)
{
	bson_iter_t iter;
	if(!bson_iter_init(&iter,doc)) {
		return;
	}

	while(bson_iter_next(&iter)) {
		const char *key = bson_iter_key(&iter);

		if(strcmp("child_frame_id",key)==0) {
			ts_.child_frame_id = std::string(bson_iter_utf8(&iter,NULL));
		}

		else if(strcmp("header",key)==0) {
			bson_iter_t header_iter;
			bson_iter_recurse(&iter, &header_iter);
			while(bson_iter_next(&header_iter)) {
				const char *header_key = bson_iter_key(&header_iter);
				if(strcmp("seq",header_key)==0) {
					ts_.header.seq = bson_iter_int32(&header_iter);
				}
				else if(strcmp("frame_id",header_key)==0) {
					ts_.header.frame_id = std::string(bson_iter_utf8(&header_iter,NULL));
				}
				else if(strcmp("stamp",header_key)==0) {
					int64_t msec_since_epoch = bson_iter_date_time(&header_iter);
					ts_.header.stamp.sec  = msec_since_epoch / 1000;
					ts_.header.stamp.nsec = (msec_since_epoch % 1000) * 1000 * 1000;
				}
			}
		}

		else if(strcmp("transform",key)==0) {
			bson_iter_t transform_iter;
			bson_iter_recurse(&iter, &transform_iter);
			while(bson_iter_next(&transform_iter)) {
				const char *transform_key = bson_iter_key(&transform_iter);
				//
				bson_iter_t iter1;
				bson_iter_recurse(&transform_iter, &iter1);

				if(strcmp("translation",transform_key)==0) {
					while(bson_iter_next(&iter1)) {
						const char *key1 = bson_iter_key(&iter1);
						if(strcmp("x",key1)==0) {
							ts_.transform.translation.x = bson_iter_double(&iter1);
						}
						else if(strcmp("y",key1)==0) {
							ts_.transform.translation.y = bson_iter_double(&iter1);
						}
						else if(strcmp("z",key1)==0) {
							ts_.transform.translation.z = bson_iter_double(&iter1);
						}
					}
				}

				else if(strcmp("rotation",transform_key)==0) {
					while(bson_iter_next(&iter1)) {
						const char *key1 = bson_iter_key(&iter1);
						if(strcmp("x",key1)==0) {
							ts_.transform.rotation.x = bson_iter_double(&iter1);
						}
						else if(strcmp("y",key1)==0) {
							ts_.transform.rotation.y = bson_iter_double(&iter1);
						}
						else if(strcmp("z",key1)==0) {
							ts_.transform.rotation.z = bson_iter_double(&iter1);
						}
						else if(strcmp("w",key1)==0) {
							ts_.transform.rotation.w = bson_iter_double(&iter1);
						}
					}
				}
			}
		}
	}
}

