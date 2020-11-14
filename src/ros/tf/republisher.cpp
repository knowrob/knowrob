#include <knowrob/ros/tf/republisher.h>
#include <knowrob/db/mongo/MongoInterface.h>

// FIXME
static MongoCollection *collection=NULL;

TFRepublisher::TFRepublisher(double frequency) :
		realtime_factor_(1.0),
		frequency_(frequency),
		loop_(true),
		has_next_(false),
		is_running_(true),
		db_name_("neems"),
		db_collection_("tf"),
		time_min_(0.0),
		time_max_(0.0),
		time_(0.0),
		cursor_(NULL),
		memory_(),
		publisher_(memory_,frequency),
	    thread_(&TFRepublisher::loop, this)
{
}

TFRepublisher::~TFRepublisher()
{
	is_running_ = false;
	thread_.join();
	if(collection) {
		delete collection;
		collection = NULL;
	}
}

void TFRepublisher::loop()
{
	ros::Rate r(frequency_);
	double last_t = ros::Time::now().toSec();
	while(ros::ok()) {
		double this_t = ros::Time::now().toSec();
		advance_cursor(this_t-last_t);
		last_t = this_t;
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
	create_cursor();
}

void TFRepublisher::create_cursor()
{
	if(cursor_!=NULL) {
		mongoc_cursor_destroy(cursor_);
		cursor_ = NULL;
	}
	// ascending order
	bson_t *opts = BCON_NEW(
		"sort", "{", "header.stamp", BCON_INT32 (1), "}"
	);
	// filter documents outside of time interval
	bson_t *filter = BCON_NEW(
		"header.stamp", "{", "$gt", BCON_DATE_TIME((unsigned long long)(1000.0*time_min_)), "}",
		"header.stamp", "{", "$lt", BCON_DATE_TIME((unsigned long long)(1000.0*time_max_)), "}"
	);
	// get the cursor
	if(collection) {
		delete collection;
	}
	collection = MongoInterface::get_collection(
			db_name_.c_str(),db_collection_.c_str());
	cursor_ = mongoc_collection_find_with_opts(
	    (*collection)(), filter, opts, NULL /* read_prefs */ );
}

void TFRepublisher::reset_cursor()
{
	if(cursor_!=NULL) {
		mongoc_cursor_t *clone = mongoc_cursor_clone(cursor_);
		mongoc_cursor_destroy(cursor_);
		cursor_ = clone;
	}
}

void TFRepublisher::advance_cursor(double dt)
{
	// advance time
	time_ += dt*realtime_factor_;
	if(time_ > time_max_) {
		if(loop_) {
			time_ = time_min_;
			has_next_ = false;
			reset_cursor();
		}
		else {
			return;
		}
	}
	// push transforms from beginning of cursor until
	// one is reached with stamp>time_
	do {
		if(has_next_) {
			double t_next = (ts_.header.stamp.sec * 1000.0 +
					ts_.header.stamp.nsec / 1000000.0) / 1000.0;
			if(t_next > time_) {
				// the next transform is too far in the future
				break;
			}
			// push the next transform
			memory_.set_transform(ts_);
		}
		// read the next transform
		const bson_t *doc;
		if(cursor_!=NULL && mongoc_cursor_next(cursor_,&doc)) {
			read_transform(doc);
			has_next_ = true;
		}
		else {
			has_next_ = false;
		}
	} while(has_next_);
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

