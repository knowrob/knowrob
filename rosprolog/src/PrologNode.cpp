
#include <ros/ros.h>
#include <ros/console.h>
#include <ros/package.h>

#include <rosprolog/PrologNode.h>

#define PARAM_INITIAL_PACKAGE "initial_package"
#define PARAM_INITIAL_GOAL "initial_goal"
#define PARAM_NUM_PL_THREADS "num_pl_threads"
#define PARAM_NUM_ROS_THREADS "num_ros_threads"

#define NUM_PL_THREADS_DEFAULT 2
#define NUM_ROS_THREADS_DEFAULT 2

PrologNode::PrologNode(ros::NodeHandle *node)
	: thread_pool_(PrologNode::num_pl_threads(node)),
	  is_initialized_(false)
{
	if(!ensure_loaded("rosprolog")) {
		return;
	}
	std::string param;
	// register initial packages
	if (node->getParam(PARAM_INITIAL_PACKAGE, param)) {
		if(!call1("register_ros_package",param)) {
			ROS_ERROR("Failed to load initial_package %s.", param.c_str());
			return;
		}
	}
	else if(!call1("register_ros_package","knowrob_common")) {
		ROS_ERROR("Failed to load knowrob_common.");
		return;
	}
	// execute initial goal
	{
		boost::shared_ptr<PrologEngine> engine = thread_pool_.claim();
		if (node->getParam(PARAM_INITIAL_GOAL, param)) {
			engine->one_solution(param);
		}
		thread_pool_.release(engine);
	}
	is_initialized_ = true;
}

int PrologNode::num_pl_threads(ros::NodeHandle *node)
{
	int count=0;
	if(node->getParam(PARAM_NUM_PL_THREADS, count)) {
		return count;
	}
	return NUM_PL_THREADS_DEFAULT;
}

int PrologNode::call1(const std::string &p, const std::string &arg1)
{
	term_t a1 = PL_new_term_refs(1); {
		PL_put_atom_chars(a1, arg1.c_str());
	}
	qid_t qid = PL_open_query(NULL, PL_Q_NODEBUG,
	    PL_pred(PL_new_functor(PL_new_atom(p.c_str()),1), NULL),
	    a1);
	bool status = TRUE;
	
	if(!PL_next_solution(qid)) {
		std::string error_msg;
		if(PrologEngine::pl_exception(qid,error_msg)) {
			ROS_ERROR("%s", error_msg.c_str());
		}
		status = FALSE;
	}
	// cleanup
	PL_close_query(qid);
	PL_reset_term_refs(a1);
	
	return status;
}

int PrologNode::ensure_loaded(const char *ros_pkg)
{
	std::stringstream ss;
	ss << ros::package::getPath(ros_pkg) << "/prolog/init.pl";
	if(!call1("ensure_loaded",ss.str())) {
		ROS_ERROR("Failed to load init.pl of %s.", ros_pkg);
		return FALSE;
	}
	return TRUE;
}

bool PrologNode::exists(const std::string &id)
{
	return claimed_engines_.find(id) != claimed_engines_.end();
}

bool PrologNode::has_more_solutions(const std::string &id)
{
	return claimed_engines_.find(id)->second->has_more_solutions();
}

void PrologNode::finish(const std::string &id)
{
	auto it = claimed_engines_.find(id);
	it->second->release(true);
	thread_pool_.release(it->second);
	claimed_engines_.erase(it);
}

void PrologNode::finish()
{
	auto it = claimed_engines_.begin();
	while (it != claimed_engines_.end()) {
		finish(it->first);
		it = claimed_engines_.begin();
	}
}

bool PrologNode::query(json_prolog_msgs::PrologQuery::Request &req,
			   json_prolog_msgs::PrologQuery::Response &res)
{
	if (exists(req.id)) {
		std::stringstream ss;
		ss << "Another query is already being processed with id " << req.id << ".";
		res.ok = false;
		res.message = ss.str();
	} else {
		boost::shared_ptr<PrologEngine> engine = thread_pool_.claim();
		engine->claim(req.query,
			(req.mode == json_prolog_msgs::PrologQuery::Request::INCREMENTAL));
		claimed_engines_[req.id] = engine;
		res.ok = true;
		res.message = "";
	}
	return true;
}

bool PrologNode::finish(json_prolog_msgs::PrologFinish::Request &req,
			    json_prolog_msgs::PrologFinish::Response &res)
{
	if (req.id == "*"){
		// finish all queries
		finish();
	} else {
		finish(req.id);
	}
	return true;
}

bool PrologNode::next_solution(json_prolog_msgs::PrologNextSolution::Request &req,
				   json_prolog_msgs::PrologNextSolution::Response &res)
{
	if(!exists(req.id)) {
		res.status = json_prolog_msgs::PrologNextSolution::Response::WRONG_ID;
		res.solution = "";
	}
	else {
		if (!has_more_solutions(req.id)){
			boost::shared_ptr<PrologEngine> x = claimed_engines_.find(req.id)->second;
			if(x->has_error()) {
				res.status = json_prolog_msgs::PrologNextSolution::Response::QUERY_FAILED;
				res.solution = x->error();
				ROS_WARN("%s.", res.solution.c_str());
			}
			else {
				res.status = json_prolog_msgs::PrologNextSolution::Response::NO_SOLUTION;
				res.solution = "";
			}
		}
		else {
			res.status = json_prolog_msgs::PrologNextSolution::Response::OK;
			boost::shared_ptr<std::string> solution =
				claimed_engines_.find(req.id)->second->next_solution();
			res.solution = solution->c_str();
		}
	}
	return true;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "rosprolog");
	ros::NodeHandle n;
	// rosprolog can serve requests in parallel
	int num_ros_threads=0;
	if(!n.getParam(PARAM_NUM_ROS_THREADS, num_ros_threads)) {
		num_ros_threads = NUM_ROS_THREADS_DEFAULT;
	}
	ros::AsyncSpinner spinner(num_ros_threads);
	// loop at 10Hz
	ros::Rate loop_rate(10.0); //Hz
	spinner.start();
	// initialize Prolog
	int pl_ac = 0;
	char *pl_av[5]; {
		pl_av[pl_ac++] = argv[0];
		// '-g true' is used to suppress the welcome message
		pl_av[pl_ac++] = (char *) "-g";
		pl_av[pl_ac++] = (char *) "true";
		// Inhibit any signal handling by Prolog
		// pl_av[pl_ac++] = (char *) "-nosignals";
                pl_av[pl_ac++] = (char *) "--signals=false";
		// Limit the combined size of the Prolog stacks to the indicated size.
		//pl_av[pl_ac++] = (char *) "--stack_limit=32g";
		// Limit for the table space.
		// This is where tries holding memoized11 answers for tabling are stored.
		//pl_av[pl_ac++] = (char *) "--table_space=32g";
		//pl_av[pl_ac++] = (char *) "-G256M";
		pl_av[pl_ac] = NULL;
	}
	PL_initialise(pl_ac, pl_av);
	//
	PrologNode rosprolog(&n);
	if(rosprolog.is_initialized()) {
		ros::ServiceServer service_query = n.advertiseService(
		    "/rosprolog/query", &PrologNode::query, &rosprolog);
		ros::ServiceServer service_next_solution = n.advertiseService(
		    "/rosprolog/next_solution", &PrologNode::next_solution, &rosprolog);
		ros::ServiceServer service_finish = n.advertiseService(
		    "/rosprolog/finish", &PrologNode::finish, &rosprolog);
		
		ROS_INFO("rosprolog service is running.");
		while (ros::ok()) {
			ros::spinOnce();
			loop_rate.sleep();
		}
		ROS_INFO("rosprolog service is exiting.");
		
		return EXIT_SUCCESS;
	}
	else {
		ROS_ERROR("rosprolog service failed to initialize.");
		return EXIT_FAILURE;
	}
}
