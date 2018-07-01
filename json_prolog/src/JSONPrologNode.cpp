

class JSONPrologNode {
public:
	JSONPrologNode(ros::NodeHandle *node);
	bool query(
		json_prolog_msgs::PrologQuery::Request &req,
		json_prolog_msgs::PrologQuery::Response &res);
	bool finish(
		json_prolog_msgs::PrologFinish::Request &req,
		json_prolog_msgs::PrologFinish::Response &res);
	bool next_solution(
		json_prolog_msgs::PrologNextSolution::Request &req,
		json_prolog_msgs::PrologNextSolution::Response &res);
private:
	PlEngineThread pl_thread;
	
	bool encode_solution(
		const PlQuerySolution &pl_solution,
		string &json_encoded);
};

using namespace json_prolog_msgs;

JSONPrologNode::JSONPrologNode(ros::NodeHandle *node) {
}

bool JSONPrologNode::query(PrologQuery::Request &req,
						   PrologQuery::Response &res) {
	if (pl_thread.exists(req.id)) {
		res.ok = false;
		res.message = "Another query is already being processed with id " + req.id;
		return true;
	} else {
		pl_thread.submit(req.id, req.query, req.mode);
		res.ok = true;
		res.message = "";
		return true;
	}
}

bool JSONPrologNode::finish(PrologFinish::Request &req,
							PrologFinish::Response &res) {
	// finish all queries
	if (req.id == "*"){
		// TODO
	} else {
		pl_thread.finish(req.id);
	}
	return true;
}

bool JSONPrologNode::next_solution(PrologNextSolution::Request &req,
								   PrologNextSolution::Response &res) {
	res.solution = "";
	if (queries.count(req.id) == 0) {
		res.status = PrologNextSolutionResponse::WRONG_ID;
	}
	else {
		if (!pl_thread.has_more_solutions(req.id)){
			response.status = PrologNextSolutionResponse::NO_SOLUTION;
		}
		else {
			encode_solution(pl_thread.next_solution(req.id), res.solution);
			response.status = PrologNextSolutionResponse::OK;
		}
	}
    return true;
}

bool JSONPrologNode::encode_solution(
		const PlQuerySolution &pl_solution,
		string &json_encoded) {
	// TODO
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "json_prolog_cpp");
	ros::NodeHandle n;
	JSONPrologNode node(&n);
	ros::ServiceServer service_query =
		n.advertiseService("query", &JSONPrologNode::query, &node);
	ros::ServiceServer service_next_solution =
		n.advertiseService("next_solution", &JSONPrologNode::next_solution, &node);
	ros::ServiceServer service_finish =
		n.advertiseService("finish", &JSONPrologNode::finish, &node);
	ros::spin();
	return 0;
}
