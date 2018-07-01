
class PrologQuery {
public:
	NON_BLOCKING=0;
	BLOCKING=1;
}

PrologQuery::PrologQuery(const std::string &query_string, int mode) :
		query_string(query_string),
		thread_id(""),
		mode(mode)
{
}

const std::string& PrologQuery::start() {
	PlTermv av(2);
	av[0] = query_string.c_str();
	try {
		PlQuery q("queryt_create", av);
		if(q.next_solution()) thread_id = std::string((char*)av[1]));
	}
	catch (PlException &ex) {
		ROS_WARN((char *) ex);
	}
	return thread_id;
}

// TODO: non blocking / blocking versions?

bool PrologQuery::has_next() {
	PlTermv av(1);
	av[0] = thread_id.c_str(); 
	try {
		PlQuery q("queryt_has_next", av);
		return q.next_solution();
	}
	catch (PlException &ex) {
		ROS_WARN((char *) ex);
	}
	return false;
}

XXX PrologQuery::next_solution() {
	PlTermv av(2);
	av[0] = thread_id.c_str(); 
	try {
		PlQuery q("queryt_next_solution", av);
		// av[1] is a list [string var_name, var_value]
		// TODO
		return XXX;
	}
	catch (PlException &ex) {
		ROS_WARN((char *) ex);
	}
	return XXX;
}
