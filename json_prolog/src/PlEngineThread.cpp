
class PlEngineThread {
public:
	PlEngineThread();
	
	bool exists(const string &id);
	
	void submit(const string &id, const string &query, int mode);
	void finish(const string &id);
	
	bool has_more_solutions(const sring &query_id);
	const PlQuerySolution& next_solution(const sring &query_id);
private:
	typedef std::shared_ptr<PlEngine> PlEnginePtr;
	typedef std::unordered_map<std::string,PrologQuery> QueryMap;
	typedef std::unordered_map<std::string,QueryData> QueryDataMap;
	PlEnginePtr engine;
	QueryMap queries;
	QueryDataMap new_queries[2];
	
	void run();
	void init_pl();
	
	const PlQuerySolution& receive_next_solution(const string &id);
}

void PlEngineThread::run() {
	init_pl();
	while(ros::ok()) {
		index = update_index;
		// ping pong, less locking
		XXX.lock();
		update_index = (int)!update_index;
		XXX.unlock();
		// start new queries
		for(QueryDataMap::iterator
					it=new_queries[index].begin();
					it!=new_queries[index].end();
					it=it->next())
		{
			QueryData &d = it->second;
			QueryMap::iterator jt = queries.insert({*it,
				PrologQuery(d.query_string,d.mode)});
			jt->second.start();
		}
		new_queries[index].clear();
		// send notifications for new solutions
		for(QueryMap::iterator it=queries.begin(); it!=queries.end(); it=it->next()) {
			PrologQuery &query = it->second;
			if(query.has_more_solutions(PrologQuery.NON_BLOCKING)) {
				query.notify_solution();
			}
			// TODO: maybe solution is forgotton like this
			//if(query.has_finished()) {
			//	delete_queries.add(*it);
			//}
		}
		// delete finished queries
// 		for(std::set<std::string>::iterator it=delete_queries.begin();
// 											it!=delete_queries.end();
// 											it=it->next()) {
// 			queries.erase(*it);
// 		}
	}
}

void PlEngineThread::init_pl() {
	char *argv[4];
	int argc = 0;
	argv[argc++] = (char *) "PrologEngine";
	argv[argc++] = (char *) "-f";
	std::string rosPrologInit = ros::package::getPath("rosprolog") + "/prolog/init.pl";
	argv[argc] = new char[rosPrologInit.size() + 1];
	std::copy(rosPrologInit.begin(), rosPrologInit.end(), argv[argc]);
	argv[argc++][rosPrologInit.size()] = '\0';
	argv[argc] = NULL;
	engine = std::make_shared<PlEngine>(argc, argv);
}

void PlEngineThread::submit(const string &id, const string &query_string, int mode) {
	//has_queries_to_process = true;
	//cv_loop.notify_one();
	XXX.lock();
	new_queries[update_index].add({
		id,
		QueryData(query_string,mode)
	});
	XXX.unlock();
}

void PlEngineThread::finish(const string &id) {
	PrologQuery *query = get(*it);
	if(query!=null) {
		query->finish();
	}
	// TODO also remove from new_queries
}

bool PlEngineThread::has_more_solutions(const string &id) {
	// TODO new_queries, no new solution
	PrologQuery *query = get(*it);
	if(query!=null) {
		query->wait_solution();
	}
	else {
		return false;
	}
}

const PlQuerySolution& PlEngineThread::next_solution(const string &id) {
	if(has_more_solutions(id)) {
		return pop_next_solution(id);
	}
	else {
		return null;
	}
}

const PlQuerySolution& PlEngineThread::pop_next_solution(const string &id) {
	PrologQuery *query = get(*it);
	return query->pop_next_solution();
}
