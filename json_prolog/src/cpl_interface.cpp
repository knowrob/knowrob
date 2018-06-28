#include "ros/ros.h"
#include "json_prolog_msgs/PrologQuery.h"
#include "json_prolog_msgs/PrologFinish.h"
#include "json_prolog_msgs/PrologNextSolution.h"
#include "cpl_interface.h"
#include "ros/package.h"
#include "time.h"
#include <thread>
//#include <vector>
#include <unordered_map>
#include <mutex>
#include <condition_variable>

/*
 * the data structure of our choice are unordered maps to keep track of (a) incoming queries and
 * (b) queries that are already being processed but need to be kept alive to generate further solutions
 */
std::unordered_map <std::string, PrologQuery> queries;
std::unordered_map <std::string, PrologQuery> processed_queries;

// shared mutex, used by threads from the callback fctn
std::mutex push_lock;

// global reference to the prolog interface instance
PrologInterface *plIfaceGlobal = NULL;

/*
 * Create a query from the given query string and push it to the list of queries, where it will be handled by the
 * worker loop.
 */
bool query(json_prolog_msgs::PrologQuery::Request &req,
           json_prolog_msgs::PrologQuery::Response &res) {

    // id already exists in queries
    if (queries.count(req.id) > 0 || processed_queries.count(req.id) > 0) {
        res.ok = false;
        res.message = "Another query is already being processed with id " + req.id;
        return true;
    } else {
        int mode = req.mode;
        std::string id = req.id;
        std::string query_string = req.query;

        plIfaceGlobal->PrologInterface::push_query(mode, id, query_string, false);

        res.ok = true;
        res.message = "";

        return true;
    }
}

/*
 * Destroy a query if it is no longer needed. This is the case if no further solutions need to be generated for that query.
 */
bool finish(json_prolog_msgs::PrologFinish::Request &req,
            json_prolog_msgs::PrologFinish::Response &res) {
    //res.sum = req.a + req.b;
//  ROS_INFO("request: x=%ld, y=%ld", (long int)req.a, (long int)req.b);
//  ROS_INFO("sending back response: [%ld]", (long int)res.sum);
    return true;
}

/*
 * Calculate a next solution for a query that has been invoked beforehand by the callback function query.
 */
bool next_solution(json_prolog_msgs::PrologNextSolution::Request &req,
                   json_prolog_msgs::PrologNextSolution::Response &res) {

    /*
     * no id entered?
     */
    if (req.id == "") {
        res.status = 1; // 1 = wrong id
        res.solution = "";
        return true;
    }

    /*
     * no query for id?
     */
    std::unordered_map<std::string, PrologQuery>::iterator iterator;
    iterator = processed_queries.find(req.id);

    if (iterator == processed_queries.end()) {
        res.status = 1; // 1 = wrong id
        res.solution = "";
        return true;
    }

    /*
     * id seems to be fine, get the next solution
     */
    PrologQuery query = plIfaceGlobal->PrologInterface::pop_query(req.id); // pop the already existing query
    plIfaceGlobal->PrologInterface::push_query(query.get_mode(), query.get_id(), query.get_query(),
                                               true); // push back the updated query

    // debug print
    std::cout << "The ID of the pl thread is: " + query.get_pl_thread_id() << std::endl;

    /*
     * this needs to be done after the query has been picked from the worker loop
     */
//    std::string next_solution;
//    bool has_next_solution = has_next_solution();
//
//    if (has_next_solution) {
//        next_solution = get_next_solution(thread_id);
//    }

//    else {
//        res.status = 0; // 0 = no solution
//        res.solution = "";
//        return true;
//    }

    res.status = 3; // 3 = ok
    res.solution = "";
    return true;
}

/*
 * Make a call to prolog that is started in a new prolog thread, using
 * the threaded_query.pl interface. The prolog thread remains until it is closed properly and can be referenced by its ID.
 *
 * @param input: a string that contains the query that is executed by the prolog engine.
 * @param engine: the prolog engine that is handling everything related to prolog.
 */
std::string pl_threaded_call(std::shared_ptr <PlEngine> engine, std::string input) {

    /*
     * The prolog term, the prolog engine will try to unify,
     * av[1] yields the solution to the imposed query
     *
     * These queries usually look like this: "queryt_create('<input>', Id)."
     */
    PlFrame fr;
    PlTermv av(2);
    av[0] = input.c_str(); //PlString(query_string.c_str());
    //PLTerm foo;
    //av[1] = foo;
//    av[1] = "Id";
//    av[0] = PlCompound(query_string.c_str());

    std::string thread_id;

    try {
        PlQuery q("queryt_create", av);

        while (q.next_solution())
            thread_id.assign(av[1]);
    }
    catch (PlException &ex) {
        ROS_INFO((char *) ex);
    }
    return thread_id;
}

/*
 * Take all the information needed for a query and create a query object.
 * Push it to the map of queries, so that the worker loop will eventually pass the query to the prolog engine.
 *
 * The push is synchronized in order to prevent undefined behaviour from the different threads.
 */
void PrologInterface::push_query(int mode, std::string id, std::string query, bool request_next_solution) {

    // build a query from the requirements
    PrologQuery queryObj;
    queryObj.set_mode(mode);
    queryObj.set_id(id);
    queryObj.set_pl_thread_id("");
    queryObj.set_query(query);
    queryObj.set_message("");
    queryObj.set_solution("");
    queryObj.set_ok(true);
    queryObj.set_request_next_solution(
            request_next_solution); // this is important bc that's how the program knows a next solution is requested

    push_lock.lock();
    queries.insert({id, queryObj}); // synchronized push of the query to the shared map of queries
    push_lock.unlock();

    has_queries_to_process = true;
    cv_loop.notify_one();
    ROS_INFO("PUSH QUERY PASSED");
}

/*
 * Pops a query from either the map for processed or unprocessed queries.
 * This is needed when a next solution for an imposed query is requested. The query object is then popped (mostly from processed queries)
 * modified and pushed back to queries, where the worker loop will reprocess the object eventually.
 *
 * Popping is also synchronized to prevent changes to a query while it is transfered from unprocessed queries to queries.
 */
PrologQuery PrologInterface::pop_query(std::string id) {

    std::unordered_map<std::string, PrologQuery>::iterator iterator;
    PrologQuery popped_query;

    /*
     * check processed_queries for a matching instance
     */
    iterator = processed_queries.find(id);
    if (!(iterator == processed_queries.end())) {
        push_lock.lock();
        popped_query = processed_queries[id];
        processed_queries.erase(iterator);
        push_lock.unlock();
    }
        /*
         * maybe the matching query hasn't been transferred to processing_queries yet,
         * so we check the unprocessed queries as well
         */
    else {
        iterator = queries.find(id);
        if (!(iterator == queries.end())) {
            push_lock.lock();
            popped_query = queries[id];
            queries.erase(iterator);
            push_lock.unlock();
        }
    }
    return popped_query;
}

/*
 * Initialize the knowrob_common package, so that all the prolog predicates are available.
 */
void PrologInterface::init() {

    PlTerm av("knowrob_common");
    try {
        PlCall("register_ros_package", av);
    }
    catch (PlException &ex) {
        ROS_INFO((char *) ex);
    }
}

/*
 * Calculate the next solution to an imposed query.
 *
 * @status: under construction.
 */
std::string pl_next_solution(std::shared_ptr <PlEngine> engine, std::string thread_id) {


    // Check if more solutions are available
    std::string next_solution;

    std::cout << "pl_next_solution reached" << std::endl;
    PlFrame fr;
    PlTermv av(2);
    av[0] = thread_id.c_str(); //PlCompound(thread_id.c_str());

//    PlTerm av(PlCompound(thread_id.c_str()));

    try {
        std::cout << "PL_Thread_ID for next solution: " << (char *) av[0] << std::endl;
        PlQuery q("queryt_next_solution", av);

        if (!(q.next_solution())) {
            return "";
        } else {


            //PlTail solution_list(av[1]);
            //json_parse_list(solution_list, json_object);
            //print_json_object(json_object);
            // {
            //   'A': JSON_LIST([1,2,3]
            //   'B': ....
            // }


            std::cout << "foo1: " << std::endl;  // 'A'
            std::cout << "foo1: " << (char *) av[1] << std::endl;  // 'A'
            PlTail solution_list(av[1]);
            std::cout << "foo2: " << std::endl;  // 'A'
            PlTerm assignment_term;
            solution_list.next(assignment_term);
            std::cout << "foo3: " << (char *) assignment_term << std::endl;  // 'A'
            solution_list.close();

            PlTail assignment(assignment_term);
            PlTerm name_term, value_term;
            assignment.next(name_term);
            std::cout << "term0: " << (char *) name_term << std::endl;  // 'A'
            assignment.next(value_term);
            std::cout << "term1: " << (char *) value_term << std::endl; // [1,2,3]

            PlTail test_value_list(value_term);
            PlTerm list_value_term;
            test_value_list.next(list_value_term);
            std::cout << "term1.1: " << (char *) test_value_list << std::endl;  // 'A'
            std::cout << "term1.2: " << (char *) list_value_term << std::endl;  // 'A'

            assignment.close();

            //std::cout << (char *) av[1] << std::endl;
            // next_solution.assign(av[1]);
        }
        std::cout << "DONE: " << std::endl;
    }
    catch (PlException &ex) {
        ROS_INFO((char *) ex);
    }

    ROS_INFO(next_solution.c_str());
    return next_solution;
}

/*
 * Check if there are further solutions for a given query. Reference via prolog thread id.
 */
bool pl_has_next_solution(std::shared_ptr <PlEngine> engine, std::string thread_id) {

    bool has_next = false;

    std::cout << "pl_has_next_solution reached" << std::endl;
    PlFrame fr;
    PlTermv av(2);
    av[0] = thread_id.c_str(); //PlCompound(thread_id.c_str());

//    PlTerm av(PlCompound(thread_id.c_str()));

    try {
        std::cout << "av[1] in pl_has_next_solution: " << (char *) av[1] << std::endl;
        PlQuery q("queryt_has_next", av);

        std::cout << q.next_solution() << std::endl;
//        while (q.next_solution()) {
//        }

        // next_solution.assign(av[1]);
    }

    catch (PlException &ex) {
        ROS_INFO((char *) ex);
    }
    return has_next;
}

/*
 * A worker thread that is notified when there are queries available.
 * Process all the queries available in queries and goes back to sleep until it is notified again.
 *
 * @bug: cannot be killed using ctrl+c, use pkill instead
 */
void PrologInterface::loop() {

    ROS_INFO("Invoking prolog engine");
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
    init();

    std::unordered_map<std::string, PrologQuery>::iterator iterator;
    std::string debug_msg = "";
    while (ros::ok()) {
        {
            std::unique_lock <std::mutex> lk(loop_lock);
            while (!queries.empty()) { // queries available?

                iterator = queries.begin();
                std::string query_string(iterator->second.get_query());

                /*
                 *  is the current query new (not a next solution request for an imposed query)?
                 */
                if (!(iterator->second.get_request_next_solution())) {

                    std::string rosinfo = "Received query: " + query_string;
                    ROS_INFO(rosinfo.c_str());
                    std::string thread_id = pl_threaded_call(engine, query_string);
                    push_lock.lock();
                    iterator->second.set_pl_thread_id(thread_id);
                    push_lock.unlock();
                    ROS_INFO(thread_id.c_str());
                }
                    /*
                     * is the current query a next solution request for an imposed query?
                     */
                else {
                    std::string thread_id = iterator->second.get_pl_thread_id();

                    // are there any more solutions for this query?
                    ROS_INFO(thread_id.c_str());
                    if (!(pl_has_next_solution(engine, thread_id))) {
                        std::cout << "There are no more solutions for this query." << std::endl;
                    } else {
                        pl_next_solution(engine, thread_id);
                    }
                }

                push_lock.lock();
                processed_queries.insert({iterator->first, iterator->second});
                queries.erase(iterator); // remove query from the queued queries
                push_lock.unlock();

            }
            ROS_INFO("WAIT FOR QUERIES...");
            cv_loop.wait(lk, [this] { return has_queries_to_process; });
            ROS_INFO("WAKE UP - QUERIES AVAILABLE.");
            has_queries_to_process = false;
        }
    }
}

PrologInterface::PrologInterface() :
        thread(&PrologInterface::loop, this) {
}

/*
 * Manage the ros functions and create the prolog interface.
 */
int main(int argc, char **argv) {
    ros::init(argc, argv, "cpl_interface_node");
    ros::NodeHandle n;

    // create the thread handler, that checks for queries and passes them to prolog
    PrologInterface prologInterface;
    plIfaceGlobal = &prologInterface;

    ros::ServiceServer service_query = n.advertiseService("query", query);
    ros::ServiceServer service_next_solution = n.advertiseService("next_solution", next_solution);
    ros::ServiceServer service_finish = n.advertiseService("finish", finish);
    ros::spin();

    return 0;
}