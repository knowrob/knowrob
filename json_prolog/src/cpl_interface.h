#ifndef JSONPROLOGINTERFACE_H
#define JSONPROLOGINTERFACE_H

//SWI Prolog
#include <SWI-cpp.h>
//ros
#include <ros/package.h>
//STD
#include <memory>
#include <iostream>
#include <unordered_map>
#include <thread>
#include <mutex>
#include <condition_variable>

//wrapper class for Prolog Engine based on SWI-C++

class PrologQuery {
    int mode;
    std::string id, pl_thread_id, query, message, solution;
    bool ok, request_next_solution;


public:
    void set_mode(int p_mode) {mode = p_mode;}

    void set_id(std::string p_id) {id = p_id;}

    void set_pl_thread_id(std::string p_pl_thread_id) {pl_thread_id = p_pl_thread_id;}

    void set_query(std::string p_query) {query = p_query;}

    void set_message(std::string p_message) {message = p_message;}

    void set_solution(std::string p_solution) {solution = p_solution;}

    void set_ok(bool p_ok) {ok = p_ok;}

    void set_request_next_solution(bool p_request_next_solution) {request_next_solution = p_request_next_solution;}

    std::string &get_query() { return query;}

    std::string &get_id() {return id;}

    int get_mode () {return mode;}

    std::string &get_pl_thread_id() {return pl_thread_id;}

    std::string &get_message() { return message; }

    bool get_ok() { return ok; }

    bool get_request_next_solution() {return request_next_solution;}

    ~PrologQuery() {
    }
};

class PrologInterface {
private:
    typedef std::shared_ptr <PlEngine> PlEnginePtr;
    PlEnginePtr engine;
    std::mutex loop_lock;
    std::condition_variable cv_loop;
    std::thread thread;
    bool has_queries_to_process = false;


public:
    PrologInterface();

    ~PrologInterface() {
    }

    /*brief
     * initialize the necessary knowrob packages
     */
    void init();

    /*
     * main loop to process queries
     */
    void loop();

//    /*
//     * returns true if a given pl_thread_id has a next solution to offer
//     */
//    bool has_next_solution(std::string);

    /*
     * push a query to the map of queries
     */
    void push_query(int mode, std::string id, std::string query, bool request_next_solution);

    /*
     * pop a query from the map of queries
     */
    PrologQuery pop_query(std::string id);
};

#endif //JSONPROLOGINTERFACE_H
