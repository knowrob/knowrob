/*
 * Copyright (c) 2022, Daniel Beßler
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include <termios.h>
// STD
#include <exception>
#include <iostream>
// BOOST
#include <boost/program_options/options_description.hpp>
#include <boost/program_options/variables_map.hpp>
#include <boost/program_options/parsers.hpp>
// KnowRob
#include <knowrob/logging.h>
#include <knowrob/HybridQA.h>
#include <knowrob/queries.h>
#include <knowrob/Blackboard.h>
#include <knowrob/prolog/PrologQuery.h>
#include <knowrob/prolog/PrologReasoner.h>

using namespace knowrob;
namespace po = boost::program_options;

static const char* PROMPT = "?- ";

class HybridQATerminal : public QueryResultHandler {
public:
	HybridQATerminal(const po::variables_map &opts)
	: has_stop_request_(false),
	  currentQuery_(""),
	  cursor_(0)
	{
	}

	static char getch()
	{
		static struct termios old, current;
		static char buf = 0;
		// read termios settings
		tcgetattr(0, &old);
		// apply modified settings
		current = old;
		current.c_lflag &= ~ICANON; /* disable buffered i/o */
		current.c_lflag &= ~ECHO;   /* set no echo mode */
		tcsetattr(0, TCSANOW, &current);
		// get the next char
		if (read(0, &buf, 1) < 0)
			perror ("read()");
		// reset old settings
		tcsetattr(0, TCSANOW, &old);
		return buf;
	}

	// Override QueryResultHandler
	bool pushQueryResult(const QueryResultPtr &solution) override {
		if(solution->mapping().empty()) {
			std::cout << "yes." << std::endl;
		}
		else {
			std::cout << *solution << std::endl;
		}
		numSolutions_ += 1;
		return !has_stop_request_;
	}

	void runQuery(const std::string &queryString) {
		try {
			// parse query
			auto query = hybridQA_.parseQuery(queryString);
			// evaluate query in hybrid QA system
			numSolutions_ = 0;
			hybridQA_.runQuery(query, *this);
			if(numSolutions_ == 0) {
				std::cout << "no." << std::endl;
			}
		}
		catch (std::exception& e) {
			std::cout << e.what() << std::endl;
		}
	}

	void enter() {
		std::cout << std::endl;
		runQuery(currentQuery_);
		std::cout << std::endl << PROMPT << std::flush;
		currentQuery_.clear();
		cursor_ = 0;
	}

	void insert(char c) {
		if(cursor_ < currentQuery_.length()) {
			auto afterInsert = currentQuery_.substr(cursor_);
			std::cout << c << afterInsert <<
					  "\033[" << afterInsert.length() << "D" <<
					  std::flush;
			currentQuery_.insert(currentQuery_.begin() + cursor_, c);
		}
		else {
			std::cout << c << std::flush;
			currentQuery_ += c;
		}
		cursor_ += 1;
	}

	void backspace() {
		if(cursor_==0) {
			return;
		}
		else if(cursor_ < currentQuery_.length()) {
			auto afterDelete = currentQuery_.substr(cursor_);
			std::cout << '\b' << afterDelete << ' ' <<
					  "\033[" << (afterDelete.length()+1) << "D" << std::flush;
			currentQuery_.erase(cursor_-1, 1);
		}
		else {
			std::cout << '\b' << ' ' << '\b' << std::flush;
			currentQuery_.pop_back();
		}
		cursor_ -= 1;
	}

	void moveToBegin() {
		if(cursor_>0) {
			std::cout << "\033[" << cursor_ << "D" << std::flush;
			cursor_ = 0;
		}
	}

	void moveToEnd() {
		if(cursor_ < currentQuery_.length()) {
			std::cout << "\033[" << (currentQuery_.length() - cursor_) << "C" << std::flush;
			cursor_ = currentQuery_.length();
		}
	}

	void moveLeft() {
		if(cursor_>0) {
			std::cout << "\033[1D" << std::flush;
			cursor_ -= 1;
		}
	}

	void moveRight() {
		if(cursor_ < currentQuery_.length()) {
			std::cout << "\033[1C" << std::flush;
			cursor_ += 1;
		}
	}

	void moveUp() {
		// TODO: implement UP KEY
	}

	void moveDown() {
		// TODO: implement DOWN KEY
	}

	void handleEscapeSequence() {
		// the escape code \033 is followed by 2-3 more bytes
		if(getch() == '[') {
			switch(getch()) {
				case 'A': // "\033[A" --> UP ARROW
					moveUp();
					break;
				case 'B': // "\033[B" --> DOWN ARROW
					moveDown();
					break;
				case 'C': // "\033[C" --> RIGHT ARROW
					moveRight();
					break;
				case 'D': // "\033[D" --> LEFT_ARROW
					moveLeft();
					break;
				case 'F': // "\033[F" --> END
					moveToEnd();
					break;
				case 'H': // "\033[H" --> POS1
					moveToBegin();
					break;
			}
		}
	}

	int run() {
		std::cout << "Welcome to KnowRob." << std::endl <<
			"For online help and background, visit http://knowrob.org/" << std::endl <<
			std::endl;

		std::cout << PROMPT << std::flush;
		while(!has_stop_request_) {
			const auto c = getch();
			switch(c) {
				case -1:
					break;
				case 27:
					handleEscapeSequence();
					break;
				case 10:
					enter();
					break;
				case 127:
					backspace();
					break;
				default:
					// FIXME: make sure c is alphabetic or numeric
					insert(c);
					break;
			}
		}

		return EXIT_SUCCESS;
	}

protected:
	HybridQA hybridQA_;
	std::atomic<bool> has_stop_request_;
	int numSolutions_;
	uint32_t cursor_;
	std::string currentQuery_;
};

int main(int argc, char **argv)
{
	knowrob::logging::initialize();
	// TODO: would be nice if this would be done "under the hood"
	//  in the constructor. but I got a segfault when I tried, not sure why.
	knowrob::PrologReasoner::initialize(argc, argv);

	try {
		po::options_description general("General options");
		general.add_options()
			("help", "produce a help message")
			//("help-module", po::value<std::string>(), "produce a help for a given module")
			("version", "output the version number");

		// Declare an options description instance which will be shown
		// to the user
		po::options_description visible("Allowed options");
		visible.add(general);

		// parse command line arguments
		po::variables_map vm;
		po::store(po::parse_command_line(argc, argv, visible), vm);

		if (vm.count("help"))
		{
			std::cout << visible;
			return EXIT_SUCCESS;
		}

		return HybridQATerminal(vm).run();
	}
	catch(std::exception& e) {
		KB_ERROR("an exception occurred when running hyres-commandline: {}.", e.what());
		return EXIT_FAILURE;
	}
}
