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
#include <algorithm>
#include <list>
// BOOST
#include <boost/program_options/options_description.hpp>
#include <boost/program_options/variables_map.hpp>
#include <boost/program_options/parsers.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
// KnowRob
#include <knowrob/knowrob.h>
#include <knowrob/logging.h>
#include <knowrob/HybridQA.h>
#include <knowrob/queries.h>

using namespace knowrob;
namespace po = boost::program_options;

static const char* PROMPT = "?- ";

// TODO support queries like: mongolog:(woman(X), ...) or (mongolog | prolog):(woman(X), ...)
//    but there could be different instances of the same reasoner type. So might need a reasoner id instead.

class QueryHistory {
public:
	explicit QueryHistory()
	: selection_(data_.end()),
	  pos_(-1),
	  maxHistoryItems_(100) {}

	void append(const std::string &queryString) {
		data_.push_front(queryString);
		reset();
	}

	void reset() {
		selection_ = data_.end();
		pos_ = -1;
	}

	void save(const std::string &historyFile) {
		std::ofstream file(historyFile);
		if(file.good()) {
			boost::archive::text_oarchive oa(file);
			oa << data_.size();
			for(auto &x : data_) oa << x;
		}
		else {
			KB_WARN("unable to write history to file '{}'", historyFile);
		}
	}

	void load(const std::string &historyFile) {
		std::ifstream file(historyFile);
		if(file.good()) {
			boost::archive::text_iarchive ia(file);
			std::list<std::string>::size_type size;
			ia >> size;
			size = std::min(maxHistoryItems_, size);
			for(int i = 0; i < size; ++i) {
				std::string queryString;
				ia >> queryString;
				data_.push_back(queryString);
			}
		}
	}

	const std::string& getSelection() { return *selection_; }

	bool hasSelection() { return selection_ != data_.end(); }

	void nextItem() {
		if(pos_ == -1) {
			selection_ = data_.begin();
			pos_ = 0;
		}
		else if(pos_ == 0) {
			selection_++;
		}
		if(selection_ == data_.end()) {
			pos_ = 1;
		}
	}

	void previousItem() {
		if(selection_ == data_.begin()) {
			selection_ = data_.end();
			pos_ = -1;
		}
		else if(pos_ == 0 || pos_ == 1) {
			selection_--;
			pos_ = 0;
		}
	}
protected:
	std::list<std::string> data_;
	std::list<std::string>::iterator selection_;
	const std::string historyFile_;
	unsigned long maxHistoryItems_;
	int pos_;
};

class HybridQATerminal : public QueryResultHandler {
public:
	explicit HybridQATerminal(const boost::property_tree::ptree &config)
	: has_stop_request_(false),
	  cursor_(0),
	  numSolutions_(0),
	  hybridQA_(config),
	  historyFile_("history.txt")
	{
		history_.load(historyFile_);
	}

	static char getch() {
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
			// add query to history
			history_.append(queryString);
			history_.save(historyFile_);
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

	void setQuery(const std::string &queryString) {
		auto oldLength = currentQuery_.length();
		auto newLength = queryString.length();
		// move to cursor pos=0 and insert the new query
		std::cout << "\r" << PROMPT << queryString;
		// overwrite remainder of old query string with spaces
		for(auto counter = oldLength; counter > newLength; --counter) {
			std::cout << ' ';
		}
		// move back cursor
		if(oldLength > newLength) {
			std::cout << "\033[" << (oldLength-newLength) << "D";
		}
		std::cout << std::flush;
		currentQuery_ = queryString;
		cursor_ = currentQuery_.length();
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
		history_.nextItem();
		if(history_.hasSelection()) {
			setQuery(history_.getSelection());
		}
	}

	void moveDown() {
		history_.previousItem();
		setQuery(history_.hasSelection() ? history_.getSelection() : "");
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
	std::string historyFile_;
	QueryHistory history_;
};


int run(int argc, char **argv) {
	po::options_description general("General options");
	general.add_options()
			("help", "produce a help message")
			("verbose", "print informational messages")
			("config-file", po::value<std::string>()->required(), "a configuration file in JSON format")
			("version", "output the version number");
	// Declare an options description instance which will be shown
	// to the user
	po::options_description visible("Allowed options");
	visible.add(general);
	// parse command line arguments
	po::variables_map vm;
	po::store(po::parse_command_line(argc, argv, visible), vm);

	if(vm.count("help")) {
		std::cout << visible;
		return EXIT_SUCCESS;
	}

	// read settings
	// TODO: fallback to default settings
	boost::property_tree::ptree config;
	if(vm.count("config-file")) {
		boost::property_tree::read_json(
				vm["config-file"].as<std::string>(),
				config);
	} else {
		std::cout << "config-file is missing" << std::endl;
		return EXIT_FAILURE;
	}

	// configure logging
	auto log_config = config.get_child_optional("logging");
	if(log_config) {
		Logger::loadConfiguration(log_config.value());
	}
	// overwrite console logger level (default: prevent messages being printed, only print warnings and errors)
	Logger::setSinkLevel(Logger::Console,
						 vm.count("verbose") ? spdlog::level::debug : spdlog::level::warn);

	return HybridQATerminal(config).run();
}


int main(int argc, char **argv) {
	InitKnowledgeBase(argc, argv);
	try {
		return run(argc,argv);
	}
	catch(std::exception& e) {
		KB_ERROR("an exception occurred: {}.", e.what());
		return EXIT_FAILURE;
	}
}
