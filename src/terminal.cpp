/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include <termios.h>
// STD
#include <exception>
#include <iostream>
#include <algorithm>
#include <memory>
#include <list>
// BOOST
#include <boost/program_options/options_description.hpp>
#include <boost/program_options/variables_map.hpp>
#include <boost/program_options/parsers.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <utility>
// KnowRob
#include <knowrob/knowrob.h>
#include <knowrob/Logger.h>
#include <knowrob/KnowledgeBase.h>
#include "knowrob/formulas/Predicate.h"
#include "knowrob/queries/QueryParser.h"
#include "knowrob/semweb/PrefixRegistry.h"
#include "knowrob/queries/QueryError.h"
#include "knowrob/queries/QueryTree.h"
#include "knowrob/queries/Answer.h"
#include "knowrob/queries/AnswerYes.h"

using namespace knowrob;
namespace po = boost::program_options;

static const char *PROMPT = "?- ";

namespace knowrob {
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
			// FIXME: seems like history file will be corrupted in case the program is terminated during writing!
			std::ofstream file(historyFile);
			if (file.good()) {
				boost::archive::text_oarchive oa(file);
				oa << data_.size();
				for (auto &x: data_) oa << x;
			} else {
				KB_WARN("unable to write history to file '{}'", historyFile);
			}
		}

		void load(const std::string &historyFile) {
			std::ifstream file(historyFile);
			if (file.good()) {
				boost::archive::text_iarchive ia(file);
				std::list<std::string>::size_type size;
				ia >> size;
				size = std::min(maxHistoryItems_, size);
				for (int i = 0; i < size; ++i) {
					std::string queryString;
					ia >> queryString;
					data_.push_back(queryString);
				}
			}
		}

		const std::string &getSelection() { return *selection_; }

		bool hasSelection() { return selection_ != data_.end(); }

		void nextItem() {
			if (pos_ == -1) {
				selection_ = data_.begin();
				pos_ = 0;
			} else if (pos_ == 0) {
				selection_++;
			}
			if (selection_ == data_.end()) {
				pos_ = 1;
			}
		}

		void previousItem() {
			if (selection_ == data_.begin()) {
				selection_ = data_.end();
				pos_ = -1;
			} else if (pos_ == 0 || pos_ == 1) {
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

	template<class T>
	class TerminalCommand {
	public:
		using CommandFunction = std::function<bool(const std::vector<T> &arguments)>;

		TerminalCommand(std::string functor, uint32_t arity, CommandFunction function)
				: functor_(std::move(functor)), arity_(arity), function_(std::move(function)) {}

		bool runCommand(const std::vector<T> &arguments) {
			if (arguments.size() != arity_) {
				throw QueryError("Wrong number of arguments for terminal command '{}/{}'. "
								 "Actual number of arguments: {}.", functor_, arity_, arguments.size());
			}
			return function_(arguments);
		}

	protected:
		const std::string functor_;
		const uint32_t arity_;
		const CommandFunction function_;
	};
}

class KnowRobTerminal {
public:
	explicit KnowRobTerminal(const boost::property_tree::ptree &config)
			: has_stop_request_(false),
			  cursor_(0),
			  numSolutions_(0),
			  kb_(config),
			  historyFile_("history.txt") {
		try {
			history_.load(historyFile_);
		}
		catch (boost::archive::archive_exception &e) {
			KB_WARN("A 'boost::archive' exception occurred "
					"when loading history file ({}) of the terminal: {}. "
					"It might be that the file is corrupted for some reason.",
					historyFile_, e.what());
		}
		// define some terminal commands
		registerCommand("exit", 0,
						[this](const std::vector<TermPtr> &) { return exitTerminal(); });
		registerCommand("assert", 1,
						[this](const std::vector<FormulaPtr> &x) { return assertStatements(x); });
		registerCommand("tell", 1,
						[this](const std::vector<FormulaPtr> &x) { return assertStatements(x); });
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
			perror("read()");
		// reset old settings
		tcsetattr(0, TCSANOW, &old);
		return buf;
	}

	void registerCommand(const std::string &functor,
						 uint32_t arity,
						 const TerminalCommand<TermPtr>::CommandFunction &function) {
		firstOrderCommands_.emplace(functor, TerminalCommand(functor, arity, function));
	}

	void registerCommand(const std::string &functor,
						 uint32_t arity,
						 const TerminalCommand<FormulaPtr>::CommandFunction &function) {
		higherOrderCommands_.emplace(functor, TerminalCommand(functor, arity, function));
	}

	// Override QueryResultHandler
	bool pushQueryResult(const AnswerPtr &solution) {
		std::cout << *solution;
		numSolutions_ += 1;
		return !has_stop_request_;
	}

	bool runHigherOrderCommand(const std::string &functor, const std::string &queryString) {
		auto argsFormula = QueryParser::parse(queryString);
		auto needle = higherOrderCommands_.find(functor);
		if (needle == higherOrderCommands_.end()) {
			throw QueryError("Ignoring unknown higher-order command '{}'", functor);
		}
		if (argsFormula->type() == FormulaType::CONJUNCTION) {
			return needle->second.runCommand(((Conjunction *) argsFormula.get())->formulae());
		} else {
			return needle->second.runCommand({argsFormula});
		}
	}

	void runQuery(const std::string &queryString) {
		auto ctx = std::make_shared<QueryContext>(
			QUERY_FLAG_ALL_SOLUTIONS
			//| QUERY_FLAG_UNIQUE_SOLUTIONS
		);
		try {
			bool isQueryHandled = false;
			// make a lookahead if the query string starts with a functor of a registered
			// higher order command.
			// NOTE: this is needed because the query parser might not accept formula as argument of a predicate.
			size_t pos = queryString.find_first_of('(');
			if (pos != std::string::npos) {
				auto functor = queryString.substr(0, pos);
				auto needle = higherOrderCommands_.find(functor);
				if (needle != higherOrderCommands_.end()) {
					runHigherOrderCommand(functor, queryString.substr(pos));
					isQueryHandled = true;
				}
			}

			// parse query
			if (!isQueryHandled) {
				auto phi = QueryParser::parse(queryString);
				auto query = std::make_shared<FormulaQuery>(phi, ctx);
				if (query->formula()->type() == FormulaType::PREDICATE) {
					auto p = std::dynamic_pointer_cast<Predicate>(query->formula());
					auto needle = firstOrderCommands_.find(p->functor()->stringForm());
					if (needle != firstOrderCommands_.end()) {
						needle->second.runCommand(p->arguments());
						isQueryHandled = true;
					}
				}
				if (!isQueryHandled) {
					runQuery(query);
				}
			}
		}
		catch (std::exception &e) {
			std::cout << e.what() << std::endl;
		}
		// add query to history
		history_.append(queryString);
		history_.save(historyFile_);
	}

	void runQuery(const std::shared_ptr<const FormulaQuery> &query) {
		// evaluate query in hybrid QA system
		auto resultStream = kb_.submitQuery(query->formula(), query->ctx());
		auto resultQueue = resultStream->createQueue();

		numSolutions_ = 0;
		while (true) {
			auto nextResult = resultQueue->pop_front();

			if (nextResult->indicatesEndOfEvaluation()) {
				break;
			} else if (nextResult->type() == TokenType::ANSWER_TOKEN) {
				auto answer = std::static_pointer_cast<const Answer>(nextResult);

				if (answer->isPositive()) {
					auto positiveAnswer = std::static_pointer_cast<const AnswerYes>(answer);
					if (positiveAnswer->substitution()->empty()) {
						std::cout << "yes." << std::endl;
						numSolutions_ += 1;
						break;
					} else {
						pushQueryResult(positiveAnswer);
						numSolutions_ += 1;
					}
				} else {
					std::cout << *answer;
					numSolutions_ += 1;
				}
			}
		}

		if (numSolutions_ == 0) {
			std::cout << "no." << std::endl;
		}
	}

	bool assertStatements(const std::vector<FormulaPtr> &args) {
		std::vector<FramedTriplePtr> data(args.size());
		std::vector<FramedTriplePatternPtr> buf(args.size());
		uint32_t dataIndex = 0;

		for (auto &phi: args) {
			const QueryTree qt(phi);
			if (qt.numPaths() > 1) {
				throw QueryError("Disjunctions are not allowed in assertions. "
								 "Appears in statement {}.", *phi);
			} else if (qt.numPaths() == 0) {
				throw QueryError("Invalid assertion: '{}'", *phi);
			}
			for (auto &psi: qt.begin()->nodes()) {
				switch (psi->type()) {
					case knowrob::FormulaType::PREDICATE:
						buf[dataIndex] = std::make_shared<FramedTriplePattern>(
								std::static_pointer_cast<Predicate>(psi), false);
						data[dataIndex].ptr = new FramedTripleCopy();
						data[dataIndex].owned = true;
						buf[dataIndex]->instantiateInto(*data[dataIndex].ptr);
						dataIndex += 1;
						break;
					default:
						throw QueryError("Invalid assertion: '{}'", *phi);
				}
			}
		}
		if (kb_.insertAll(data)) {
			std::cout << "success, " << dataIndex << " statement(s) were asserted." << "\n";
			return true;
		} else {
			std::cout << "assertion failed." << "\n";
			return false;
		}
	}

	void enter() {
		std::cout << std::endl;
		runQuery(currentQuery_);
		if (!has_stop_request_) {
			std::cout << std::endl << PROMPT << std::flush;
			currentQuery_.clear();
			cursor_ = 0;
		}
	}

	void insert(char c) {
		if (cursor_ < currentQuery_.length()) {
			auto afterInsert = currentQuery_.substr(cursor_);
			std::cout << c << afterInsert <<
					  "\033[" << afterInsert.length() << "D" <<
					  std::flush;
			currentQuery_.insert(currentQuery_.begin() + cursor_, c);
		} else {
			std::cout << c << std::flush;
			currentQuery_ += c;
		}
		cursor_ += 1;
	}

	void insert(const std::string &str) {
		if (cursor_ < currentQuery_.length()) {
			auto afterInsert = currentQuery_.substr(cursor_);
			std::cout << str << afterInsert <<
					  "\033[" << afterInsert.length() << "D" <<
					  std::flush;
			currentQuery_.insert(currentQuery_.begin() + cursor_, str.begin(), str.end());
		} else {
			std::cout << str << std::flush;
			currentQuery_ += str;
		}
		cursor_ += str.length();
	}

	void setQuery(const std::string &queryString) {
		auto oldLength = currentQuery_.length();
		auto newLength = queryString.length();
		// move to cursor pos=0 and insert the new query
		std::cout << "\r" << PROMPT << queryString;
		// overwrite remainder of old query string with spaces
		for (auto counter = oldLength; counter > newLength; --counter) {
			std::cout << ' ';
		}
		// move back cursor
		if (oldLength > newLength) {
			std::cout << "\033[" << (oldLength - newLength) << "D";
		}
		std::cout << std::flush;
		currentQuery_ = queryString;
		cursor_ = currentQuery_.length();
	}

	void tabulator() {
		// extract last typed word
		auto itr = currentQuery_.rbegin();
		while (itr != currentQuery_.rend() && isalpha(*itr)) { ++itr; }
		auto lastWord = std::string(itr.base(), currentQuery_.end());

		// if not the first word, check if preceded by "$ns:"
		std::optional<std::string> namespaceAlias;
		if (itr != currentQuery_.rend() && *itr == ':') {
			// read the namespace alias
			++itr;
			auto aliasEnd = itr;
			while (itr != currentQuery_.rend() && isalpha(*itr)) { ++itr; }
			namespaceAlias = std::string(itr.base(), aliasEnd.base());
		}

		if (namespaceAlias.has_value()) {
			autoCompleteLocal(lastWord, namespaceAlias.value());
		} else {
			autoCompleteGlobal(lastWord);
		}
	}

	bool autoCompleteCurrentWord(const std::string &word, const std::string_view &completion) {
		auto mismatch = std::mismatch(word.begin(), word.end(), completion.begin());
		if (mismatch.first == word.end()) {
			// insert remainder
			auto remainder = std::string(mismatch.second, completion.end());
			insert(remainder);
			return true;
		}
		return false;
	}

	bool autoCompleteGlobal(const std::string &word) {
		std::vector<std::string_view> aliases;
		if (word.empty()) {
			aliases = semweb::PrefixRegistry::getAliasesWithPrefix("");
		} else {
			aliases = semweb::PrefixRegistry::getAliasesWithPrefix(word);
		}

		if (aliases.size() == 1) {
			// only one possible completion
			if (autoCompleteCurrentWord(word, aliases[0])) {
				insert(':');
				return true;
			}
		} else if (aliases.size() > 1) {
			// TODO: auto-complete up to common prefix among aliases
			displayOptions(aliases);
			return true;
		}

		return false;
	}

	bool autoCompleteLocal(const std::string &word, const std::string &nsAlias) {
		auto uri = semweb::PrefixRegistry::aliasToUri(nsAlias);
		if (uri.has_value()) {
			auto partialIRI = uri.value().get() + "#" + word;
			auto propertyOptions = kb_.vocabulary()->getDefinedPropertyNamesWithPrefix(partialIRI);
			auto classOptions = kb_.vocabulary()->getDefinedClassNamesWithPrefix(partialIRI);
			size_t namePosition = uri.value().get().length() + 1;

			// create options array holding only the name of entities.
			// note that strings are not copied by using string_view
			std::vector<std::string_view> options(propertyOptions.size() + classOptions.size());
			uint32_t index = 0;
			for (auto &iri: propertyOptions) options[index++] = iri.substr(namePosition);
			for (auto &iri: classOptions) options[index++] = iri.substr(namePosition);

			if (options.size() == 1) {
				// only one possible completion
				if (autoCompleteCurrentWord(word, options[0])) {
					return true;
				}
			} else if (options.size() > 1) {
				// TODO: auto-complete up to common prefix among options
				displayOptions(options);
				return true;
			}
		} else {
			KB_WARN("the namespace alias '{}' is unknown.", nsAlias);
		}
		return false;
	}

	void displayOptions(const std::vector<std::string_view> &options) {
		std::string optionsStr = "\n";
		for (const auto &option: options) {
			optionsStr += std::string(option) + "\n";
		}
		// print options to the terminal
		std::cout << optionsStr << PROMPT << currentQuery_ << std::flush;
	}

	void backspace() {
		if (cursor_ == 0) {
			return;
		} else if (cursor_ < currentQuery_.length()) {
			auto afterDelete = currentQuery_.substr(cursor_);
			std::cout << '\b' << afterDelete << ' ' <<
					  "\033[" << (afterDelete.length() + 1) << "D" << std::flush;
			currentQuery_.erase(cursor_ - 1, 1);
		} else {
			std::cout << '\b' << ' ' << '\b' << std::flush;
			currentQuery_.pop_back();
		}
		cursor_ -= 1;
	}

	void moveToBegin() {
		if (cursor_ > 0) {
			std::cout << "\033[" << cursor_ << "D" << std::flush;
			cursor_ = 0;
		}
	}

	void moveToEnd() {
		if (cursor_ < currentQuery_.length()) {
			std::cout << "\033[" << (currentQuery_.length() - cursor_) << "C" << std::flush;
			cursor_ = currentQuery_.length();
		}
	}

	void moveLeft() {
		if (cursor_ > 0) {
			std::cout << "\033[1D" << std::flush;
			cursor_ -= 1;
		}
	}

	void moveRight() {
		if (cursor_ < currentQuery_.length()) {
			std::cout << "\033[1C" << std::flush;
			cursor_ += 1;
		}
	}

	void moveUp() {
		history_.nextItem();
		if (history_.hasSelection()) {
			setQuery(history_.getSelection());
		}
	}

	void moveDown() {
		history_.previousItem();
		setQuery(history_.hasSelection() ? history_.getSelection() : "");
	}

	void handleEscapeSequence() {
		// the escape code \033 is followed by 2-3 more bytes
		if (getch() == '[') {
			switch (getch()) {
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

	bool exitTerminal() {
		has_stop_request_ = true;
		return true;
	}

	int run() {
		std::cout << "Welcome to KnowRob." << '\n' <<
				  "For online help and background, visit http://knowrob.org/" << '\n' <<
				  '\n';

		std::cout << PROMPT << std::flush;
		while (!has_stop_request_) {
			const auto c = getch();
			switch (c) {
				case -1:
					break;
				case 27:
					handleEscapeSequence();
					break;
				case 9:
					tabulator();
					break;
				case 10:
					enter();
					break;
				case 127:
					backspace();
					break;
				default:
					insert(c);
					break;
			}
		}

		return EXIT_SUCCESS;
	}

protected:
	KnowledgeBase kb_;
	std::atomic<bool> has_stop_request_;
	int numSolutions_;
	uint32_t cursor_;
	std::string currentQuery_;
	std::string historyFile_;
	QueryHistory history_;
	std::map<std::string, TerminalCommand<TermPtr>, std::less<>> firstOrderCommands_;
	std::map<std::string, TerminalCommand<FormulaPtr>> higherOrderCommands_;
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

	if (vm.count("help")) {
		std::cout << visible;
		return EXIT_SUCCESS;
	}

	// read settings
	// TODO: fallback to default settings
	boost::property_tree::ptree config;
	if (vm.count("config-file")) {
		boost::property_tree::read_json(
				vm["config-file"].as<std::string>(),
				config);
	} else {
		std::cout << "'config-file' commandline argument is missing" << std::endl;
		return EXIT_FAILURE;
	}

	// configure logging
	auto log_config = config.get_child_optional("logging");
	if (log_config) {
		Logger::loadConfiguration(log_config.value());
	}
	// overwrite console logger level (default: prevent messages being printed, only print warnings and errors)
	Logger::setSinkLevel(Logger::Console,
						 vm.count("verbose") ? spdlog::level::debug : spdlog::level::warn);

	return KnowRobTerminal(config).run();
}


int main(int argc, char **argv) {
	InitKnowledgeBase(argc, argv);
	try {
		return run(argc, argv);
	}
	catch (std::exception &e) {
		KB_ERROR("a '{}' exception occurred in main loop: {}.", typeid(e).name(), e.what());
		return EXIT_FAILURE;
	}
}
