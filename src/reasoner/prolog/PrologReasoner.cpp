/*
 * Copyright (c) 2022, Daniel Beßler
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include <memory>
#include <filesystem>
#include <utility>

#define PL_SAFE_ARG_MACROS

#include <SWI-cpp.h>

#include "knowrob/knowrob.h"
#include "knowrob/Logger.h"
#include "knowrob/reasoner/ReasonerManager.h"
#include "knowrob/reasoner/prolog/PrologReasoner.h"
#include "knowrob/reasoner/prolog/logging.h"
#include "knowrob/reasoner/prolog/algebra.h"
#include "knowrob/terms/ListTerm.h"
#include "knowrob/formulas/Bottom.h"
#include "knowrob/queries/TokenQueue.h"
#include "knowrob/semweb/PrefixRegistry.h"
#include "knowrob/semweb/ImportHierarchy.h"
#include "knowrob/backend/KnowledgeGraph.h"
#include "knowrob/URI.h"
#include "knowrob/KnowledgeBase.h"

using namespace knowrob;

// make reasoner type accessible
KNOWROB_BUILTIN_REASONER("Prolog", PrologReasoner)

// forward declarations of foreign predicates
foreign_t pl_rdf_register_namespace2(term_t, term_t);

foreign_t sw_graph_get_imports4(term_t, term_t, term_t, term_t);

foreign_t sw_graph_add_direct_import4(term_t, term_t, term_t, term_t);

foreign_t sw_current_graph3(term_t, term_t, term_t);

foreign_t sw_set_current_graph3(term_t, term_t, term_t);

foreign_t sw_unset_current_graph3(term_t, term_t, term_t);

foreign_t sw_default_graph3(term_t, term_t, term_t);

foreign_t sw_set_default_graph3(term_t, term_t, term_t);

foreign_t sw_url_graph2(term_t, term_t);

foreign_t sw_url_version2(term_t, term_t);

foreign_t url_resolve2(term_t, term_t);

// semantic web extension
PL_extension sw_extension[] = {
		{"sw_url_graph",                   2, (pl_function_t) sw_url_graph2,               0},
		{"sw_url_version",                 2, (pl_function_t) sw_url_version2,             0},
		{"sw_default_graph_cpp",           3, (pl_function_t) sw_default_graph3,           0},
		{"sw_set_default_graph_cpp",       3, (pl_function_t) sw_set_default_graph3,       0},
		{"sw_graph_get_imports_cpp",       4, (pl_function_t) sw_graph_get_imports4,       0},
		{"sw_graph_add_direct_import_cpp", 4, (pl_function_t) sw_graph_add_direct_import4, 0},
		{"sw_current_graph_cpp",           3, (pl_function_t) sw_current_graph3,           0},
		{"sw_set_current_graph_cpp",       3, (pl_function_t) sw_set_current_graph3,       0},
		{"sw_unset_current_graph_cpp",     3, (pl_function_t) sw_unset_current_graph3,     0},
		{nullptr,                          0, nullptr,                                     0}
};
// defined in blackboard.cpp
extern PL_extension qa_predicates[];

bool PrologReasoner::isPrologInitialized_ = false;
bool PrologReasoner::isKnowRobInitialized_ = false;

PrologReasoner::PrologReasoner(std::string reasonerID)
		: reasonerID_(std::move(reasonerID)),
		  reasonerIDTerm_(std::make_shared<StringTerm>(reasonerID_)),
		  importHierarchy_(std::make_unique<semweb::ImportHierarchy>()) {
	addDataHandler(DataSource::PROLOG_FORMAT, [this]
			(const DataSourcePtr &dataFile) { return consult(dataFile->uri()); });
	addDataHandler(DataSource::RDF_XML_FORMAT, [this]
			(const DataSourcePtr &dataFile) { return load_rdf_xml(dataFile->uri()); });
}

PrologReasoner::~PrologReasoner() {
}

const functor_t &PrologReasoner::callFunctor() {
	static const auto reasoner_call_f = PL_new_functor(PL_new_atom("reasoner_call"), 2);
	return reasoner_call_f;
}

PrologThreadPool &PrologReasoner::threadPool() {
	// make sure PL_initialise was called
	initializeProlog();
	// a thread pool shared among all PrologReasoner instances
	static PrologThreadPool threadPool_(std::thread::hardware_concurrency());
	return threadPool_;
}

void PrologReasoner::initializeProlog() {
	if (isPrologInitialized_) return;
	// toggle on flag
	isPrologInitialized_ = true;

	static int pl_ac = 0;
	static char *pl_av[5];
	pl_av[pl_ac++] = getNameOfExecutable();
	// '-g true' is used to suppress the welcome message
	pl_av[pl_ac++] = (char *) "-g";
	pl_av[pl_ac++] = (char *) "true";
	// Inhibit any signal handling by Prolog
	pl_av[pl_ac++] = (char *) "--signals=false";
	pl_av[pl_ac] = nullptr;
	PL_initialise(pl_ac, pl_av);
	KB_DEBUG("Prolog has been initialized.");
}

bool PrologReasoner::initializeGlobalPackages() {
	// load some default code into user module.  e.g. the extended module syntax etc
	return consult(std::filesystem::path("reasoner") / "prolog" / "__init__.pl",
				   "user", false);
}

bool PrologReasoner::loadConfig(const ReasonerConfig &cfg) {
	// call PL_initialize
	initializeProlog();

	if (!isKnowRobInitialized_) {
		isKnowRobInitialized_ = true;
		// register some foreign predicates, i.e. cpp functions that are used to evaluate predicates.
		// note: the predicates are loaded into module "user"
		PL_register_foreign("knowrob_register_namespace",
							2, (pl_function_t) pl_rdf_register_namespace2, 0);
		PL_register_foreign("url_resolve",
							2, (pl_function_t) url_resolve2, 0);
		PL_register_foreign("log_message", 2, (pl_function_t) pl_log_message2, 0);
		PL_register_foreign("log_message", 4, (pl_function_t) pl_log_message4, 0);
		PL_register_extensions_in_module("algebra", algebra_predicates);
		PL_register_extensions_in_module("semweb", sw_extension);
		PL_register_extensions_in_module("user", qa_predicates);
		KB_INFO("common foreign Prolog modules have been registered.");
		// auto-load some files into "user" module
		initializeGlobalPackages();

		// register RDF namespaces with Prolog.
		// in particular the ones specified in settings are globally registered with PrefixRegistry.
		static const auto register_prefix_i =
				std::make_shared<PredicateIndicator>("rdf_register_prefix", 3);
		for (auto &pair: semweb::PrefixRegistry::get()) {
			const auto &uri = pair.first;
			const auto &alias = pair.second;
			eval(std::make_shared<Predicate>(Predicate(register_prefix_i, {
					std::make_shared<StringTerm>(alias),
					std::make_shared<StringTerm>(uri + "#")
			})), nullptr, false);
		}
	}

	// load properties into the reasoner module.
	// this is needed mainly for the `reasoner_setting/2` that provides reasoner instance specific settings.
	for (auto &pair: cfg) {
		auto key_t = cfg.createKeyTerm(pair.first, ":");
		auto val_t = std::make_shared<StringTerm>(pair.second);
		setReasonerSetting(key_t, val_t);
	}
	// load reasoner default packages. this is usually the code that implements the reasoner.
	initializeDefaultPackages();

	return true;
}

void PrologReasoner::setDataBackend(const DataBackendPtr &backend) {
	// TODO: think about how data backend of Prolog would be configured
}

void PrologReasoner::start() {
}

void PrologReasoner::stop() {
}

bool PrologReasoner::setReasonerSetting(const TermPtr &key, const TermPtr &valueString) {
	static auto set_setting_f =
			std::make_shared<PredicateIndicator>("reasoner_set_setting", 3);
	return eval(std::make_shared<Predicate>(
						Predicate(set_setting_f, {reasonerIDTerm_, key, valueString})),
				nullptr, false);
}

std::shared_ptr<DefinedReasoner> PrologReasoner::getDefinedReasoner(
		const term_t &t_reasonerManager, const term_t &t_reasonerModule) {
	int i_reasonerManager;
	if (!PL_get_integer(t_reasonerManager, &i_reasonerManager)) return {};
	auto reasonerManager = ReasonerManager::getReasonerManager(i_reasonerManager);
	if (!reasonerManager) return {};

	char *reasonerModule;
	if (PL_get_atom_chars(t_reasonerModule, &reasonerModule)) {
		return reasonerManager->getReasonerWithID(reasonerModule);
	} else {
		return {};
	}
}

bool PrologReasoner::consult(const std::filesystem::path &prologFile,
							 const char *contextModule,
							 bool doTransformQuery) {
	static auto consult_f = std::make_shared<PredicateIndicator>("consult", 1);
	auto path = getPrologPath(prologFile);
	auto arg = std::make_shared<StringTerm>(path.native());
	return eval(std::make_shared<Predicate>(Predicate(consult_f, {arg})), contextModule, doTransformQuery);
}

bool PrologReasoner::load_rdf_xml(const std::filesystem::path &rdfFile) {
	static auto consult_f = std::make_shared<PredicateIndicator>("load_rdf_xml", 2);
	auto path = getResourcePath(rdfFile);
	auto arg0 = std::make_shared<StringTerm>(path.native());
	return eval(std::make_shared<Predicate>(Predicate(consult_f, {arg0, reasonerIDTerm_})));
}

bool PrologReasoner::assertFact(const std::shared_ptr<Predicate> &fact) {
	static auto assert_f = std::make_shared<PredicateIndicator>("assertz", 1);
	return eval(std::make_shared<Predicate>(Predicate(assert_f, {fact})));
}

PredicateDescriptionPtr PrologReasoner::getDescription(const PredicateIndicatorPtr &indicator) {
	static auto current_predicate_f = std::make_shared<PredicateIndicator>(
			"reasoner_defined_predicate", 2);

	// TODO: rather cache predicate descriptions centrally for all reasoner
	auto it = predicateDescriptions_.find(*indicator);
	if (it != predicateDescriptions_.end()) {
		return it->second;
	} else {
		// evaluate query
		auto type_v = std::make_shared<Variable>("Type");
		auto solution = oneSolution(std::make_shared<Predicate>(Predicate(
				current_predicate_f, {indicator->toTerm(), type_v})));

		if (!solution || solution->indicatesEndOfEvaluation()) {
			// FIXME: this is not safe if files are consulted at runtime
			static std::shared_ptr<PredicateDescription> nullDescr;
			predicateDescriptions_[*indicator] = nullDescr;
			return nullDescr;
		} else {
			// read type of predicate
			auto ptype = predicateTypeFromTerm(solution->substitution()->get(*type_v));
			// create a PredicateDescription
			auto newDescr = std::make_shared<PredicateDescription>(indicator, ptype);
			predicateDescriptions_[*indicator] = newDescr;
			return newDescr;
		}
	}
}

std::shared_ptr<Term> PrologReasoner::readTerm(const std::string &queryString) {
	static std::shared_ptr<Variable> termVar(new Variable("TermFromAtom"));
	static std::shared_ptr<Variable> listVar(new Variable("Vars"));
	static std::shared_ptr<ListTerm> opts(new ListTerm({
															   std::make_shared<Predicate>(
																	   Predicate("variable_names", {listVar}))
													   }));

	auto termAtom = std::make_shared<StringTerm>(queryString);
	// run a query
	auto result = oneSolution(std::make_shared<Predicate>(Predicate(
			"read_query", {termAtom, termVar, opts})), nullptr, false);

	if (result->indicatesEndOfEvaluation()) {
		return Bottom::get();
	} else {
		std::shared_ptr<Term> term = result->substitution()->get(*termVar);
		std::shared_ptr<Term> varNames = result->substitution()->get(*listVar);

		if (varNames->type() == TermType::LIST) {
			auto *l = (ListTerm *) varNames.get();
			if (l->elements().empty()) {
				return term;
			}

			Substitution s;
			std::map<std::string, Variable *> varNames0;
			for (const auto &x: l->elements()) {
				// each element in the list has the form '='(VarName,Var)
				auto *p = (Predicate *) x.get();
				auto *varName = (StringTerm *) (p->arguments()[0].get());
				auto *var = (Variable *) (p->arguments()[1].get());

				s.set(*var, std::make_shared<Variable>(varName->value()));
			}

			auto *p0 = (Predicate *) term.get();
			return std::dynamic_pointer_cast<Predicate>(p0->applySubstitution(s));
		} else {
			KB_WARN("something went wrong");
			return term;
		}
	}
}

bool PrologReasoner::eval(
		const std::shared_ptr<Predicate> &p,
		const char *moduleName,
		bool doTransformQuery) {
	auto answer = oneSolution(p, moduleName, doTransformQuery);
	return (answer ? !answer->indicatesEndOfEvaluation() : false);
}

bool PrologReasoner::eval(
		const std::shared_ptr<const Query> &q,
		const char *moduleName,
		bool doTransformQuery) {
	auto answer = oneSolution(q, moduleName, doTransformQuery);
	return (answer ? !answer->indicatesEndOfEvaluation() : false);
}

AnswerYesPtr PrologReasoner::oneSolution(const std::shared_ptr<Predicate> &goal,
										 const char *moduleName,
										 bool doTransformQuery) {
	auto ctx = std::make_shared<QueryContext>(QUERY_FLAG_ONE_SOLUTION);
	return oneSolution(
			std::make_shared<FormulaQuery>(goal, ctx),
			moduleName,
			doTransformQuery);
}

AnswerYesPtr PrologReasoner::oneSolution(const std::shared_ptr<const Query> &goal,
										 const char *moduleName,
										 bool doTransformQuery) {
	// create an output queue for the query
	auto outputStream = std::make_shared<TokenQueue>();
	auto outputChannel = TokenStream::Channel::create(outputStream);
	auto call_f = (doTransformQuery ? callFunctor() : (functor_t) 0);
	std::shared_ptr<StringTerm> moduleTerm = moduleName ?
											 std::make_shared<StringTerm>(moduleName) : reasonerIDTerm_;
	// create a runner for a worker thread
	auto workerGoal = std::make_shared<PrologQueryRunner>(
			this,
			PrologQueryRunner::Request(goal, call_f, moduleTerm, *DefaultGraphSelector()),
			outputChannel,
			true
	);

	std::optional<std::exception> exc;
	auto excPtr = &exc;
	PrologReasoner::threadPool().pushWork(workerGoal,
										  [outputStream, excPtr](const std::exception &e) {
											  *excPtr = e;
											  outputStream->close();
										  });
	auto solution = outputStream->pop_front();
	// rethrow any exceptions in this thread!
	if (exc.has_value()) throw (exc.value());

	if (solution->indicatesEndOfEvaluation()) {
		return {};
	}
	if (solution->type() == TokenType::ANSWER_TOKEN) {
		auto nextAnswer = std::static_pointer_cast<const Answer>(solution);
		if (nextAnswer->isPositive()) {
			return std::static_pointer_cast<const AnswerYes>(nextAnswer);
		}
	}
	return {};
}

std::list<AnswerYesPtr> PrologReasoner::allSolutions(const std::shared_ptr<Predicate> &goal,
													 const char *moduleName,
													 bool doTransformQuery) {
	auto ctx = std::make_shared<QueryContext>(QUERY_FLAG_ALL_SOLUTIONS);
	return allSolutions(
			std::make_shared<FormulaQuery>(goal, ctx),
			moduleName,
			doTransformQuery);
}

std::list<AnswerYesPtr> PrologReasoner::allSolutions(const std::shared_ptr<const Query> &goal,
													 const char *moduleName,
													 bool doTransformQuery) {
	std::list<AnswerYesPtr> results;
	TokenPtr nextResult;

	// create an output queue for the query
	auto outputStream = std::make_shared<TokenQueue>();
	auto outputChannel = TokenStream::Channel::create(outputStream);
	auto call_f = (doTransformQuery ? callFunctor() : (functor_t) 0);
	// stores exception if any
	std::optional<std::exception> exc;
	auto excPtr = &exc;
	std::shared_ptr<StringTerm> moduleTerm = moduleName ?
											 std::make_shared<StringTerm>(moduleName) : reasonerIDTerm_;
	// create a runner for a worker thread
	auto workerGoal = std::make_shared<PrologQueryRunner>(
			this,
			PrologQueryRunner::Request(goal, call_f, moduleTerm, *DefaultGraphSelector()),
			outputChannel,
			true
	);

	// assign the goal to a worker thread
	PrologReasoner::threadPool().pushWork(workerGoal,
										  [outputStream, excPtr](const std::exception &e) {
											  *excPtr = e;
											  outputStream->close();
										  });
	// wait until work is done, and push EOS
	workerGoal->join();
	// get all results
	while (true) {
		nextResult = outputStream->pop_front();

		if (nextResult->indicatesEndOfEvaluation()) {
			break;
		} else if (nextResult->type() == TokenType::ANSWER_TOKEN) {
			auto nextAnswer = std::static_pointer_cast<const Answer>(nextResult);
			if (nextAnswer->isPositive()) {
				results.push_back(std::static_pointer_cast<const AnswerYes>(nextAnswer));
			}
		}
	}

	if (exc.has_value()) throw (exc.value());

	return results;
}

TokenBufferPtr PrologReasoner::submitQuery(const RDFLiteralPtr &literal, const QueryContextPtr &ctx) {
	bool sendEOS = true;
	auto reasoner = this;
	auto answerBuffer = std::make_shared<TokenBuffer>();
	auto outputChannel = TokenStream::Channel::create(answerBuffer);

	auto query = std::make_shared<ConjunctiveQuery>(literal, ctx);
	// create a runner for a worker thread
	auto workerGoal = std::make_shared<PrologQueryRunner>(
			reasoner,
			PrologQueryRunner::Request(
					query,
					callFunctor(),
					reasonerIDTerm_,
					ctx->selector_),
			outputChannel,
			sendEOS);
	// assign the goal to a worker thread
	PrologReasoner::threadPool().pushWork(workerGoal,
										  [literal, outputChannel](const std::exception &e) {
											  KB_WARN("an exception occurred for prolog query ({}): {}.", literal,
													  e.what());
											  outputChannel->close();
										  });

	return answerBuffer;
}

std::filesystem::path PrologReasoner::getPrologPath(const std::filesystem::path &filePath) {
	static std::filesystem::path projectPath(KNOWROB_SOURCE_DIR);
	static std::filesystem::path installPath(KNOWROB_INSTALL_PREFIX);

	if (!exists(filePath)) {
		auto possiblePaths = {
				projectPath / filePath,
				projectPath / "src" / filePath,
				projectPath / "src" / "reasoner" / "prolog" / filePath,
				installPath / "share" / "knowrob" / filePath
		};
		for (const auto &p: possiblePaths) {
			if (exists(p)) return p;
		}
	}
	return filePath;
}

std::filesystem::path PrologReasoner::getResourcePath(const std::filesystem::path &filePath) {
	static std::filesystem::path projectPath(KNOWROB_SOURCE_DIR);
	static std::filesystem::path installPath(KNOWROB_INSTALL_PREFIX);

	if (!exists(filePath)) {
		auto possiblePaths = {
				projectPath / filePath,
				installPath / "share" / "knowrob" / filePath
		};
		for (const auto &p: possiblePaths) {
			if (exists(p)) return p;
		}
	}
	return filePath;
}

std::list<TermPtr> PrologReasoner::runTests(const std::string &target) {
	static const auto xunit_indicator =
			std::make_shared<PredicateIndicator>("xunit_term", 1);
	static const auto xunit_var = std::make_shared<Variable>("Term");
	static const auto silent_flag = std::make_shared<StringTerm>("silent");

	auto solutions = allSolutions(std::make_shared<Predicate>(Predicate(
			"test_and_report", {
					// unittest target
					std::make_shared<StringTerm>(target),
					// options
					std::make_shared<ListTerm>(ListTerm({
																std::make_shared<Predicate>(
																		Predicate(xunit_indicator, {xunit_var})),
																silent_flag
														}))
			})), nullptr, false);

	std::list<TermPtr> output;
	for (auto &solution: solutions) {
		output.push_back(solution->substitution()->get(*xunit_var));
	}
	return output;
}

// FOREIGN PREDICATES

foreign_t pl_rdf_register_namespace2(term_t prefix_term, term_t uri_term) {
	char *prefix, *uri;
	if (PL_get_atom_chars(prefix_term, &prefix) && PL_get_atom_chars(uri_term, &uri)) {
		semweb::PrefixRegistry::get().registerPrefix(prefix, uri);
	}
	return TRUE;
}

std::shared_ptr<semweb::ImportHierarchy> &getImportHierarchy(term_t t_manager, term_t t_reasoner) {
	static std::shared_ptr<semweb::ImportHierarchy> null;
	auto definedReasoner =
			PrologReasoner::getDefinedReasoner(t_manager, t_reasoner);
	if (!definedReasoner) return null;
	auto prologReasoner =
			std::dynamic_pointer_cast<PrologReasoner>(definedReasoner->reasoner());
	return prologReasoner ? prologReasoner->importHierarchy() : null;
}

foreign_t sw_current_graph3(term_t t_manager, term_t t_reasoner, term_t t_graph) {
	auto &hierarchy = getImportHierarchy(t_manager, t_reasoner);

	char *graph;
	if (hierarchy && PL_get_atom_chars(t_graph, &graph)) {
		return hierarchy->isCurrentGraph(graph);
	} else {
		return false;
	}
}

foreign_t sw_set_current_graph3(term_t t_manager, term_t t_reasoner, term_t t_graph) {
	auto &hierarchy = getImportHierarchy(t_manager, t_reasoner);

	char *graph;
	if (hierarchy && PL_get_atom_chars(t_graph, &graph)) {
		hierarchy->addCurrentGraph(graph);
		return true;
	} else {
		return false;
	}
}

foreign_t sw_unset_current_graph3(term_t t_manager, term_t t_reasoner, term_t t_graph) {
	auto &hierarchy = getImportHierarchy(t_manager, t_reasoner);

	char *graph;
	if (hierarchy && PL_get_atom_chars(t_graph, &graph)) {
		hierarchy->removeCurrentGraph(graph);
		return true;
	} else {
		return false;
	}
}

foreign_t sw_graph_add_direct_import4(term_t t_manager, term_t t_reasoner, term_t t_importer, term_t t_imported) {
	auto &hierarchy = getImportHierarchy(t_manager, t_reasoner);

	char *importer, *imported;
	if (hierarchy &&
		PL_get_atom_chars(t_importer, &importer) &&
		PL_get_atom_chars(t_imported, &imported)) {
		hierarchy->addDirectImport(importer, imported);
		return true;
	} else {
		return false;
	}
}

foreign_t sw_graph_get_imports4(term_t t_manager, term_t t_reasoner, term_t t_importer, term_t t_importedList) {
	auto &hierarchy = getImportHierarchy(t_manager, t_reasoner);
	char *importer;
	if (hierarchy && PL_get_atom_chars(t_importer, &importer)) {
		PlTail l(t_importedList);
		for (auto &x: hierarchy->getImports(importer))
			l.append(x->name().c_str());
		l.close();
		return true;
	}
	return false;
}

foreign_t sw_set_default_graph3(term_t t_manager, term_t t_reasoner, term_t t_graph) {
	auto &hierarchy = getImportHierarchy(t_manager, t_reasoner);
	char *graph;
	if (hierarchy && PL_get_atom_chars(t_graph, &graph)) {
		hierarchy->setDefaultGraph(graph);
		return true;
	}
	return false;

}

foreign_t sw_default_graph3(term_t t_manager, term_t t_reasoner, term_t t_graph) {
	auto &hierarchy = getImportHierarchy(t_manager, t_reasoner);
	return hierarchy && PL_unify_atom_chars(t_graph, hierarchy->defaultGraph().c_str());
}

foreign_t sw_url_graph2(term_t t_url, term_t t_graph) {
	char *url;
	if (PL_get_atom_chars(t_url, &url)) {
		auto name = KnowledgeGraph::getNameFromURI(url);
		return PL_unify_atom_chars(t_graph, name.c_str());
	}
	return false;
}

foreign_t sw_url_version2(term_t t_url, term_t t_version) {
	char *url;
	if (PL_get_atom_chars(t_url, &url)) {
		auto version = KnowledgeGraph::getVersionFromURI(url);
		return PL_unify_atom_chars(t_version, version.c_str());
	}
	return false;
}

foreign_t url_resolve2(term_t t_url, term_t t_resolved) {
	char *url;
	if (PL_get_atom_chars(t_url, &url)) {
		auto resolved = URI::resolve(url);
		return PL_unify_atom_chars(t_resolved, resolved.c_str());
	}
	return false;
}
