//
// Created by daniel on 25.01.24.
//

#include <iostream>
#include <functional>
#include <future>
#include <memory>
#include <boost/python.hpp>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>

#include "knowrob/Logger.h"
#include "knowrob/terms/Constant.h"
#include "knowrob/terms/ListTerm.h"
#include "knowrob/formulas/Bottom.h"
#include "knowrob/formulas/Top.h"
#include "knowrob/formulas/Negation.h"
#include "knowrob/formulas/Conjunction.h"
#include "knowrob/formulas/Disjunction.h"
#include "knowrob/formulas/Implication.h"
#include "knowrob/formulas/ModalFormula.h"
#include "knowrob/formulas/PredicateDescription.h"
#include "knowrob/modalities/EpistemicModality.h"
#include "knowrob/queries/QueryContext.h"
#include "knowrob/reasoner/Reasoner.h"

// optional member must be added with add_property
#define BOOST_PYTHON_ADD_OPTIONAL(X, Y) add_property(X, \
    make_getter(Y, KB_RETURN_POLICY_BY_VALUE), make_setter(Y, KB_RETURN_POLICY_BY_VALUE))

#define CONST_REF_RETURN return_value_policy<copy_const_reference>()
#define EXISTING_REF_RETURN return_value_policy<reference_existing_object>()
#define KB_RETURN_POLICY_BY_VALUE return_value_policy<return_by_value>()

#include "knowrob/py/converter.h"
#include "knowrob/py/wrapper.h"

using namespace knowrob;
namespace python = boost::python;

BOOST_PYTHON_MODULE (knowrob) {
	using namespace boost::python;
	typedef std::vector<TermPtr> TermList;
	typedef std::vector<FormulaPtr> FormulaList;
	typedef std::vector<StatementData> StatementList;

	/////////////////////////////////////////////////////
	// logging
	/////////////////////////////////////////////////////
	def("logInfo", +[](const std::string &msg) { KB_INFO(msg); });
	def("logWarn", +[](const std::string &msg) { KB_WARN(msg); });
	def("logError", +[](const std::string &msg) { KB_ERROR(msg); });
	def("logDebug", +[](const std::string &msg) { KB_DEBUG(msg); });
	def("logCritical", +[](const std::string &msg) { KB_CRITICAL(msg); });

	/////////////////////////////////////////////////////
	// mappings for the Agent class
	/////////////////////////////////////////////////////
	class_<Agent, std::shared_ptr<Agent>>("Agent", no_init)
			.def("iri", &Agent::iri, CONST_REF_RETURN)
			.def("getEgo", &Agent::getEgo)
			.def("get", &Agent::get);

	/////////////////////////////////////////////////////
	// mappings for the Term and its subclasses
	/////////////////////////////////////////////////////
	enum_<TermType>("TermType")
			.value("PREDICATE", TermType::PREDICATE)
			.value("DOUBLE", TermType::DOUBLE)
			.value("INT32", TermType::INT32)
			.value("LONG", TermType::LONG)
			.value("STRING", TermType::STRING)
			.value("VARIABLE", TermType::VARIABLE)
			.value("LIST", TermType::LIST);
	class_<Term, std::shared_ptr<TermWrap>, boost::noncopyable>("Term", boost::python::no_init)
			.def("type", &Term::type, CONST_REF_RETURN)
			.def("__eq__", &Term::operator==)
			.def("__str__", +[](Term &t){ return t.toString(); })
			.def("isGround", python::pure_virtual(&Term::isGround))
			.def("isAtomic", python::pure_virtual(&Term::isAtomic))
			.def("computeHash", python::pure_virtual(&Term::computeHash))
			.def("write", python::pure_virtual(&Term::write))
			.def("getVariables", python::pure_virtual(&Term::getVariables), CONST_REF_RETURN);
	class_<Variable, std::shared_ptr<Variable>, bases<Term>>("Variable", init<std::string>())
			.def(self < self)
			.def("name", &Variable::name, CONST_REF_RETURN);
	class_<ListTerm, std::shared_ptr<ListTerm>, bases<Term>>("ListTerm", init<const std::vector<TermPtr> &>())
			.def("__iter__", range(&ListTerm::begin, &ListTerm::end))
			.def("isNIL", &ListTerm::isNIL)
			.def("elements", &ListTerm::elements, CONST_REF_RETURN);
	class_<StringTerm, std::shared_ptr<StringTerm>, bases<Term>>
			("StringTerm", init<std::string>())
			.def("value", &StringTerm::value, CONST_REF_RETURN);
	class_<DoubleTerm, std::shared_ptr<DoubleTerm>, bases<Term>>
			("DoubleTerm", init<double>())
			.def("value", &DoubleTerm::value, CONST_REF_RETURN);
	class_<LongTerm, std::shared_ptr<LongTerm>, bases<Term>>
			("LongTerm", init<long>())
			.def("value", &LongTerm::value, CONST_REF_RETURN);
	class_<Integer32Term, std::shared_ptr<Integer32Term>, bases<Term>>
			("Integer32Term", init<int32_t>())
			.def("value", &Integer32Term::value, CONST_REF_RETURN);
	register_ptr_to_python<std::shared_ptr<knowrob::Term>>();

	// allow conversion between std::vector and python::list for Term objects.
	custom_vector_from_seq<TermPtr>();
	class_<TermList>("TermList").def(vector_indexing_suite<TermList, true>());

	/////////////////////////////////////////////////////
	// mappings for Formula and its subclasses
	/////////////////////////////////////////////////////
	enum_<FormulaType>("FormulaType")
			.value("PREDICATE", FormulaType::PREDICATE)
			.value("CONJUNCTION", FormulaType::CONJUNCTION)
			.value("DISJUNCTION", FormulaType::DISJUNCTION)
			.value("NEGATION", FormulaType::NEGATION)
			.value("IMPLICATION", FormulaType::IMPLICATION)
			.value("MODAL", FormulaType::MODAL);
	class_<Formula, std::shared_ptr<FormulaWrap>, boost::noncopyable>("Formula", python::no_init)
			.def("type", &Formula::type)
			.def("__eq__", &Formula::operator==)
			.def("isGround", python::pure_virtual(&Formula::isGround))
			.def("write", python::pure_virtual(&Formula::write))
			.def("applySubstitution", python::pure_virtual(&Formula::applySubstitution))
			.def("isAtomic", &Formula::isAtomic)
			.def("isTop", &Formula::isTop)
			.def("isBottom", &Formula::isBottom);
	class_<CompoundFormulaWrap, std::shared_ptr<CompoundFormula>, boost::noncopyable, bases<Formula>>
			("CompoundFormula", no_init)
			.def("operator_symbol", python::pure_virtual(&CompoundFormula::operator_symbol))
			.def("formulae", &CompoundFormula::formulae, CONST_REF_RETURN);
	class_<Negation, std::shared_ptr<Negation>, bases<CompoundFormula>>
			("Negation", init<const FormulaPtr &>())
			.def("negatedFormula", &Negation::negatedFormula, CONST_REF_RETURN);
	class_<Conjunction, std::shared_ptr<Conjunction>, bases<CompoundFormula>>
			("Conjunction", init<const FormulaList &>());
	class_<Disjunction, std::shared_ptr<Disjunction>, bases<CompoundFormula>>
			("Disjunction", init<const FormulaList &>());
	class_<Implication, std::shared_ptr<Implication>, bases<CompoundFormula>>
			("Implication", init<const FormulaPtr &, const FormulaPtr &>())
			.def("antecedent", &Implication::antecedent, CONST_REF_RETURN)
			.def("consequent", &Implication::consequent, CONST_REF_RETURN);

	// allow conversion between std::vector and python::list for Formula objects.
	custom_vector_from_seq<FormulaPtr>();
	class_<FormulaList>("FormulaList").def(vector_indexing_suite<FormulaList, true>());

	/////////////////////////////////////////////////////
	// mappings for Modals
	/////////////////////////////////////////////////////
	enum_<ModalityType>("ModalityType")
			.value("Epistemic", ModalityType::Epistemic)
			.value("Temporal_Past", ModalityType::Temporal_Past);
	enum_<TemporalOperator>("TemporalOperator")
			.value("ALWAYS", TemporalOperator::ALWAYS)
			.value("SOMETIMES", TemporalOperator::SOMETIMES);
	enum_<EpistemicOperator>("EpistemicOperator")
			.value("KNOWLEDGE", EpistemicOperator::KNOWLEDGE)
			.value("BELIEF", EpistemicOperator::BELIEF);
	class_<ModalFormula, std::shared_ptr<ModalFormula>, bases<CompoundFormula>>
			("ModalFormula", init<const ModalOperatorPtr &, const FormulaPtr &>())
			.def("modalOperator", &ModalFormula::modalOperator, CONST_REF_RETURN)
			.def("modalFormula", &ModalFormula::modalFormula, CONST_REF_RETURN)
			.def("isModalPossibility", &ModalFormula::isModalPossibility)
			.def("isModalNecessity", &ModalFormula::isModalNecessity);
	class_<ModalOperator, std::shared_ptr<ModalOperator>, bases<Term>>
			("ModalOperator", init<const std::shared_ptr<Modality> &, ModalOperatorType>())
			.def("modality", &ModalOperator::modality, CONST_REF_RETURN)
			.def("isModalNecessity", &ModalOperator::isModalNecessity)
			.def("isModalPossibility", &ModalOperator::isModalPossibility)
			.def("operatorType", &ModalOperator::operatorType)
			.def("isTransitive", &ModalOperator::isTransitive)
			.def("isEuclidean", &ModalOperator::isEuclidean)
			.def("symbol", &ModalOperator::symbol);
	class_<ModalIteration, std::shared_ptr<ModalIteration>>
			("ModalIteration", init<>())
			.def("__eq__", &ModalIteration::operator==)
			.def("__iter__", range(&ModalIteration::begin, &ModalIteration::end))
			.def("numOperators", &ModalIteration::numOperators)
			.def("emptyIteration", &ModalIteration::emptyIteration, CONST_REF_RETURN);
	/*
class_<Modality, std::shared_ptr<Modality>, boost::noncopyable>("Modality", no_init)
	.def("modalityType", python::pure_virtual(&Modality::modalityType))
	.def("parameters", &Modality::parameters, CONST_REF_RETURN)
	;
class_<EpistemicModality, std::shared_ptr<EpistemicModality>, bases<Modality>>
	("EpistemicModality", init<>())
	.def(init<const std::string_view&>())
	.def("agent", &EpistemicModality::agent, CONST_REF_RETURN)
	.def("necessity_symbol", &EpistemicModality::necessity_symbol)
	.def("possibility_symbol", &EpistemicModality::possibility_symbol)
	;
	 */

	/////////////////////////////////////////////////////
	// mappings for the Predicate class
	/////////////////////////////////////////////////////
	enum_<MaterializationStrategy>("MaterializationStrategy")
			.value("NEVER", MaterializationStrategy::NEVER)
			.value("ALWAYS", MaterializationStrategy::ALWAYS)
			.value("ON_DEMAND", MaterializationStrategy::ON_DEMAND);
	enum_<PredicateType>("PredicateType")
			.value("BUILT_IN", PredicateType::BUILT_IN)
			.value("EDB_RELATION", PredicateType::EDB_RELATION)
			.value("IDB_RELATION", PredicateType::IDB_RELATION);
	class_<PredicateIndicator, std::shared_ptr<PredicateIndicator>>
			("PredicateIndicator", init<const std::string &, unsigned int>())
			.def("__eq__", &PredicateIndicator::operator==)
			.def(self < self)
			.def("name", &PredicateIndicator::functor, CONST_REF_RETURN)
			.def("arity", &PredicateIndicator::arity)
			.def("toTerm", &PredicateIndicator::toTerm);
	class_<PredicateDescription, std::shared_ptr<PredicateDescription>>
			("PredicateDescription", init<
					const std::shared_ptr<PredicateIndicator> &,
					PredicateType,
					MaterializationStrategy>())
			.def("indicator", &PredicateDescription::indicator, CONST_REF_RETURN)
			.def("type", &PredicateDescription::type)
			.def("materializationStrategy", &PredicateDescription::materializationStrategy);
	class_<Predicate, std::shared_ptr<Predicate>, bases<Formula, Term>>
			("Predicate", init<const std::string &, const TermList &>())
			.def(init<const std::shared_ptr<PredicateIndicator> &, const TermList &>())
			.def(init<const Predicate &, const Substitution &>())
			.def("arguments", &Predicate::arguments, CONST_REF_RETURN)
			.def("indicator", &Predicate::indicator, CONST_REF_RETURN);
	class_<Bottom, std::shared_ptr<Bottom>, bases<Predicate>>("Bottom", no_init)
			.def("get", &Bottom::get, CONST_REF_RETURN);
	class_<Top, std::shared_ptr<Top>, bases<Predicate>>("Top", no_init)
			.def("get", &Top::get, CONST_REF_RETURN);

	/////////////////////////////////////////////////////
	// mappings for StatementData
	/////////////////////////////////////////////////////

	enum_<RDFType>("RDFType")
			.value("BOOLEAN_LITERAL", RDF_BOOLEAN_LITERAL)
			.value("INT64_LITERAL", RDF_INT64_LITERAL)
			.value("DOUBLE_LITERAL", RDF_DOUBLE_LITERAL)
			.value("STRING_LITERAL", RDF_STRING_LITERAL)
			.value("RESOURCE", RDF_RESOURCE);

	class_<StatementData, std::shared_ptr<StatementData>>("StatementData", init<>())
			.def(init<const char *, const char *, const char *, const char *, const char *>())
			.def_readwrite("documentID", &StatementData::documentID)
			.def_readwrite("subject", &StatementData::subject)
			.def_readwrite("predicate", &StatementData::predicate)
			.def_readwrite("object", &StatementData::object)
			.def_readwrite("graph", &StatementData::graph)
			.def_readwrite("agent", &StatementData::agent)
			.def_readwrite("objectDouble", &StatementData::objectDouble)
			.def_readwrite("objectInteger", &StatementData::objectInteger)
			.def_readwrite("objectType", &StatementData::objectType)
			.BOOST_PYTHON_ADD_OPTIONAL("temporalOperator", &StatementData::temporalOperator)
			.BOOST_PYTHON_ADD_OPTIONAL("epistemicOperator", &StatementData::epistemicOperator)
			.BOOST_PYTHON_ADD_OPTIONAL("begin", &StatementData::begin)
			.BOOST_PYTHON_ADD_OPTIONAL("end", &StatementData::end)
			.BOOST_PYTHON_ADD_OPTIONAL("confidence", &StatementData::confidence);

	// allow conversion between std::vector and python::list for Formula objects.
	custom_vector_from_seq<StatementData>();
	class_<StatementList>("StatementList").def(vector_indexing_suite<StatementList, true>());

	/////////////////////////////////////////////////////
	// mappings for literals
	/////////////////////////////////////////////////////
	class_<Literal, std::shared_ptr<Literal>>("Literal", init<const PredicatePtr &, bool>())
			.def(init<const Literal &, const Substitution &>())
			.def("predicate", &Literal::predicate, CONST_REF_RETURN)
			.def("isNegated", &Literal::isNegated)
			.def("functor", &Literal::functor, CONST_REF_RETURN)
			.def("arity", &Literal::arity)
					// FIXME: is a virtual method, might need a wrapper
			.def("numVariables", &Literal::numVariables)
					// FIXME: is a virtual method, might need a wrapper
			.def("applySubstitution", &Literal::applySubstitution);
	class_<RDFLiteral, std::shared_ptr<RDFLiteral>, bases<Literal>>
			("RDFLiteral", init<const StatementData &, bool>())
			.def(init<const StatementData &>())
			.def(init<const RDFLiteral &, const Substitution &>())
			.def("subjectTerm", &RDFLiteral::subjectTerm)
			.def("propertyTerm", &RDFLiteral::propertyTerm)
			.def("objectTerm", &RDFLiteral::objectTerm)
			.def("graphTerm", &RDFLiteral::graphTerm)
			.def("agentTerm", &RDFLiteral::agentTerm)
			.def("beginTerm", &RDFLiteral::beginTerm)
			.def("endTerm", &RDFLiteral::endTerm)
			.def("confidenceTerm", &RDFLiteral::confidenceTerm)
			.def("objectOperator", &RDFLiteral::objectOperator)
			.def("temporalOperator", &RDFLiteral::temporalOperator)
			.def("epistemicOperator", &RDFLiteral::epistemicOperator)
			.def("setGraphName", &RDFLiteral::setGraphName)
			.def("toStatementData", &RDFLiteral::toStatementData);

	/////////////////////////////////////////////////////
	// mappings for the Substitution class
	/////////////////////////////////////////////////////
	// need to select one overload for get, there is another for Variable as argument
	// which currently cannot be addressed in Python.
	using Substitution_get_string = const TermPtr &(Substitution::*)(const std::string &) const;
	class_<Substitution, std::shared_ptr<Substitution>>("Substitution", init<>())
			.def(init<std::map<Variable, TermPtr>>())
			.def("__eq__", &Substitution::operator==)
			.def("__iter__", range(&Substitution::begin, &Substitution::end))
			.def("empty", &Substitution::empty)
			.def("set", &Substitution::set)
			.def("get", static_cast<Substitution_get_string>(&Substitution::get), CONST_REF_RETURN)
			.def("contains", &Substitution::contains)
			.def("computeHash", &Substitution::computeHash);

	/////////////////////////////////////////////////////
	// mappings for the QueryContext class
	/////////////////////////////////////////////////////
	class_<QueryContext, std::shared_ptr<QueryContext>>("QueryContext", init<int>())
			.def(init<const QueryContext &, const ModalOperatorPtr &>())
			.def_readwrite("queryFlags", &QueryContext::queryFlags_)
			.def_readwrite("modalIteration", &QueryContext::modalIteration_)
			.def_readwrite("selector", &QueryContext::selector_);
	class_<GraphSelector, std::shared_ptr<GraphSelector>>("GraphSelector", init<>())
			.def_readwrite("graph", &GraphSelector::graph)
			.BOOST_PYTHON_ADD_OPTIONAL("agent", &GraphSelector::agent)
			.BOOST_PYTHON_ADD_OPTIONAL("temporalOperator", &GraphSelector::temporalOperator)
			.BOOST_PYTHON_ADD_OPTIONAL("epistemicOperator", &GraphSelector::epistemicOperator)
			.BOOST_PYTHON_ADD_OPTIONAL("begin", &GraphSelector::begin)
			.BOOST_PYTHON_ADD_OPTIONAL("end", &GraphSelector::end)
			.BOOST_PYTHON_ADD_OPTIONAL("confidence", &GraphSelector::confidence);
	register_ptr_to_python<std::shared_ptr<knowrob::QueryContext const>>();


	/////////////////////////////////////////////////////
	// mappings for the AnswerStream class and related classes
	/////////////////////////////////////////////////////
	class_<TokenStream::Channel, std::shared_ptr<TokenStream::Channel>, boost::noncopyable>
			("TokenChannel", no_init)
			.def("create", &TokenStream::Channel::create)
			.def("push", &TokenStream::Channel::push)
			.def("close", &TokenStream::Channel::close)
			.def("isOpened", &TokenStream::Channel::isOpened)
			.def("id", &TokenStream::Channel::id);
	class_<TokenStream, std::shared_ptr<TokenStream>, boost::noncopyable>("TokenStream", no_init)
			.def("isOpened", &TokenStream::isOpened);
	class_<TokenQueue, std::shared_ptr<TokenQueue>, bases<TokenStream>, boost::noncopyable>
			("TokenQueue", init<>())
			.def("front", &TokenQueue::front, EXISTING_REF_RETURN)
			.def("pop", &TokenQueue::pop)
			.def("pop_front", &TokenQueue::pop_front)
			.def("empty", &TokenQueue::empty)
			.def("size", &TokenQueue::size);
	class_<TokenBroadcaster, std::shared_ptr<TokenBroadcaster>, bases<TokenStream>, boost::noncopyable>
			("TokenBroadcaster", init<>())
			.def("addSubscriber", &TokenBroadcaster::addSubscriber)
			.def("removeSubscriber", &TokenBroadcaster::removeSubscriber);
	class_<TokenBuffer, std::shared_ptr<TokenBuffer>, bases<TokenBroadcaster>, boost::noncopyable>
			("TokenBuffer", init<>())
			.def("stopBuffering", &TokenBuffer::stopBuffering)
			.def("createQueue", &TokenBuffer::createQueue);

	/////////////////////////////////////////////////////
	// mappings for the data source and backend classes
	/////////////////////////////////////////////////////
	class_<DataSource, std::shared_ptr<DataSource>>("DataSource", init<std::string>())
			.def("dataFormat", &DataSource::dataFormat, CONST_REF_RETURN)
			.def("uri", &DataSource::uri, CONST_REF_RETURN)
			.def("path", &DataSource::path, CONST_REF_RETURN);
	class_<DataSourceHandler, std::shared_ptr<DataSourceHandler>>("DataSourceHandler", init<>())
			.def("addDataHandler", +[]
				(DataSourceHandler &x, const std::string &format, python::object &fn)
				{ x.addDataHandler(format, fn); })
			.def("loadDataSource", &DataSourceHandler::loadDataSource);

	class_<DataBackend, std::shared_ptr<DataBackendWrap>, bases<DataSourceHandler>, boost::noncopyable>
	        ("DataBackend", init<>())
			// methods that must be implemented by backend plugins
			.def("loadConfig", &DataBackendWrap::loadConfig)
			.def("insertOne", &DataBackendWrap::insertOne)
			.def("insertAll", &DataBackendWrap::insertAll)
			.def("removeAll", &DataBackendWrap::removeAll)
			.def("removeOne", &DataBackendWrap::removeOne)
			.def("loadConfig", &DataBackendWrap::loadConfig);

	/////////////////////////////////////////////////////
	// mappings for the Reasoner class and it sub-classes
	/////////////////////////////////////////////////////
	class_<ReasonerConfig, std::shared_ptr<ReasonerConfig>>("ReasonerConfiguration", init<>())
			.def("__iter__", range(&ReasonerConfig::begin, &ReasonerConfig::end))
			.def("get", &ReasonerConfig::get)
			.def("dataSources", &ReasonerConfig::dataSources, CONST_REF_RETURN);
	class_<Reasoner, std::shared_ptr<ReasonerWrap>, bases<DataSourceHandler>, boost::noncopyable>("Reasoner", init<>())
			.def("managerID", &ReasonerWrap::managerID)
			.def("createTriple", &ReasonerWrap::createTriple)
			.def("createTriples", &ReasonerWrap::createTriples)
			.def("pushWork", +[](Reasoner &x, python::object &fn){ x.pushWork(fn); })
			.def("setInferredTriples", &ReasonerWrap::setInferredTriples)
			.def("addInferredTriples", &ReasonerWrap::addInferredTriples)
			.def("removeInferredTriples", &ReasonerWrap::removeInferredTriples)
					// methods that must be implemented by reasoner plugins
			.def("loadConfig", &ReasonerWrap::loadConfig)
			.def("setDataBackend", &ReasonerWrap::setDataBackend)
			.def("getDescription", &ReasonerWrap::getDescription)
			.def("start", &ReasonerWrap::start)
			.def("stop", &ReasonerWrap::stop)
			.def("submitQuery", &ReasonerWrap::submitQuery);
	class_<ReasonerWithBackend, std::shared_ptr<ReasonerWithBackendWrap>, bases<Reasoner, DataBackend>, boost::noncopyable>
			("ReasonerWithBackend", init<>());

	/////////////////////////////////////////////////////
	// mappings for optionals used in the structs above
	/////////////////////////////////////////////////////
	python_optional<TemporalOperator>();
	python_optional<EpistemicOperator>();
	python_optional<double>();
	python_optional<AgentPtr>();
}
