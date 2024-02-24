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
#include "knowrob/terms/ListTerm.h"
#include "knowrob/terms/Numeric.h"
#include "knowrob/terms/Blank.h"
#include "knowrob/terms/String.h"
#include "knowrob/terms/IRIAtom.h"
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
#include "knowrob/knowrob.h"

using namespace knowrob;
namespace python = boost::python;

BOOST_PYTHON_MODULE (knowrob) {
	using namespace boost::python;
	typedef std::vector<TermPtr> TermList;
	typedef std::vector<FormulaPtr> FormulaList;
	typedef std::vector<FramedTriple*> StatementList;

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
	// RDF types
	/////////////////////////////////////////////////////

	enum_<RDFNodeType>("RDFNodeType")
	        .value("BLANK", RDFNodeType::BLANK)
			.value("IRI", RDFNodeType::IRI)
			.value("LITERAL", RDFNodeType::LITERAL);

	class_<RDFNode, std::shared_ptr<RDFNodeWrap>, boost::noncopyable>("RDFNode", no_init)
			.def("rdfNodeType", python::pure_virtual(&RDFNode::rdfNodeType));

	/////////////////////////////////////////////////////
	// mappings for the Term and its subclasses
	/////////////////////////////////////////////////////
	enum_<TermType>("TermType")
			.value("FUNCTION", TermType::FUNCTION)
			.value("ATOMIC", TermType::ATOMIC)
			.value("VARIABLE", TermType::VARIABLE);

	class_<Term, std::shared_ptr<TermWrap>, boost::noncopyable>
	        ("Term", python::no_init)
			.def("__eq__", &Term::operator==)
			.def("__str__", +[](Term &t){ return readString(t); })
			.def("termType", &Term::termType)
			.def("isGround", python::pure_virtual(&Term::isGround))
			.def("isAtomic", python::pure_virtual(&Term::isAtomic))
			.def("isAtom", &Term::isAtom)
			.def("isVariable", &Term::isVariable)
			.def("isFunction", &Term::isFunction)
			.def("isNumeric", &Term::isNumeric)
			.def("isString", &Term::isString)
			.def("isIRI", &Term::isIRI)
			.def("isBlank", &Term::isBlank)
			.def("hash", python::pure_virtual(&Term::hash))
			.def("variables", python::pure_virtual(&Term::variables), CONST_REF_RETURN);
	class_<Variable, std::shared_ptr<Variable>, bases<Term>>
			("Variable", init<std::string>())
			.def(self < self)
			.def("name", &Variable::name);
	class_<Blank, std::shared_ptr<Blank>, bases<Variable,RDFNode>>
			("Blank", init<std::string_view>());
	class_<Function, std::shared_ptr<Function>, bases<Term>>("Function", init<std::string_view, const TermList &>())
			.def(init<const AtomPtr &, const TermList &>())
			.def("functor", &Function::functor, CONST_REF_RETURN)
			.def("arguments", &Function::arguments, CONST_REF_RETURN);
	class_<ListTerm, std::shared_ptr<ListTerm>, bases<Function>>("ListTerm", init<const std::vector<TermPtr> &>())
			.def("__iter__", range(&ListTerm::begin, &ListTerm::end))
			.def("isNIL", &ListTerm::isNIL)
			.def("elements", &ListTerm::elements, CONST_REF_RETURN);
	class_<Atomic, std::shared_ptr<AtomicWrap>, bases<Term>, boost::noncopyable>
			("Atomic", no_init)
			.def("stringForm", python::pure_virtual(&Atomic::stringForm))
			.def("atomicType", python::pure_virtual(&Atomic::atomicType))
			.def("isSameAtomic", &Atomic::isSameAtomic);
	class_<Atom, std::shared_ptr<Atom>, bases<Atomic>>("Atom", no_init)
			.def("Tabled", &Atom::Tabled)
			.def("atomType", &Atom::atomType)
			.def("isSameAtom", &Atom::isSameAtom);
	class_<IRIAtom, std::shared_ptr<IRIAtom>, bases<Atom,RDFNode>>("IRIAtom", no_init)
			.def("Tabled", &IRIAtom::Tabled)
			.def("rdfNodeType", &IRIAtom::rdfNodeType)
			.def("isIRI", &IRIAtom::isIRI);
	class_<XSDAtomic, std::shared_ptr<XSDAtomicWrap>, bases<Atomic,RDFNode>, boost::noncopyable>
	        ("XSDAtomic", no_init)
			.def("xsdTypeIRI", &XSDAtomic::xsdTypeIRI)
			.def("xsdType", python::pure_virtual(&XSDAtomic::xsdType));
	class_<StringBase, std::shared_ptr<StringBase>, bases<XSDAtomic>, boost::noncopyable>
			("StringTerm", no_init)
			.def("isSameString", &StringBase::isSameString);
	class_<String, std::shared_ptr<String>, bases<StringBase>>
			("String", init<std::string_view>());
	class_<StringView, std::shared_ptr<StringView>, bases<StringBase>>
			("StringView", init<std::string_view>());
	class_<Numeric, std::shared_ptr<NumericWrap>, bases<XSDAtomic>, boost::noncopyable>
	        ("Numeric", no_init)
			.def("isFloatingNumber", &Numeric::isFloatingNumber)
			.def("isSameNumeric", &Numeric::isSameNumeric)
			.def("asDouble", python::pure_virtual(&Numeric::asDouble))
			.def("asFloat", python::pure_virtual(&Numeric::asFloat))
			.def("asInteger", python::pure_virtual(&Numeric::asInteger))
			.def("asLong", python::pure_virtual(&Numeric::asLong))
			.def("asShort", python::pure_virtual(&Numeric::asShort))
			.def("asUnsignedInteger", python::pure_virtual(&Numeric::asUnsignedInteger))
			.def("asUnsignedLong", python::pure_virtual(&Numeric::asUnsignedLong))
			.def("asUnsignedShort", python::pure_virtual(&Numeric::asUnsignedShort))
			.def("asBoolean", python::pure_virtual(&Numeric::asBoolean));
	class_<Double, std::shared_ptr<Double>, bases<Numeric>>
			("Double", init<double>())
			.def(init<std::string_view>())
			.def("numericForm", &Double::numericForm);
	class_<Float, std::shared_ptr<Float>, bases<Numeric>>
			("Float", init<float>())
			.def(init<std::string_view>())
			.def("numericForm", &Float::numericForm);
	class_<Integer, std::shared_ptr<Integer>, bases<Numeric>>
			("Integer", init<int>())
			.def(init<std::string_view>())
			.def("numericForm", &Integer::numericForm);
	class_<Long, std::shared_ptr<Long>, bases<Numeric>>
			("Long", init<long>())
			.def(init<std::string_view>())
			.def("numericForm", &Long::numericForm);
	class_<Short, std::shared_ptr<Short>, bases<Numeric>>
			("Short", init<short>())
			.def(init<std::string_view>())
			.def("numericForm", &Short::numericForm);

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
	class_<ModalOperator, std::shared_ptr<ModalOperator>>
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
	class_<Predicate, std::shared_ptr<Predicate>, bases<Formula>>
			("Predicate", init<std::string_view, const TermList &>())
			.def(init<const AtomPtr &, const TermList &>())
			.def("arguments", &Predicate::arguments, CONST_REF_RETURN)
			.def("functor", &Predicate::functor, CONST_REF_RETURN);
	class_<Bottom, std::shared_ptr<Bottom>, bases<Predicate>>("Bottom", no_init)
			.def("get", &Bottom::get, CONST_REF_RETURN);
	class_<Top, std::shared_ptr<Top>, bases<Predicate>>("Top", no_init)
			.def("get", &Top::get, CONST_REF_RETURN);

	/////////////////////////////////////////////////////
	// mappings for StatementData
	/////////////////////////////////////////////////////

	class_<FramedTriple, std::shared_ptr<FramedTripleWrap>, boost::noncopyable>
	        ("FramedTriple", no_init)
	        .def("__eq__", &FramedTriple::operator==)
			.def("setSubject", python::pure_virtual(&FramedTriple::setSubject))
			.def("setPredicate", python::pure_virtual(&FramedTriple::setPredicate))
			.def("setSubjectBlank", python::pure_virtual(&FramedTriple::setSubjectBlank))
			.def("setObjectIRI", python::pure_virtual(&FramedTriple::setObjectIRI))
			.def("setObjectBlank", python::pure_virtual(&FramedTriple::setObjectBlank))
			.def("setXSDValue", &FramedTriple::setXSDValue)
			.def("setGraph", python::pure_virtual(&FramedTriple::setGraph))
			.def("setAgent", python::pure_virtual(&FramedTriple::setAgent))
			.def("setTemporalOperator", &FramedTriple::setTemporalOperator)
			.def("setEpistemicOperator", &FramedTriple::setEpistemicOperator)
			.def("setBegin", &FramedTriple::setBegin)
			.def("setEnd", &FramedTriple::setEnd)
			.def("setConfidence", &FramedTriple::setConfidence)
			.def("xsdType", &FramedTriple::xsdType)
			.def("subject", python::pure_virtual(&FramedTriple::subject))
			.def("predicate", python::pure_virtual(&FramedTriple::predicate))
			.def("graph", python::pure_virtual(&FramedTriple::graph))
			.def("agent", python::pure_virtual(&FramedTriple::agent))
			.def("temporalOperator", &FramedTriple::temporalOperator)
			.def("epistemicOperator", &FramedTriple::epistemicOperator)
			.def("begin", &FramedTriple::begin)
			.def("end", &FramedTriple::end)
			.def("confidence", &FramedTriple::confidence);

	//custom_vector_from_seq<FramedTriple*>();
	//class_<StatementList>("StatementList").def(vector_indexing_suite<StatementList, true>());

	// abstract container which is used in the DataBackend class
	class_<semweb::TripleContainer, std::shared_ptr<semweb::TripleContainer>, boost::noncopyable>
	        ("TripleContainer", no_init)
			.def("__iter__", range(&semweb::TripleContainer::begin, &semweb::TripleContainer::end));

	/////////////////////////////////////////////////////
	// mappings for literals
	/////////////////////////////////////////////////////
	class_<FirstOrderLiteral, std::shared_ptr<FirstOrderLiteral>>("FirstOrderLiteral", init<const PredicatePtr &, bool>())
			.def("predicate", &FirstOrderLiteral::predicate, CONST_REF_RETURN)
			.def("isNegated", &FirstOrderLiteral::isNegated)
			.def("functor", &FirstOrderLiteral::functor, CONST_REF_RETURN)
			.def("arity", &FirstOrderLiteral::arity)
					// FIXME: is a virtual method, might need a wrapper
			.def("numVariables", &FirstOrderLiteral::numVariables);
	class_<FramedTriplePattern, std::shared_ptr<FramedTriplePattern>, bases<FirstOrderLiteral>>
			("FramedTriplePattern", init<const FramedTriple &, bool>())
			.def(init<const FramedTriple &>())
			.def("subjectTerm", &FramedTriplePattern::subjectTerm)
			.def("propertyTerm", &FramedTriplePattern::propertyTerm)
			.def("objectTerm", &FramedTriplePattern::objectTerm)
			.def("graphTerm", &FramedTriplePattern::graphTerm)
			.def("agentTerm", &FramedTriplePattern::agentTerm)
			.def("beginTerm", &FramedTriplePattern::beginTerm)
			.def("endTerm", &FramedTriplePattern::endTerm)
			.def("confidenceTerm", &FramedTriplePattern::confidenceTerm)
			.def("objectOperator", &FramedTriplePattern::objectOperator)
			.def("temporalOperator", &FramedTriplePattern::temporalOperator)
			.def("epistemicOperator", &FramedTriplePattern::epistemicOperator)
			.def("setGraphName", &FramedTriplePattern::setGraphName)
			.def("toStatementData", &FramedTriplePattern::toStatementData);

	/////////////////////////////////////////////////////
	// mappings for the Substitution class
	/////////////////////////////////////////////////////
	// need to select one overload for get, there is another for Variable as argument
	// which currently cannot be addressed in Python.
	class_<Substitution, std::shared_ptr<Substitution>>("Substitution", init<>())
			.def(init<std::map<std::shared_ptr<Variable>, TermPtr>>())
			.def("__eq__", &Substitution::operator==)
			.def("__iter__", range(&Substitution::begin, &Substitution::end))
			.def("empty", &Substitution::empty)
			.def("set", &Substitution::set)
			.def("get", &Substitution::get, CONST_REF_RETURN)
			.def("contains", &Substitution::contains)
			.def("hash", &Substitution::hash);

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
	class_<DataSource, std::shared_ptr<DataSource>>("DataSource", no_init)
			.def("format", &DataSource::format, CONST_REF_RETURN)
			.def("uri", &DataSource::uri, CONST_REF_RETURN)
			.def("path", &DataSource::path, CONST_REF_RETURN)
			.def("version", &DataSource::version)
			.def("name", &DataSource::name);
	class_<DataSourceHandler, std::shared_ptr<DataSourceHandler>>("DataSourceHandler", init<>())
			.def("addDataHandler", +[]
				(DataSourceHandler &x, const std::string &format, python::object &fn)
				{ x.addDataHandler(format, fn); })
			.def("loadDataSource", &DataSourceHandler::loadDataSource);

	class_<DataBackend, std::shared_ptr<DataBackendWrap>, bases<DataSourceHandler>, boost::noncopyable>
	        ("DataBackend", init<>())
			// methods that must be implemented by backend plugins
			.def("initializeBackend", &DataBackendWrap::initializeBackend)
			.def("insertOne", &DataBackendWrap::insertOne)
			.def("insertAll", &DataBackendWrap::insertAll)
			.def("removeAll", &DataBackendWrap::removeAll)
			.def("removeOne", &DataBackendWrap::removeOne)
			.def("removeAllWithOrigin", &DataBackendWrap::removeAllWithOrigin)
			.def("removeAllMatching", &DataBackendWrap::removeAllMatching);

	/////////////////////////////////////////////////////
	// mappings for the Reasoner class and it sub-classes
	/////////////////////////////////////////////////////
	class_<ReasonerConfig, std::shared_ptr<ReasonerConfig>>("ReasonerConfiguration", init<>())
			.def("__iter__", range(&ReasonerConfig::begin, &ReasonerConfig::end))
			.def("get", &ReasonerConfig::get)
			.def("dataSources", &ReasonerConfig::dataSources, CONST_REF_RETURN);
	class_<Reasoner, std::shared_ptr<ReasonerWrap>, bases<DataSourceHandler>, boost::noncopyable>("Reasoner", init<>())
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
	python_optional<std::string_view>();
	python_optional<double>();
	python_optional<AgentPtr>();
}
