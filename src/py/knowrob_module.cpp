/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

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
#include "knowrob/formulas/PredicateIndicator.h"
#include "knowrob/queries/QueryContext.h"
#include "knowrob/reasoner/Reasoner.h"
#include "knowrob/reasoner/DataDrivenReasoner.h"
#include "knowrob/reasoner/GoalDrivenReasoner.h"

#include "knowrob/py/converter.h"
#include "knowrob/py/utils.h"
#include "knowrob/knowrob.h"

using namespace knowrob;

static inline void register_common_types() {
	py::createType<Logger>();
	py::createType<Perspective>();
	py::createType<ModalOperator>();
	py::createType<PropertyTree>();
}

static inline void register_term_types() {
	py::createType<RDFNode>();
	py::createType<Term>();
	py::createType<Variable>();
	py::createType<Function>();
	py::createType<ListTerm>();
	py::createType<Atomic>();
	py::createType<Atom>();
	py::createType<Blank>();
	py::createType<IRIAtom>();
	py::createType<XSDAtomic>();
	py::createType<String>();
	py::createType<Numeric>();
	py::createType<Bindings>();
	// allow conversion between std::vector and python::list for Term objects.
	typedef std::vector<TermPtr> TermList;
	py::custom_vector_from_seq<TermPtr>();
	boost::python::class_<TermList>("TermList").def(boost::python::vector_indexing_suite<TermList, true>());
}

static inline void register_formula_types() {
	py::createType<Formula>();
	py::createType<CompoundFormula>();
	py::createType<Negation>();
	py::createType<Conjunction>();
	py::createType<Disjunction>();
	py::createType<Implication>();
	py::createType<Predicate>();
	py::createType<Bottom>();
	py::createType<Top>();
	py::createType<ModalFormula>();
	py::createType<FirstOrderLiteral>();
	py::createType<PredicateIndicator>();
	// allow conversion between std::vector and python::list for Formula objects.
	typedef std::vector<FormulaPtr> FormulaList;
	py::custom_vector_from_seq<FormulaPtr>();
	boost::python::class_<FormulaList>("FormulaList").def(boost::python::vector_indexing_suite<FormulaList, true>());
}

static inline void register_triple_types() {
	py::createType<FramedTriple>();
	py::createType<FramedTriplePattern>();
	py::createType<GraphSelector>();
	py::createType<TripleContainer>();
	// allow conversion between std::vector and python::list for FramedTriple objects.
	typedef std::vector<std::shared_ptr<FramedTriple>> TripleList;
	py::custom_vector_from_seq<std::shared_ptr<FramedTriple>>();
	boost::python::class_<TripleList>("TripleList").def(boost::python::vector_indexing_suite<TripleList, true>());
}

static inline void register_query_types() {
	py::createType<TokenStream>();
	py::createType<TokenQueue>();
	py::createType<TokenBroadcaster>();
	py::createType<TokenBuffer>();
	py::createType<QueryContext>();
}

static inline void register_db_types() {
	py::createType<DataSource>();
	py::createType<DataSourceHandler>();
	py::createType<DataBackend>();
}

static inline void register_reasoner_types() {
	py::createType<Reasoner>();
	py::createType<DataDrivenReasoner>();
	py::createType<GoalDrivenReasoner>();
}

BOOST_PYTHON_MODULE (knowrob) {
	using namespace boost::python;
	using namespace knowrob::py;

	// convert std::string_view to python::str and vice versa.
	register_string_view_converter();

    //boost::python::scope().attr("__doc__") = "knowrob module";
	//boost::python::object submodule1(boost::python::handle<>(boost::python::borrowed(PyImport_AddModule("knowrob.submodule1"))));
    //boost::python::scope().attr("submodule1") = submodule1;
    //boost::python::scope submodule1_scope = submodule1;
    //initialize_knowrob_submodule1();

	/////////////////////////////////////////////////////
	// mappings for KnowRob types
	/////////////////////////////////////////////////////
	register_common_types();
	register_term_types();
	register_formula_types();
	register_triple_types();
	register_query_types();
	register_db_types();
	register_reasoner_types();

	/////////////////////////////////////////////////////
	// mappings for optionals used in the structs above
	/////////////////////////////////////////////////////
	// Note: At the moment each optional must be listed individually in the module declaration.
	//       It would be nice if this could be avoided...
	python_optional<XSDType>();
	python_optional<std::string_view>();
	python_optional<double>();
	python_optional<PerspectivePtr>();
}
