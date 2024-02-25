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

#include "knowrob/py/converter.h"
#include "knowrob/py/utils.h"
#include "knowrob/knowrob.h"

using namespace knowrob;
namespace python = boost::python;

BOOST_PYTHON_MODULE (knowrob) {
	using namespace boost::python;
	typedef std::vector<TermPtr> TermList;
	typedef std::vector<FormulaPtr> FormulaList;
	typedef std::vector<std::shared_ptr<FramedTriple>> TripleList;

	// convert std::string_view to python::str and vice versa.
	python::to_python_converter<std::string_view, string_view_to_python_str>();
	python::converter::registry::push_back(
			&python_str_to_string_view::convertible,
			&python_str_to_string_view::construct,
			python::type_id<std::string_view>());

	/**
    boost::python::scope().attr("__doc__") = "knowrob module";
    boost::python::object submodule1(boost::python::handle<>(boost::python::borrowed(PyImport_AddModule("knowrob.submodule1"))));
    boost::python::scope().attr("submodule1") = submodule1;
    boost::python::scope submodule1_scope = submodule1;
    initialize_knowrob_submodule1();

    boost::python::object submodule2(boost::python::handle<>(boost::python::borrowed(PyImport_AddModule("knowrob.submodule2"))));
    boost::python::scope().attr("submodule2") = submodule2;
    boost::python::scope submodule2_scope = submodule2;
    initialize_knowrob_submodule2();
	 */

	/////////////////////////////////////////////////////
	// some common types
	/////////////////////////////////////////////////////
	py::createType<Logger>();
	py::createType<Agent>();
	py::createType<Modality>();
	py::createType<ModalOperator>();

	/////////////////////////////////////////////////////
	// Term and related classes
	/////////////////////////////////////////////////////
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
	py::createType<Substitution>();

	// allow conversion between std::vector and python::list for Term objects.
	custom_vector_from_seq<TermPtr>();
	class_<TermList>("TermList").def(vector_indexing_suite<TermList, true>());

	/////////////////////////////////////////////////////
	// Formula and related classes
	/////////////////////////////////////////////////////
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
	py::createType<PredicateDescription>();

	// allow conversion between std::vector and python::list for Formula objects.
	custom_vector_from_seq<FormulaPtr>();
	class_<FormulaList>("FormulaList").def(vector_indexing_suite<FormulaList, true>());

	/////////////////////////////////////////////////////
	// FramedTriple and related classes
	/////////////////////////////////////////////////////
	py::createType<FramedTriple>();
	py::createType<FramedTriplePattern>();
	py::createType<GraphSelector>();
	py::createType<semweb::TripleContainer>();

	// allow conversion between std::vector and python::list for FramedTriple objects.
	custom_vector_from_seq<std::shared_ptr<FramedTriple>>();
	class_<TripleList>("TripleList").def(vector_indexing_suite<TripleList, true>());

	/////////////////////////////////////////////////////
	// mappings for the AnswerStream class and related classes
	/////////////////////////////////////////////////////
	py::createType<TokenStream>();
	py::createType<TokenQueue>();
	py::createType<TokenBroadcaster>();
	py::createType<TokenBuffer>();
	py::createType<QueryContext>();

	/////////////////////////////////////////////////////
	// mappings for the data source and backend classes
	/////////////////////////////////////////////////////
	py::createType<DataSource>();
	py::createType<DataSourceHandler>();
	py::createType<DataBackend>();

	/////////////////////////////////////////////////////
	// mappings for the Reasoner class and it sub-classes
	/////////////////////////////////////////////////////
	py::createType<Reasoner>();

	/////////////////////////////////////////////////////
	// mappings for optionals used in the structs above
	/////////////////////////////////////////////////////
	// Note: At the moment each optional must be listed individually in the module declaration.
	//       It would be nice if this could be avoided...
	python_optional<TemporalOperator>();
	python_optional<EpistemicOperator>();
	python_optional<XSDType>();
	python_optional<std::string_view>();
	python_optional<double>();
	python_optional<AgentPtr>();
}
