
#ifndef KNOWROB_PY_WRAPPER_H
#define KNOWROB_PY_WRAPPER_H

#include <boost/python.hpp>
#include "knowrob/terms/Term.h"
#include "knowrob/formulas/Formula.h"
#include "knowrob/formulas/CompoundFormula.h"
#include "knowrob/reasoner/ReasonerError.h"
#include "knowrob/py/utils.h"
#include "knowrob/semweb/FramedTriple.h"
#include "knowrob/db/DataBackend.h"
#include "knowrob/terms/Numeric.h"
#include "knowrob/reasoner/Reasoner.h"

using namespace knowrob;
namespace python = boost::python;

// This header contains wrapper classes for the mapping of C++ classes to Python.
// The wrapper classes are needed because the C++ classes have pure virtual methods
// and cannot be instantiated directly.
// This header should be included in code that generates a Python module,
// it does not need to be included in the KnowRob library itself.

namespace knowrob::py {
	// call a method of a python object
	template<typename R, typename... Args> R call_method(PyObject *self, const char *method, Args... args) {
		try {
			return python::call_method<R>(self, method, python::object(args)...);
		} catch (const python::error_already_set&) {
			knowrob::py::handlePythonError();
			throw ReasonerError("Python error");
		}
	}
}

// this struct is needed because FramedTriple has pure virtual methods
struct FramedTripleWrap : public FramedTriple, boost::python::wrapper<FramedTriple> {
	explicit FramedTripleWrap(PyObject *p) : self(p), FramedTriple() {}

	void setSubject(std::string_view subject) override
	{ knowrob::py::call_method<void>(self, "setSubject", subject); }

	void setPredicate(std::string_view predicate) override
	{ knowrob::py::call_method<void>(self, "setPredicate", predicate); }

	void setObjectIRI(std::string_view object) override
	{ knowrob::py::call_method<void>(self, "setObjectIRI", object); }

	void setSubjectBlank(std::string_view str) override
	{ knowrob::py::call_method<void>(self, "setSubjectBlank", str); }

	void setObjectBlank(std::string_view str) override
	{ knowrob::py::call_method<void>(self, "setObjectBlank", str); }

	std::string_view subject() const override
	{ return knowrob::py::call_method<std::string_view>(self, "subject"); }

	std::string_view predicate() const override
	{ return knowrob::py::call_method<std::string_view>(self, "predicate"); }

	void setGraph(std::string_view graph) override
	{ knowrob::py::call_method<void>(self, "setGraph", graph); }

	void setAgent(std::string_view agent) override
	{ knowrob::py::call_method<void>(self, "setAgent", agent); }

	std::optional<std::string_view> graph() const override
	{ return knowrob::py::call_method<std::optional<std::string_view>>(self, "graph"); }

	std::optional<std::string_view> agent() const override
	{ return knowrob::py::call_method<std::optional<std::string_view>>(self, "agent"); }
private:
	PyObject *self;
};

// this struct is needed because RDFNode has pure virtual methods
struct RDFNodeWrap : public RDFNode, boost::python::wrapper<RDFNode> {
	explicit RDFNodeWrap(PyObject *p) : self(p), RDFNode() {}

	RDFNodeType rdfNodeType() const override
	{ return knowrob::py::call_method<RDFNodeType>(self, "rdfNodeType"); }
private:
	PyObject *self;
};

// this struct is needed because Term has pure virtual methods
struct TermWrap : public Term, boost::python::wrapper<Term> {
	explicit TermWrap(PyObject *p) : self(p), Term() {}

	bool isAtomic() const override
	{ return knowrob::py::call_method<bool>(self, "isAtomic"); }

	TermType termType() const override
	{ return knowrob::py::call_method<TermType>(self, "termType"); }

	size_t hash() const override
	{ return knowrob::py::call_method<size_t>(self, "hash"); }

	const std::set<std::string_view>& variables() const override
	{ return knowrob::py::call_method<std::set<std::string_view>&>(self, "variables"); }
private:
	PyObject *self;
};

// this struct is needed because Atomic has pure virtual methods
struct AtomicWrap : public Atomic, boost::python::wrapper<Atomic> {
	explicit AtomicWrap(PyObject *p) : self(p), Atomic() {}

	AtomicType atomicType() const override
	{ return knowrob::py::call_method<AtomicType>(self, "atomicType"); }

	std::string_view stringForm() const override
	{ return knowrob::py::call_method<std::string_view>(self, "stringForm"); }
private:
	PyObject *self;
};

// this struct is needed because XSDAtomic has pure virtual methods
struct XSDAtomicWrap : public XSDAtomic, boost::python::wrapper<XSDAtomic> {
	explicit XSDAtomicWrap(PyObject *p) : self(p), XSDAtomic() {}

	XSDType xsdType() const override
	{ return knowrob::py::call_method<XSDType>(self, "xsdType"); }
private:
	PyObject *self;
};

// this struct is needed because Numeric has pure virtual methods
struct NumericWrap : public Numeric, boost::python::wrapper<Numeric> {
	explicit NumericWrap(PyObject *p) : self(p), Numeric() {}

	double asDouble() const override
	{ return knowrob::py::call_method<double>(self, "asDouble"); }

	float asFloat() const override
	{ return knowrob::py::call_method<float>(self, "asFloat"); }

	int asInteger() const override
	{ return knowrob::py::call_method<int>(self, "asInteger"); }

	long asLong() const override
	{ return knowrob::py::call_method<long>(self, "asLong"); }

	short asShort() const override
	{ return knowrob::py::call_method<short>(self, "asShort"); }

	unsigned int asUnsignedInteger() const override
	{ return knowrob::py::call_method<unsigned int>(self, "asUnsignedInteger"); }

	unsigned long asUnsignedLong() const override
	{ return knowrob::py::call_method<unsigned long>(self, "asUnsignedLong"); }

	unsigned short asUnsignedShort() const override
	{ return knowrob::py::call_method<unsigned short>(self, "asUnsignedShort"); }

	bool asBoolean() const override
	{ return knowrob::py::call_method<bool>(self, "asBoolean"); }
private:
	PyObject *self;
};

// this struct is needed because Formula has pure virtual methods
struct FormulaWrap : public Formula, boost::python::wrapper<Formula>
{
	explicit FormulaWrap(FormulaType type) : Formula(type) {}

	bool isGround() const override 					{ return this->get_override("isGround")(); }
	void write(std::ostream& os) const override 	{ this->get_override("write")(os); }

	// protected:
	bool isEqual(const Formula &other) const override { return this->get_override("isEqual")(other); }
};

// this struct is needed because Compound has pure virtual methods
struct CompoundFormulaWrap : public CompoundFormula, boost::python::wrapper<CompoundFormula>
{
	explicit CompoundFormulaWrap(FormulaType type, const std::vector<FormulaPtr> &sub_formulas)
		: CompoundFormula(type, sub_formulas) {}
	const char* operator_symbol() const override { return this->get_override("write")(); }
};

struct DataBackendWrap : public DataBackend, boost::python::wrapper<DataBackend>
{
	explicit DataBackendWrap(PyObject *p) : self(p), DataBackend() {}

	bool initializeBackend(const ReasonerConfig &config) override
	{ return knowrob::py::call_method<bool>(self, "initializeBackend", config); }

	bool insertOne(const FramedTriple &triple) override
	{ return knowrob::py::call_method<bool>(self, "insertOne", &triple); }

	bool insertAll(const semweb::TripleContainerPtr &triples) override
	{ return knowrob::py::call_method<bool>(self, "insertAll", triples); }

	bool removeOne(const FramedTriple &triple) override
	{ return knowrob::py::call_method<bool>(self, "removeOne", &triple); }

	bool removeAll(const semweb::TripleContainerPtr &triples) override
	{ return knowrob::py::call_method<bool>(self, "removeAll", triples); }

	bool removeAllWithOrigin(std::string_view origin) override
	{ return knowrob::py::call_method<bool>(self, "removeAllWithOrigin", origin.data()); }

	bool removeAllMatching(const FramedTriplePatternPtr &query) override
	{ return knowrob::py::call_method<int>(self, "removeAllMatching", query); }

private:
	PyObject *self;
};

// this struct is needed because Reasoner has pure virtual methods
struct ReasonerWrap : public Reasoner, boost::python::wrapper<Reasoner>
{
	explicit ReasonerWrap(PyObject *p) : self(p), Reasoner() {}

	void setDataBackend(const DataBackendPtr &backend) override
	{ knowrob::py::call_method<void>(self, "setDataBackend", backend); }

	bool loadConfig(const ReasonerConfig &config) override
	{ return knowrob::py::call_method<bool>(self, "loadConfig", config); }

	PredicateDescriptionPtr getDescription(const PredicateIndicatorPtr &indicator) override
	{ return knowrob::py::call_method<PredicateDescriptionPtr>(self, "getDescription", indicator); }

	TokenBufferPtr submitQuery(const FramedTriplePatternPtr &literal, const QueryContextPtr &ctx) override
	{ return knowrob::py::call_method<TokenBufferPtr>(self, "submitQuery", literal, ctx); }

	void start() override { knowrob::py::call_method<void>(self, "start"); }
	void stop() override { knowrob::py::call_method<void>(self, "stop"); }

private:
	PyObject *self;
};

struct ReasonerWithBackendWrap :
		public ReasonerWithBackend,
		boost::python::wrapper<ReasonerWithBackend>
{
	explicit ReasonerWithBackendWrap(PyObject *p)
	: self(p), ReasonerWithBackend() {}

	// TODO: below duplicates code of ReasonerWrap and DataBackendWrap.
	//    I could not find a way to make it work with multiple inheritance, maybe
	//    because of the boost::python::wrapper superclass.

	bool loadConfig(const ReasonerConfig &config) override
	{ return knowrob::py::call_method<bool>(self, "loadConfig", config); }

	bool initializeBackend(const ReasonerConfig &config) override
	{ return knowrob::py::call_method<bool>(self, "initializeBackend", config); }

	bool insertOne(const FramedTriple &triple) override
	{ return knowrob::py::call_method<bool>(self, "insertOne", &triple); }

	bool insertAll(const semweb::TripleContainerPtr &triples) override
	{ return knowrob::py::call_method<bool>(self, "insertAll", triples); }

	bool removeOne(const FramedTriple &triple) override
	{ return knowrob::py::call_method<bool>(self, "removeOne", &triple); }

	bool removeAll(const semweb::TripleContainerPtr &triples) override
	{ return knowrob::py::call_method<bool>(self, "removeAll", triples); }

	bool removeAllWithOrigin(std::string_view origin) override
	{ return knowrob::py::call_method<bool>(self, "removeAllWithOrigin", origin.data()); }

	bool removeAllMatching(const FramedTriplePatternPtr &query) override
	{ return knowrob::py::call_method<int>(self, "removeAllMatching", query); }

	PredicateDescriptionPtr getDescription(const PredicateIndicatorPtr &indicator) override
	{ return knowrob::py::call_method<PredicateDescriptionPtr>(self, "getDescription", indicator); }

	TokenBufferPtr submitQuery(const FramedTriplePatternPtr &literal, const QueryContextPtr &ctx) override
	{ return knowrob::py::call_method<TokenBufferPtr>(self, "submitQuery", literal, ctx); }

	void start() override { knowrob::py::call_method<void>(self, "start"); }
	void stop() override { knowrob::py::call_method<void>(self, "stop"); }
private:
	PyObject *self;
};

#endif //KNOWROB_PY_WRAPPER_H
