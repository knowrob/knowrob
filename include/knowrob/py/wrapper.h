
#ifndef KNOWROB_PY_WRAPPER_H
#define KNOWROB_PY_WRAPPER_H

#include <boost/python.hpp>
#include "knowrob/terms/Term.h"
#include "knowrob/formulas/Formula.h"
#include "knowrob/formulas/CompoundFormula.h"
#include "knowrob/reasoner/ReasonerError.h"
#include "knowrob/py/utils.h"

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

// this struct is needed because Term has pure virtual methods
struct TermWrap : public Term, boost::python::wrapper<Term>
{
	explicit TermWrap(TermType type) : Term(type) {}

	bool isGround() const override 					{ return this->get_override("isGround")(); }
	bool isAtomic() const override 					{ return this->get_override("isAtomic")(); }
	size_t computeHash() const override 			{ return this->get_override("computeHash")(); }
	void write(std::ostream& os) const override 	{ this->get_override("write")(os); }
	const VariableSet& getVariables() override 		{ return this->get_override("getVariables")(); }
	// protected:
	bool isEqual(const Term &other) const override	{ return this->get_override("isEqual")(other); }
};

// this struct is needed because Formula has pure virtual methods
struct FormulaWrap : public Formula, boost::python::wrapper<Formula>
{
	explicit FormulaWrap(FormulaType type) : Formula(type) {}

	bool isGround() const override 					{ return this->get_override("isGround")(); }
	void write(std::ostream& os) const override 	{ this->get_override("write")(os); }

	std::shared_ptr<Formula> applySubstitution(const Substitution &substitution) const override
	{ return this->get_override("applySubstitution")(substitution); }

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

	bool loadConfig(const ReasonerConfig &config) override
	{ return knowrob::py::call_method<bool>(self, "loadConfig", config); }

	bool insertOne(const StatementData &triple) override
	{ return knowrob::py::call_method<bool>(self, "insertOne", triple); }

	bool insertAll(const std::vector<StatementData> &triples) override
	{ return knowrob::py::call_method<bool>(self, "insertAll", triples); }

	bool removeOne(const StatementData &triple) override
	{ return knowrob::py::call_method<bool>(self, "removeOne", triple); }

	bool removeAll(const std::vector<StatementData> &triples) override
	{ return knowrob::py::call_method<bool>(self, "removeOne", triples); }

	int removeMatching(const RDFLiteral &query, bool doMatchMany) override
	{ return knowrob::py::call_method<int>(self, "removeMatching", query, doMatchMany); }

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

	TokenBufferPtr submitQuery(const RDFLiteralPtr &literal, const QueryContextPtr &ctx) override
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

	bool insertOne(const StatementData &triple) override
	{ return knowrob::py::call_method<bool>(self, "insertOne", triple); }

	bool insertAll(const std::vector<StatementData> &triples) override
	{ return knowrob::py::call_method<bool>(self, "insertAll", triples); }

	bool removeOne(const StatementData &triple) override
	{ return knowrob::py::call_method<bool>(self, "removeOne", triple); }

	bool removeAll(const std::vector<StatementData> &triples) override
	{ return knowrob::py::call_method<bool>(self, "removeOne", triples); }

	int removeMatching(const RDFLiteral &query, bool doMatchMany) override
	{ return knowrob::py::call_method<int>(self, "removeMatching", query, doMatchMany); }

	PredicateDescriptionPtr getDescription(const PredicateIndicatorPtr &indicator) override
	{ return knowrob::py::call_method<PredicateDescriptionPtr>(self, "getDescription", indicator); }

	TokenBufferPtr submitQuery(const RDFLiteralPtr &literal, const QueryContextPtr &ctx) override
	{ return knowrob::py::call_method<TokenBufferPtr>(self, "submitQuery", literal, ctx); }

	void start() override { knowrob::py::call_method<void>(self, "start"); }
	void stop() override { knowrob::py::call_method<void>(self, "stop"); }
private:
	PyObject *self;
};

#endif //KNOWROB_PY_WRAPPER_H
