/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include "knowrob/KnowledgeBase.h"
#include "knowrob/reasoner/GoalDrivenReasoner.h"
#include "knowrob/py/utils.h"

using namespace knowrob;

PredicateDescriptionPtr GoalDrivenReasoner::getLiteralDescription(const FramedTriplePattern &literal) {
	if (literal.propertyTerm() && literal.propertyTerm()->termType() == TermType::ATOMIC) {
		auto p = std::static_pointer_cast<Atomic>(literal.propertyTerm());
		return getDescription(std::make_shared<PredicateIndicator>(p->stringForm().data(), 2));
	} else {
		return {};
	}
}

namespace knowrob::py {
	// this struct is needed because Reasoner has pure virtual methods
	struct GoalDrivenReasonerWrap : public GoalDrivenReasoner, boost::python::wrapper<GoalDrivenReasoner> {
		explicit GoalDrivenReasonerWrap(PyObject *p) : self(p), GoalDrivenReasoner() {}

		void setDataBackend(const DataBackendPtr &backend) override {
			call_method<void>(self, "setDataBackend", backend);
		}

		bool initializeReasoner(const PropertyTree &config) override {
			return call_method<bool>(self, "initializeReasoner", config);
		}

		PredicateDescriptionPtr getDescription(const PredicateIndicatorPtr &indicator) override {
			return call_method<PredicateDescriptionPtr>(self, "getDescription", indicator);
		}

		TokenBufferPtr submitQuery(const FramedTriplePatternPtr &literal, const QueryContextPtr &ctx) override {
			return call_method<TokenBufferPtr>(self, "submitQuery", literal, ctx);
		}

	private:
		PyObject *self;
	};

	template<>
	void createType<GoalDrivenReasoner>() {
		using namespace boost::python;
		class_<GoalDrivenReasoner, std::shared_ptr<GoalDrivenReasonerWrap>, bases<Reasoner>, boost::noncopyable>
				("GoalDrivenReasoner", init<>())
				// methods that must be implemented by reasoner plugins
				.def("getDescription", &GoalDrivenReasonerWrap::getDescription)
				.def("submitQuery", &GoalDrivenReasonerWrap::submitQuery);
	}
}
