/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include "knowrob/formulas/PredicateDescription.h"
#include "knowrob/terms/Atomic.h"
#include "knowrob/py/utils.h"

using namespace knowrob;

PredicateDescription::PredicateDescription(
		const std::shared_ptr<PredicateIndicator> &indicator,
		PredicateType type,
		MaterializationStrategy materializationStrategy)
		: indicator_(indicator),
		  type_(type),
		  materializationStrategy_(materializationStrategy) {
}

namespace knowrob {
	PredicateType predicateTypeFromTerm(const TermPtr &term) {
		auto type_string = ((Atomic *) term.get())->stringForm();
		if (type_string == "relation") return PredicateType::IDB_RELATION;
		else if (type_string == "idb_relation") return PredicateType::IDB_RELATION;
		else if (type_string == "edb_relation") return PredicateType::EDB_RELATION;
		else return PredicateType::BUILT_IN;
	}
}

namespace knowrob::py {
	template<>
	void createType<PredicateDescription>() {
		using namespace boost::python;
		class_<PredicateDescription, std::shared_ptr<PredicateDescription>>
				("PredicateDescription", init<
						const std::shared_ptr<PredicateIndicator> &,
						PredicateType,
						MaterializationStrategy>())
				.def("indicator", &PredicateDescription::indicator, return_value_policy<copy_const_reference>())
				.def("type", &PredicateDescription::type)
				.def("materializationStrategy", &PredicateDescription::materializationStrategy);
	}
}
