/*
 * Copyright (c) 2022, Daniel Beßler
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include "knowrob/formulas/PredicateDescription.h"
#include "knowrob/terms/Atomic.h"

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
