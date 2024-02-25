/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include "knowrob/formulas/Negation.h"
#include "knowrob/formulas/Predicate.h"
#include "knowrob/formulas/Top.h"
#include "knowrob/formulas/Bottom.h"
#include "knowrob/py/utils.h"

using namespace knowrob;

Negation::Negation(const FormulaPtr &formula)
		: CompoundFormula(FormulaType::NEGATION, {formula}) {
}

bool Negation::isEqual(const Formula &other) const {
	const auto &x = static_cast<const Negation &>(other); // NOLINT
	return (*negatedFormula()) == (*x.negatedFormula());
}

namespace knowrob {
	FormulaPtr operator~(const FormulaPtr &phi) {
		if (phi->type() == FormulaType::NEGATION) {
			return ((Negation *) phi.get())->negatedFormula();
		} else if (phi->isBottom()) {
			return Top::get();
		} else if (phi->isTop()) {
			return Bottom::get();
		} else {
			return std::make_shared<Negation>(phi);
		}
	}
}

namespace knowrob::py {
	template<>
	void createType<Negation>() {
		using namespace boost::python;
		class_<Negation, std::shared_ptr<Negation>, bases<CompoundFormula>>
				("Negation", init<const FormulaPtr &>())
				.def("negatedFormula", &Negation::negatedFormula, return_value_policy<copy_const_reference>());
	}
}
