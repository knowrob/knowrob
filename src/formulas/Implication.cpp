/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include "knowrob/formulas/Implication.h"
#include "knowrob/py/utils.h"

using namespace knowrob;

Implication::Implication(const FormulaPtr &antecedent, const FormulaPtr &consequent)
		: CompoundFormula(FormulaType::IMPLICATION, {antecedent, consequent}) {
}

bool Implication::isEqual(const Formula &other) const {
	const auto &x = static_cast<const Implication &>(other); // NOLINT
	return (*antecedent()) == (*x.antecedent()) &&
		   (*consequent()) == (*x.consequent());
}

namespace knowrob::py {
	template<>
	void createType<Implication>() {
		using namespace boost::python;
		class_<Implication, std::shared_ptr<Implication>, bases<CompoundFormula>>
				("Implication", init<const FormulaPtr &, const FormulaPtr &>())
				.def("antecedent", &Implication::antecedent, return_value_policy<copy_const_reference>())
				.def("consequent", &Implication::consequent, return_value_policy<copy_const_reference>());
	}
}
