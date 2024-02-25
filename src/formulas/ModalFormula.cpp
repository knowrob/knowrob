/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include <utility>

#include "knowrob/formulas/ModalFormula.h"
#include "knowrob/modalities/BeliefModality.h"
#include "knowrob/modalities/KnowledgeModality.h"
#include "knowrob/modalities/PastModality.h"
#include "knowrob/py/utils.h"

using namespace knowrob;

ModalFormula::ModalFormula(const ModalOperatorPtr &modalOperator, const FormulaPtr &formula)
		: CompoundFormula(FormulaType::MODAL, {formula}),
		  modalOperator_(modalOperator) {
}

bool ModalFormula::isEqual(const Formula &other) const {
	const auto &x = static_cast<const ModalFormula &>(other); // NOLINT
	return (modalOperator()->isSameModalOperator(*x.modalOperator())) &&
		   (*modalFormula()) == (*x.modalFormula());
}

bool ModalFormula::isModalNecessity() const {
	return modalOperator_->isModalNecessity();
}

const char *ModalFormula::operator_symbol() const {
	if (isModalNecessity()) {
		return modalOperator_->modality().necessity_symbol();
	} else {
		return modalOperator_->modality().possibility_symbol();
	}
}

void ModalFormula::write(std::ostream &os) const {
	modalOperator_->write(os);
	os << *(formulae_[0].get());
}

namespace knowrob::modality {
	std::shared_ptr<ModalFormula> B(const FormulaPtr &phi) {
		return std::make_shared<ModalFormula>(BeliefModality::B(), phi);
	}

	std::shared_ptr<ModalFormula> K(const FormulaPtr &phi) {
		return std::make_shared<ModalFormula>(KnowledgeModality::K(), phi);
	}

	std::shared_ptr<ModalFormula> P(const FormulaPtr &phi) {
		return std::make_shared<ModalFormula>(PastModality::P(), phi);
	}

	std::shared_ptr<ModalFormula> H(const FormulaPtr &phi) {
		return std::make_shared<ModalFormula>(PastModality::H(), phi);
	}
} // knowrob::modality

namespace knowrob::py {
	template<>
	void createType<ModalFormula>() {
		using namespace boost::python;
		class_<ModalFormula, std::shared_ptr<ModalFormula>, bases<CompoundFormula>>
				("ModalFormula", init<const ModalOperatorPtr &, const FormulaPtr &>())
				.def("modalOperator", &ModalFormula::modalOperator, return_value_policy<copy_const_reference>())
				.def("modalFormula", &ModalFormula::modalFormula, return_value_policy<copy_const_reference>())
				.def("isModalPossibility", &ModalFormula::isModalPossibility)
				.def("isModalNecessity", &ModalFormula::isModalNecessity);
	}
}
