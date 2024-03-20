/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include <utility>

#include "knowrob/formulas/ModalFormula.h"
#include "knowrob/py/utils.h"
#include "knowrob/terms/Numeric.h"

using namespace knowrob;

ModalFormula::ModalFormula(ModalOperatorPtr modalOperator, const FormulaPtr &formula)
		: CompoundFormula(FormulaType::MODAL, {formula}),
		  modalOperator_(std::move(modalOperator)) {
}

bool ModalFormula::isEqual(const Formula &other) const {
	const auto &x = static_cast<const ModalFormula &>(other); // NOLINT
	return (*modalOperator() == *x.modalOperator()) &&  (*modalFormula()) == (*x.modalFormula());
}

bool ModalFormula::isModalNecessity() const {
	return modalOperator_->isModalNecessity();
}

const char *ModalFormula::operator_symbol() const {
	return modalOperator_->symbol();
}

void ModalFormula::write(std::ostream &os) const {
	modalOperator_->write(os);
	os << *(formulae_[0].get());
}

namespace knowrob::modals {
	std::shared_ptr<ModalFormula> B(const FormulaPtr &phi) {
		return std::make_shared<ModalFormula>(B(), phi);
	}

	std::shared_ptr<ModalFormula> K(const FormulaPtr &phi) {
		return std::make_shared<ModalFormula>(K(), phi);
	}

	std::shared_ptr<ModalFormula> P(const FormulaPtr &phi) {
		return std::make_shared<ModalFormula>(P(), phi);
	}

	std::shared_ptr<ModalFormula> H(const FormulaPtr &phi) {
		return std::make_shared<ModalFormula>(H(), phi);
	}

	ModalOperatorPtr K() {
		static auto k_withoutArgs = std::make_shared<ModalOperator>(ModalType::KNOWLEDGE);
		return k_withoutArgs;
	}

	ModalOperatorPtr K(std::string_view perspective) {
		auto op = std::make_shared<ModalOperator>(ModalType::KNOWLEDGE);
		op->setPerspective(perspective);
		return op;
	}

	ModalOperatorPtr B() {
		static auto beliefOperator = std::make_shared<ModalOperator>(ModalType::BELIEF);
		return beliefOperator;
	}

	ModalOperatorPtr B(std::string_view perspective) {
		auto op = std::make_shared<ModalOperator>(ModalType::BELIEF);
		op->setPerspective(perspective);
		return op;
	}

	ModalOperatorPtr B(std::string_view perspective, double confidence) {
		auto op = std::make_shared<ModalOperator>(ModalType::BELIEF);
		op->setPerspective(perspective);
		op->setConfidence(confidence);
		return op;
	}

	ModalOperatorPtr B(double confidence) {
		auto op = std::make_shared<ModalOperator>(ModalType::BELIEF);
		op->setConfidence(confidence);
		return op;
	}

	ModalOperatorPtr P() {
		static auto op = std::make_shared<ModalOperator>(ModalType::SOMETIMES);
		return op;
	}

	ModalOperatorPtr P(const TimeInterval &timeInterval) {
		auto op = std::make_shared<ModalOperator>(ModalType::SOMETIMES);
		op->setTimeInterval(timeInterval);
		return op;
	}

	ModalOperatorPtr H() {
		static auto op = std::make_shared<ModalOperator>(ModalType::ALWAYS);
		return op;
	}

	ModalOperatorPtr H(const TimeInterval &timeInterval) {
		auto op = std::make_shared<ModalOperator>(ModalType::ALWAYS);
		op->setTimeInterval(timeInterval);
		return op;
	}
} // knowrob::modals

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
