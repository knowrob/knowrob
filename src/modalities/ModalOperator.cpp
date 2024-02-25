/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include "knowrob/modalities/ModalOperator.h"
#include "knowrob/modalities/Modality.h"
#include "knowrob/Logger.h"
#include "knowrob/py/utils.h"

using namespace knowrob;

ModalOperator::ModalOperator(const std::shared_ptr<Modality> &modality, ModalOperatorType operatorType)
		: modality_(modality),
		  operatorType_(operatorType) {
}

bool ModalOperator::isTransitive() const {
	return modality_->isTransitive();
}

bool ModalOperator::isEuclidean() const {
	return modality_->isEuclidean();
}

const char *ModalOperator::symbol() const {
	return operatorType_ == ModalOperatorType::NECESSITY ?
		   modality_->necessity_symbol() : modality_->possibility_symbol();
}

void ModalOperator::write(std::ostream &os) const {
	if (operatorType_ == ModalOperatorType::NECESSITY)
		os << modality_->necessity_symbol();
	else
		os << modality_->possibility_symbol();

	auto &params = modality_->parameters();
	if (!params.empty()) {
		os << '[';
		int paramIndex = 0;
		for (auto &pair: params) {
			if (paramIndex++ > 0) os << ", ";
			os << pair.first << '=' << *pair.second;
		}
		os << ']';
	}
}

bool ModalOperator::isSameModalOperator(const ModalOperator &other) const {
	return operatorType_ == other.operatorType_ && modality_ == other.modality_;
}

bool ModalOperator::isModalNecessity() const {
	return operatorType_ == ModalOperatorType::NECESSITY;
}

bool ModalOperator::isModalPossibility() const {
	return operatorType_ == ModalOperatorType::POSSIBILITY;
}

const ModalIteration &ModalIteration::emptyIteration() {
	static ModalIteration empty;
	return empty;
}

bool ModalIteration::operator==(const ModalIteration &other) const {
	if (modalitySequence_.size() != other.modalitySequence_.size()) {
		return false;
	}
	auto it = modalitySequence_.begin();
	auto jt = other.modalitySequence_.begin();
	while (it != modalitySequence_.end()) {
		if (!(*it == *jt)) {
			return false;
		}
		++it;
		++jt;
	}
	return true;
}

ModalIteration ModalIteration::operator+(const ModalOperatorPtr &modalOperator) const {
	ModalIteration it(*this);
	it += modalOperator;
	return it;
}

void ModalIteration::operator+=(const ModalOperatorPtr &next) //NOLINT
{
	if (modalitySequence_.empty()) {
		modalitySequence_.push_back(next);
	} else {
		auto &last = modalitySequence_.back();

		ModalOperatorPtr reduced;
		if (last->modality().modalityType() == next->modality().modalityType()) {
			// Axiom (4): <square>p -> <square><square>p corresponds to a transitive accessibility relation.
			// If the axioms is adopted by a modality, then iteration over the same operator
			// symbol can be omitted.
			if (last->operatorType() == next->operatorType() &&
				last->modality().isTransitive()) { reduced = last; }

			// Axiom (5): <square>p -> <diamond><square>p corresponds to a euclidean accessibility relation.
			// If the axioms is adopted by a modality, then iteration over possibility and necessity operator
			// can be reduced to the latter operator.
			if (last->operatorType() != next->operatorType() &&
				last->modality().isEuclidean()) { reduced = next; }
		}
		if (!reduced) {
			// different modalities can be reduced in case some principles governing their
			// interaction or known.
			reduced = last->modality().reduce(last, next);
		}

		if (reduced) {
			modalitySequence_.pop_back();
			*this += reduced;
		} else {
			modalitySequence_.push_back(next);
		}
	}
}

namespace knowrob::py {
	template<>
	void createType<ModalOperator>() {
		using namespace boost::python;
		enum_<ModalOperatorType>("ModalOperatorType")
				.value("NECESSITY", ModalOperatorType::NECESSITY)
				.value("POSSIBILITY", ModalOperatorType::POSSIBILITY);
		class_<ModalOperator, std::shared_ptr<ModalOperator>>
				("ModalOperator", init<const std::shared_ptr<Modality> &, ModalOperatorType>())
				.def("modality", &ModalOperator::modality, return_value_policy<copy_const_reference>())
				.def("isModalNecessity", &ModalOperator::isModalNecessity)
				.def("isModalPossibility", &ModalOperator::isModalPossibility)
				.def("operatorType", &ModalOperator::operatorType)
				.def("isTransitive", &ModalOperator::isTransitive)
				.def("isEuclidean", &ModalOperator::isEuclidean)
				.def("symbol", &ModalOperator::symbol);
		class_<ModalIteration, std::shared_ptr<ModalIteration>>
				("ModalIteration", init<>())
				.def("__eq__", &ModalIteration::operator==)
				.def("__iter__", range(&ModalIteration::begin, &ModalIteration::end))
				.def("numOperators", &ModalIteration::numOperators)
				.def("emptyIteration", &ModalIteration::emptyIteration, return_value_policy<copy_const_reference>());
	}
}
