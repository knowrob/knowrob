/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include "knowrob/formulas/ModalOperator.h"
#include "knowrob/Logger.h"
#include "knowrob/py/utils.h"
#include "knowrob/terms/Numeric.h"

using namespace knowrob;

ModalOperator::ModalOperator(ModalType modalType)
		: modalType_(modalType) {
}

const char *ModalOperator::symbol() const {
	switch (modalType_) {
		case ModalType::KNOWLEDGE:
			return "K";
		case ModalType::BELIEF:
			return "B";
		case ModalType::ALWAYS:
			return "H";
		case ModalType::SOMETIMES:
			return "P";
	}
	return "K";
}

void ModalOperator::write(std::ostream &os) const {
	os << symbol();

	if (!parameters_.empty()) {
		os << '[';
		int paramIndex = 0;
		for (auto &pair: parameters_) {
			if (paramIndex++ > 0) os << ", ";
			os << pair.first << '=' << *pair.second;
		}
		os << ']';
	}
}

bool ModalOperator::operator==(const ModalOperator &other) const {
	if (modalType_ != other.modalType_) {
		return false;
	}
	if (parameters_.size() != other.parameters_.size()) {
		return false;
	}
	for (auto &pair: parameters_) {
		auto it = other.parameters_.find(pair.first);
		if (it == other.parameters_.end() || *it->second != *pair.second) {
			return false;
		}
	}
	return true;
}

bool ModalOperator::isModalNecessity() const {
	switch (modalType_) {
		case ModalType::KNOWLEDGE:
		case ModalType::ALWAYS:
			return true;
		case ModalType::SOMETIMES:
		case ModalType::BELIEF:
			return false;
	}
	return true;
}

bool ModalOperator::isModalPossibility() const {
	return !isModalNecessity();
}

void ModalOperator::setParameter(std::string_view key, const TermPtr &value) {
	parameters_[key.data()] = value;
}

std::optional<TermPtr> ModalOperator::parameter(std::string_view key) const {
	auto it = parameters_.find(key.data());
	if (it == parameters_.end()) {
		return std::nullopt;
	}
	return it->second;
}

std::optional<PerspectivePtr> ModalOperator::perspective() const {
	auto it = parameters_.find(KEY_PERSPECTIVE);
	if (it != parameters_.end() && it->second->isAtom()) {
		return Perspective::get(std::static_pointer_cast<Atomic>(it->second)->stringForm());
	}
	return std::nullopt;
}

void ModalOperator::setPerspective(const std::string_view &agent) {
	parameters_[KEY_PERSPECTIVE] = Atom::Tabled(agent);
}

std::optional<double> ModalOperator::confidence() const {
	auto it = parameters_.find(KEY_CONFIDENCE);
	if (it != parameters_.end() && it->second->isNumeric()) {
		return std::static_pointer_cast<Numeric>(it->second)->asDouble();
	}
	return std::nullopt;
}

void ModalOperator::setConfidence(double confidence) {
	parameters_[KEY_CONFIDENCE] = std::make_shared<Double>(confidence);
}

std::optional<double> ModalOperator::begin() const {
	auto it = parameters_.find(KEY_BEGIN);
	if (it != parameters_.end() && it->second->isNumeric()) {
		return std::static_pointer_cast<Numeric>(it->second)->asDouble();
	}
	return std::nullopt;
}

void ModalOperator::setBegin(double begin) {
	parameters_[KEY_BEGIN] = std::make_shared<Double>(begin);
}

std::optional<double> ModalOperator::end() const {
	auto it = parameters_.find(KEY_END);
	if (it != parameters_.end() && it->second->isNumeric()) {
		return std::static_pointer_cast<Numeric>(it->second)->asDouble();
	}
	return std::nullopt;
}

void ModalOperator::setEnd(double end) {
	parameters_[KEY_END] = std::make_shared<Double>(end);
}

void ModalOperator::setTimeInterval(const TimeInterval &timeInterval) {
	if (timeInterval.since()) {
		setBegin(time::toSeconds(*timeInterval.since()));
	}
	if (timeInterval.until()) {
		setEnd(time::toSeconds(*timeInterval.until()));
	}
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

void ModalIteration::operator+=(const ModalOperatorPtr &next) {
	modalitySequence_.push_back(next);
}

namespace knowrob::py {
	template<>
	void createType<ModalOperator>() {
		using namespace boost::python;
		enum_<ModalType>("ModalType")
				.value("KNOWLEDGE", ModalType::KNOWLEDGE)
				.value("BELIEF", ModalType::BELIEF)
				.value("ALWAYS", ModalType::ALWAYS)
				.value("SOMETIMES", ModalType::SOMETIMES);
		class_<ModalOperator, std::shared_ptr<ModalOperator>>
				("ModalOperator", init<ModalType>())
				.def("isModalNecessity", &ModalOperator::isModalNecessity)
				.def("isModalPossibility", &ModalOperator::isModalPossibility)
				.def("modalType", &ModalOperator::modalType)
				.def("symbol", &ModalOperator::symbol);
		class_<ModalIteration, std::shared_ptr<ModalIteration>>
				("ModalIteration", init<>())
				.def("__eq__", &ModalIteration::operator==)
				.def("__iter__", range(&ModalIteration::begin, &ModalIteration::end))
				.def("numOperators", &ModalIteration::numOperators)
				.def("emptyIteration", &ModalIteration::emptyIteration, return_value_policy<copy_const_reference>());
	}
}
