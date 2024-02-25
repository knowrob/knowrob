/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include <knowrob/formulas/Formula.h>
#include "knowrob/formulas/Top.h"
#include "knowrob/formulas/Bottom.h"
#include "knowrob/py/utils.h"

using namespace knowrob;

Formula::Formula(const FormulaType &type)
		: type_(type) {}

bool Formula::operator==(const Formula &other) const {
	// note: isEqual can safely perform static cast as type id's do match
	return typeid(*this) == typeid(other) && isEqual(other);
}

bool Formula::isAtomic() const {
	return type() == FormulaType::PREDICATE;
}

bool Formula::isBottom() const {
	return (this == Bottom::get().get());
}

bool Formula::isTop() const {
	return (this == Top::get().get());
}

bool FormulaLabel::operator==(const FormulaLabel &other) {
	return typeid(*this) == typeid(other) && isEqual(other);
}

namespace std {
	std::ostream &operator<<(std::ostream &os, const knowrob::Formula &phi) //NOLINT
	{
		phi.write(os);
		return os;
	}
}

namespace knowrob::py {
	// this struct is needed because Formula has pure virtual methods
	struct FormulaWrap : public Formula, boost::python::wrapper<Formula> {
		explicit FormulaWrap(FormulaType type) : Formula(type) {}

		bool isGround() const override { return this->get_override("isGround")(); }

		void write(std::ostream &os) const override { this->get_override("write")(os); }

		// protected:
		bool isEqual(const Formula &other) const override { return this->get_override("isEqual")(other); }
	};

	template<>
	void createType<Formula>() {
		using namespace boost::python;
		enum_<FormulaType>("FormulaType")
				.value("PREDICATE", FormulaType::PREDICATE)
				.value("CONJUNCTION", FormulaType::CONJUNCTION)
				.value("DISJUNCTION", FormulaType::DISJUNCTION)
				.value("NEGATION", FormulaType::NEGATION)
				.value("IMPLICATION", FormulaType::IMPLICATION)
				.value("MODAL", FormulaType::MODAL);
		class_<Formula, std::shared_ptr<FormulaWrap>, boost::noncopyable>
				("Formula", no_init)
				.def("type", &Formula::type)
				.def("__eq__", &Formula::operator==)
				.def("isGround", pure_virtual(&Formula::isGround))
				.def("write", pure_virtual(&Formula::write))
				.def("isAtomic", &Formula::isAtomic)
				.def("isTop", &Formula::isTop)
				.def("isBottom", &Formula::isBottom);
	}
}
