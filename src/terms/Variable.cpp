/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include <utility>
#include <gtest/gtest.h>
#include "knowrob/terms/Variable.h"
#include "knowrob/py/utils.h"

using namespace knowrob;

Variable::Variable(std::string_view name)
		: Term(),
		  nameAtom_(Atom::Tabled(name)),
		  variables_({nameAtom_->stringForm()}) {
}

bool Variable::isSameVariable(const Variable &other) const {
	return *nameAtom_ == *other.nameAtom_;
}

bool Variable::operator<(const Variable &other) const {
	return (this->nameAtom_->stringForm() < other.nameAtom_->stringForm());
}

void Variable::write(std::ostream &os) const {
	os << nameAtom_->stringForm();
}

namespace knowrob::py {
	template<>
	void createType<Variable>() {
		using namespace boost::python;
		class_<Variable, std::shared_ptr<Variable>, bases<Term>>
				("Variable", init<std::string>())
				.def(self < self)
				.def("name", &Variable::name);
	}
}
