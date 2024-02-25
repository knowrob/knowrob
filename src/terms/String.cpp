/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include "knowrob/terms/String.h"
#include "knowrob/py/utils.h"

using namespace knowrob;

bool StringBase::isSameString(const StringBase &other) const {
	return stringForm() == other.stringForm();
}

void StringBase::write(std::ostream &os) const {
	os << '"' << stringForm() << '"';
}

namespace knowrob::py {
	template<>
	void createType<String>() {
		using namespace boost::python;
		class_<StringBase, std::shared_ptr<StringBase>, bases<XSDAtomic>, boost::noncopyable>
				("StringTerm", no_init)
				.def("isSameString", &StringBase::isSameString);
		class_<String, std::shared_ptr<String>, bases<StringBase>>
				("String", init<std::string_view>());
		class_<StringView, std::shared_ptr<StringView>, bases<StringBase>>
				("StringView", init<std::string_view>());
	}
}
