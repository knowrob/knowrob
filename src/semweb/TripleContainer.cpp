/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include "knowrob/semweb/TripleContainer.h"
#include "knowrob/py/utils.h"

namespace knowrob::py {
	template<>
	void createType<semweb::TripleContainer>() {
		using namespace boost::python;
		class_<semweb::TripleContainer, std::shared_ptr<semweb::TripleContainer>, boost::noncopyable>
				("TripleContainer", no_init)
				.def("__iter__", range(&semweb::TripleContainer::begin, &semweb::TripleContainer::end));
	}
}
