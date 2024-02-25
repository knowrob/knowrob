/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include "knowrob/queries/QueryContext.h"
#include "knowrob/py/utils.h"

using namespace knowrob;

namespace knowrob::py {
	template<>
	void createType<QueryContext>() {
		using namespace boost::python;
		class_<QueryContext, std::shared_ptr<QueryContext>>
				("QueryContext", init<int>())
				.def(init<const QueryContext &, const ModalOperatorPtr &>())
				.def_readwrite("queryFlags", &QueryContext::queryFlags)
				.def_readwrite("modalIteration", &QueryContext::modalIteration)
				.def_readwrite("selector", &QueryContext::selector);
	}
}
