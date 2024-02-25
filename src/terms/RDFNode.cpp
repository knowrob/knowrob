/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include <boost/algorithm/string/predicate.hpp>
#include "knowrob/terms/RDFNode.h"
#include "knowrob/py/utils.h"

namespace knowrob {
	RDFNodeType rdfNodeTypeGuess(std::string_view str) {
		if (str.empty()) return RDFNodeType::LITERAL;
		if (str[0] == '_') return RDFNodeType::BLANK;
		if (str[0] == '<') return RDFNodeType::IRI;
		if (boost::algorithm::starts_with(str, "http://") ||
			boost::algorithm::starts_with(str, "https://"))
			return RDFNodeType::IRI;
		if (boost::algorithm::starts_with(str, "genid")) return RDFNodeType::BLANK;
		return RDFNodeType::LITERAL;
	}
}

namespace knowrob::py {
	// this struct is needed because RDFNode has pure virtual methods
	struct RDFNodeWrap : public RDFNode, boost::python::wrapper<RDFNode> {
		explicit RDFNodeWrap(PyObject *p) : self(p), RDFNode() {}

		RDFNodeType rdfNodeType() const override { return call_method<RDFNodeType>(self, "rdfNodeType"); }

	private:
		PyObject *self;
	};

	template<>
	void createType<RDFNode>() {
		using namespace boost::python;
		enum_<RDFNodeType>("RDFNodeType")
				.value("BLANK", RDFNodeType::BLANK)
				.value("IRI", RDFNodeType::IRI)
				.value("LITERAL", RDFNodeType::LITERAL);
		class_<RDFNode, std::shared_ptr<RDFNodeWrap>, boost::noncopyable>("RDFNode", no_init)
				.def("rdfNodeType", pure_virtual(&RDFNode::rdfNodeType));
	}
}
