#include "knowrob/semweb/StatementData.h"

using namespace knowrob;

static inline bool c_str_equal(const char* a, const char* b) {
	if (!a) {
		return (!b);
	} else if (!b) {
		return false;
	} else {
		return std::string_view(a) == std::string_view(b);
	}
}

bool StatementData::operator==(const StatementData &other) const {
	if (!c_str_equal(subject,other.subject)) return false;
	if (!c_str_equal(predicate,other.predicate)) return false;
	if (!c_str_equal(graph,other.graph)) return false;
	if (!c_str_equal(agent,other.agent)) return false;
	if (temporalOperator != other.temporalOperator) return false;
	if (epistemicOperator != other.epistemicOperator) return false;
	if (begin != other.begin) return false;
	if (end != other.end) return false;
	if (confidence != other.confidence) return false;

	if (objectType != other.objectType) return false;
	if (objectType == RDF_DOUBLE_LITERAL) {
		if (objectDouble != other.objectDouble) return false;
	} else if (objectType == RDF_INT64_LITERAL) {
		if (objectInteger != other.objectInteger) return false;
	} else {
		if (!c_str_equal(object,other.object)) return false;
	}
	return true;
}
