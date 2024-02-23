#include "knowrob/semweb/FramedTriple.h"

using namespace knowrob;

template<typename T> T xsdConv(std::string_view str) {
	T result;
	std::istringstream(str.data()) >> result;
	return result;
}

template<typename T> T xsdConvFixed(std::string_view str) {
	T result;
	std::istringstream(str.data()) >> std::fixed >> result;
	return result;
}

bool xsdConvBool(std::string_view str) {
    bool result;
    std::istringstream(str.data()) >> std::boolalpha >> result;
    return result;
}

void FramedTriple::setXSDValue(std::string_view v, XSDType type) {
	switch (type) {
		case XSDType::STRING:
			setStringValue(v);
			break;
		case XSDType::DOUBLE:
			setDoubleValue(xsdConvFixed<double>(v));
			break;
		case XSDType::FLOAT:
			setDoubleValue(xsdConvFixed<float>(v));
			break;
		case XSDType::NON_NEGATIVE_INTEGER:
		case XSDType::INTEGER:
			setIntValue(xsdConv<int>(v));
			break;
		case XSDType::LONG:
			setLongValue(xsdConv<long>(v));
			break;
		case XSDType::SHORT:
			setShortValue(xsdConv<short>(v));
			break;
		case XSDType::UNSIGNED_LONG:
			setUnsignedLongValue(xsdConv<unsigned long>(v));
			break;
		case XSDType::UNSIGNED_INT:
			setUnsignedIntValue(xsdConv<unsigned int>(v));
			break;
		case XSDType::UNSIGNED_SHORT:
			setUnsignedShortValue(xsdConv<unsigned short>(v));
			break;
		case XSDType::BOOLEAN:
			setBooleanValue(xsdConvBool(v));
			break;
		case XSDType::LAST:
			KB_ERROR("Invalid XSD type");
			break;
	}
}

static inline bool c_str_equal(const char *a, const char *b) {
	if (!a) {
		return (!b);
	} else if (!b) {
		return false;
	} else {
		return std::string_view(a) == std::string_view(b);
	}
}

bool FramedTriple::operator<(const FramedTriple &other) const {
	if (graph() != other.graph()) {
		return graph() < other.graph();
	}
	if (agent() != other.agent()) {
		return agent() < other.agent();
	}
	if (subject() != other.subject()) {
		return subject() < other.subject();
	}
	if (predicate() != other.predicate()) {
		return predicate() < other.predicate();
	}
	if (xsdType() != other.xsdType()) {
		return xsdType() < other.xsdType();
	}
	if (xsdType()) {
		switch(xsdType().value()) {
			case XSDType::STRING:
				if (valueAsString() != other.valueAsString()) {
					return valueAsString() < other.valueAsString();
				} else {
					break;
				}
			case XSDType::DOUBLE:
				if (valueAsDouble() != other.valueAsDouble()) {
					return valueAsDouble() < other.valueAsDouble();
				} else {
					break;
				}
			case XSDType::FLOAT:
				if (valueAsFloat() != other.valueAsFloat()) {
					return valueAsFloat() < other.valueAsFloat();
				} else {
					break;
				}
			case XSDType::NON_NEGATIVE_INTEGER:
			case XSDType::INTEGER:
				if (valueAsInt() != other.valueAsInt()) {
					return valueAsInt() < other.valueAsInt();
				} else {
					break;
				}
			case XSDType::BOOLEAN:
				if (valueAsBoolean() != other.valueAsBoolean()) {
					return valueAsBoolean() < other.valueAsBoolean();
				} else {
					break;
				}
			case XSDType::LONG:
				if (valueAsLong() != other.valueAsLong()) {
					return valueAsLong() < other.valueAsLong();
				} else {
					break;
				}
			case XSDType::SHORT:
				if (valueAsShort() != other.valueAsShort()) {
					return valueAsShort() < other.valueAsShort();
				} else {
					break;
				}
			case XSDType::UNSIGNED_LONG:
				if (valueAsUnsignedLong() != other.valueAsUnsignedLong()) {
					return valueAsUnsignedLong() < other.valueAsUnsignedLong();
				} else {
					break;
				}
			case XSDType::UNSIGNED_INT:
				if (valueAsUnsignedInt() != other.valueAsUnsignedInt()) {
					return valueAsUnsignedInt() < other.valueAsUnsignedInt();
				} else {
					break;
				}
			case XSDType::UNSIGNED_SHORT:
				if (valueAsUnsignedShort() != other.valueAsUnsignedShort()) {
					return valueAsUnsignedShort() < other.valueAsUnsignedShort();
				} else {
					break;
				}
			case XSDType::LAST:
				KB_ERROR("Invalid XSD type");
				break;
		}
	} else if(valueAsString() != other.valueAsString()) {
		return valueAsString() < other.valueAsString();
	}
	if (temporalOperator() != other.temporalOperator()) {
		return temporalOperator() < other.temporalOperator();
	}
	if (epistemicOperator() != other.epistemicOperator()) {
		return epistemicOperator() < other.epistemicOperator();
	}
	if (begin() != other.begin()) {
		return begin() < other.begin();
	}
	if (end() != other.end()) {
		return end() < other.end();
	}
	if (confidence() != other.confidence()) {
		return confidence() < other.confidence();
	}
	return false;
}

bool FramedTriple::operator==(const FramedTriple &other) const {
	if (subject() != other.subject()) return false;
	if (predicate() != other.predicate()) return false;
	if (graph(), other.graph()) return false;
	if (agent(), other.agent()) return false;
	if (temporalOperator() != other.temporalOperator()) return false;
	if (epistemicOperator() != other.epistemicOperator()) return false;
	if (begin() != other.begin()) return false;
	if (end() != other.end()) return false;
	if (confidence() != other.confidence()) return false;
	if (xsdType() != other.xsdType()) return false;
	if (xsdType()) {
		switch(xsdType().value()) {
			case XSDType::STRING:
				if (valueAsString() != other.valueAsString()) return false;
				break;
			case XSDType::DOUBLE:
				if (valueAsDouble() != other.valueAsDouble()) return false;
				break;
			case XSDType::FLOAT:
				if (valueAsFloat() != other.valueAsFloat()) return false;
				break;
			case XSDType::BOOLEAN:
				if (valueAsBoolean() != other.valueAsBoolean()) return false;
				break;
			case XSDType::NON_NEGATIVE_INTEGER:
			case XSDType::INTEGER:
				if (valueAsInt() != other.valueAsInt()) return false;
				break;
			case XSDType::LONG:
				if (valueAsLong() != other.valueAsLong()) return false;
				break;
			case XSDType::SHORT:
				if (valueAsShort() != other.valueAsShort()) return false;
				break;
			case XSDType::UNSIGNED_LONG:
				if (valueAsUnsignedLong() != other.valueAsUnsignedLong()) return false;
				break;
			case XSDType::UNSIGNED_INT:
				if (valueAsUnsignedInt() != other.valueAsUnsignedInt()) return false;
				break;
			case XSDType::UNSIGNED_SHORT:
				if (valueAsUnsignedShort() != other.valueAsUnsignedShort()) return false;
				break;
			case XSDType::LAST:
				KB_ERROR("Invalid XSD type");
				break;
		}
	} else if (valueAsString() != other.valueAsString()) {
		return false;
	}
	return true;
}
