/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include "knowrob/terms/Atomic.h"
#include "knowrob/terms/Atom.h"
#include "knowrob/terms/Numeric.h"
#include "knowrob/terms/String.h"
#include "knowrob/terms/IRIAtom.h"
#include "knowrob/terms/Blank.h"

using namespace knowrob;

bool Atomic::isSameAtomic(const Atomic &other) const {
	if (atomicType() != other.atomicType()) {
		return false;
	}
	switch (atomicType()) {
		case AtomicType::ATOM:
			return ((Atom *) this)->isSameAtom(*((Atom *) &other));
		case AtomicType::NUMERIC:
			return ((Numeric *) this)->isSameNumeric(*((Numeric *) &other));
		case AtomicType::STRING:
			return ((String *) this)->isSameString(*((String *) &other));
	}
	return false;
}

std::shared_ptr<Atomic> Atomic::makeTripleValue(const FramedTriple &triple) {
	if (triple.xsdType()) {
		switch (triple.xsdType().value()) {
			case XSDType::STRING:
				return std::make_shared<String>(triple.valueAsString());
			case XSDType::BOOLEAN:
				return std::make_shared<Boolean>(triple.valueAsBoolean());
			case XSDType::DOUBLE:
				return std::make_shared<Double>(triple.valueAsDouble());
			case XSDType::FLOAT:
				return std::make_shared<Float>(triple.valueAsFloat());
			case XSDType::NON_NEGATIVE_INTEGER:
			case XSDType::INTEGER:
				return std::make_shared<Integer>(triple.valueAsInt());
			case XSDType::LONG:
				return std::make_shared<Long>(triple.valueAsLong());
			case XSDType::SHORT:
				return std::make_shared<Short>(triple.valueAsShort());
			case XSDType::UNSIGNED_LONG:
				return std::make_shared<UnsignedLong>(triple.valueAsUnsignedLong());
			case XSDType::UNSIGNED_INT:
				return std::make_shared<UnsignedInt>(triple.valueAsUnsignedInt());
			case XSDType::UNSIGNED_SHORT:
				return std::make_shared<UnsignedShort>(triple.valueAsUnsignedShort());
			case XSDType::LAST:
				break;
		}
	}
	return IRIAtom::Tabled(triple.valueAsString());
}

size_t Atomic::hash() const {
	return std::hash<std::string_view>{}(stringForm());
}
