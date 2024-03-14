/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include "knowrob/terms/Numeric.h"
#include "knowrob/py/utils.h"

using namespace knowrob;

bool Numeric::isFloatingNumber() const {
	return xsdType() == XSDType::FLOAT || xsdType() == XSDType::DOUBLE;
}

std::shared_ptr<Numeric> Numeric::trueAtom() {
	static auto trueAtom = std::make_shared<Boolean>(true);
	return trueAtom;
}

std::shared_ptr<Numeric> Numeric::falseAtom() {
	static auto falseAtom = std::make_shared<Boolean>(false);
	return falseAtom;
}

namespace knowrob::py {
	// this struct is needed because Numeric has pure virtual methods
	struct NumericWrap : public Numeric, boost::python::wrapper<Numeric> {
		explicit NumericWrap(PyObject *p) : self(p), Numeric() {}

		double asDouble() const override { return call_method<double>(self, "asDouble"); }

		float asFloat() const override { return call_method<float>(self, "asFloat"); }

		int asInteger() const override { return call_method<int>(self, "asInteger"); }

		long asLong() const override { return call_method<long>(self, "asLong"); }

		short asShort() const override { return call_method<short>(self, "asShort"); }

		unsigned int asUnsignedInteger() const override { return call_method<unsigned int>(self, "asUnsignedInteger"); }

		unsigned long asUnsignedLong() const override { return call_method<unsigned long>(self, "asUnsignedLong"); }

		unsigned short asUnsignedShort() const override { return call_method<unsigned short>(self, "asUnsignedShort"); }

		bool asBoolean() const override { return call_method<bool>(self, "asBoolean"); }

	private:
		PyObject *self;
	};

	template<>
	void createType<Numeric>() {
		using namespace boost::python;
		class_<Numeric, std::shared_ptr<NumericWrap>, bases<XSDAtomic>, boost::noncopyable>
				("Numeric", no_init)
				.def("isFloatingNumber", &Numeric::isFloatingNumber)
				.def("isSameNumeric", &Numeric::isSameNumeric)
				.def("asDouble", pure_virtual(&Numeric::asDouble))
				.def("asFloat", pure_virtual(&Numeric::asFloat))
				.def("asInteger", pure_virtual(&Numeric::asInteger))
				.def("asLong", pure_virtual(&Numeric::asLong))
				.def("asShort", pure_virtual(&Numeric::asShort))
				.def("asUnsignedInteger", pure_virtual(&Numeric::asUnsignedInteger))
				.def("asUnsignedLong", pure_virtual(&Numeric::asUnsignedLong))
				.def("asUnsignedShort", pure_virtual(&Numeric::asUnsignedShort))
				.def("asBoolean", pure_virtual(&Numeric::asBoolean));
		class_<Double, std::shared_ptr<Double>, bases<Numeric>>
				("Double", init<double>())
				.def(init<std::string_view>())
				.def("numericForm", &Double::numericForm);
		class_<Float, std::shared_ptr<Float>, bases<Numeric>>
				("Float", init<float>())
				.def(init<std::string_view>())
				.def("numericForm", &Float::numericForm);
		class_<Integer, std::shared_ptr<Integer>, bases<Numeric>>
				("Integer", init<int>())
				.def(init<std::string_view>())
				.def("numericForm", &Integer::numericForm);
		class_<Long, std::shared_ptr<Long>, bases<Numeric>>
				("Long", init<long>())
				.def(init<std::string_view>())
				.def("numericForm", &Long::numericForm);
		class_<Short, std::shared_ptr<Short>, bases<Numeric>>
				("Short", init<short>())
				.def(init<std::string_view>())
				.def("numericForm", &Short::numericForm);
	}
}
