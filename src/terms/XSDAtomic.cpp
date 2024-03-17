/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include "map"
#include "knowrob/terms/XSDAtomic.h"
#include "knowrob/terms/String.h"
#include "knowrob/terms/Numeric.h"
#include "knowrob/semweb/xsd.h"
#include "knowrob/Logger.h"
#include <boost/python.hpp>
#include "knowrob/py/utils.h"

using namespace knowrob;

std::shared_ptr<XSDAtomic> XSDAtomic::create(std::string_view lexicalForm, std::string_view xsdTypeIRI) {
	auto type = xsdTypeFromIRI(xsdTypeIRI);
	switch (type) {
		case XSDType::STRING:
			return std::make_shared<String>(lexicalForm);
		case XSDType::BOOLEAN:
			return std::make_shared<Boolean>(lexicalForm);
		case XSDType::DOUBLE:
			return std::make_shared<Double>(lexicalForm);
		case XSDType::FLOAT:
			return std::make_shared<Float>(lexicalForm);
		case XSDType::NON_NEGATIVE_INTEGER:
		case XSDType::INTEGER:
			return std::make_shared<Integer>(lexicalForm);
		case XSDType::LONG:
			return std::make_shared<Long>(lexicalForm);
		case XSDType::SHORT:
			return std::make_shared<Short>(lexicalForm);
		case XSDType::UNSIGNED_LONG:
			return std::make_shared<UnsignedLong>(lexicalForm);
		case XSDType::UNSIGNED_INT:
			return std::make_shared<UnsignedInt>(lexicalForm);
		case XSDType::UNSIGNED_SHORT:
			return std::make_shared<UnsignedShort>(lexicalForm);
		default:
			KB_ERROR("Unknown XSD type {} treated as string.", static_cast<int>(type));
			return std::make_shared<String>(lexicalForm);
	}
}

namespace knowrob {
	XSDType xsdTypeFromIRI(std::string_view iri) {
		static std::map<std::string_view, XSDType> typeIRIs = {
				{xsd::IRI_string->stringForm(),             XSDType::STRING},
				{xsd::IRI_anyURI->stringForm(),             XSDType::STRING},
				{xsd::IRI_boolean->stringForm(),            XSDType::BOOLEAN},
				{xsd::IRI_double->stringForm(),             XSDType::DOUBLE},
				{xsd::IRI_float->stringForm(),              XSDType::FLOAT},
				{xsd::IRI_integer->stringForm(),            XSDType::INTEGER},
				{xsd::IRI_int->stringForm(),                XSDType::INTEGER},
				{xsd::IRI_nonNegativeInteger->stringForm(), XSDType::NON_NEGATIVE_INTEGER},
				{xsd::IRI_long->stringForm(),               XSDType::LONG},
				{xsd::IRI_short->stringForm(),              XSDType::SHORT},
				{xsd::IRI_unsignedLong->stringForm(),       XSDType::UNSIGNED_LONG},
				{xsd::IRI_unsignedInt->stringForm(),        XSDType::UNSIGNED_INT},
				{xsd::IRI_unsignedShort->stringForm(),      XSDType::UNSIGNED_SHORT}
		};
		auto it = typeIRIs.find(iri);
		if (it != typeIRIs.end()) return it->second;
		KB_WARN("Unknown XSD type IRI {} treated as string.", iri);
		return XSDType::STRING;
	}

	std::string_view xsdTypeToIRI(XSDType type) {
		static std::map<XSDType, std::string_view> typeIRIs = {
				{XSDType::STRING,               xsd::IRI_string->stringForm()},
				{XSDType::BOOLEAN,              xsd::IRI_boolean->stringForm()},
				{XSDType::DOUBLE,               xsd::IRI_double->stringForm()},
				{XSDType::FLOAT,                xsd::IRI_float->stringForm()},
				{XSDType::INTEGER,              xsd::IRI_integer->stringForm()},
				{XSDType::NON_NEGATIVE_INTEGER, xsd::IRI_nonNegativeInteger->stringForm()},
				{XSDType::LONG,                 xsd::IRI_long->stringForm()},
				{XSDType::SHORT,                xsd::IRI_short->stringForm()},
				{XSDType::UNSIGNED_LONG,        xsd::IRI_unsignedLong->stringForm()},
				{XSDType::UNSIGNED_INT,         xsd::IRI_unsignedInt->stringForm()},
				{XSDType::UNSIGNED_SHORT,       xsd::IRI_unsignedShort->stringForm()}
		};
		auto it = typeIRIs.find(type);
		if (it != typeIRIs.end()) return it->second;
		KB_WARN("Unknown XSD type {} treated as string.", static_cast<int>(type));
		return xsd::IRI_string->stringForm();
	}

	std::string_view XSDAtomic::xsdTypeIRI() const {
		return xsdTypeToIRI(xsdType());
	}
}

namespace knowrob::py {
	// this struct is needed because XSDAtomic has pure virtual methods
	struct XSDAtomicWrap : public XSDAtomic, boost::python::wrapper<XSDAtomic> {
		explicit XSDAtomicWrap(PyObject *p) : self(p), XSDAtomic() {}

		XSDType xsdType() const override { return call_method<XSDType>(self, "xsdType"); }

	private:
		PyObject *self;
	};

	template<>
	void createType<XSDAtomic>() {
		using namespace boost::python;
		enum_<XSDType>("XSDType")
				.value("STRING", XSDType::STRING)
				.value("BOOLEAN", XSDType::BOOLEAN)
				.value("DOUBLE", XSDType::DOUBLE)
				.value("FLOAT", XSDType::FLOAT)
				.value("INTEGER", XSDType::INTEGER)
				.value("LONG", XSDType::LONG)
				.value("SHORT", XSDType::SHORT)
				.value("UNSIGNED_LONG", XSDType::UNSIGNED_LONG)
				.value("UNSIGNED_INT", XSDType::UNSIGNED_INT)
				.value("UNSIGNED_SHORT", XSDType::UNSIGNED_SHORT);
		class_<XSDAtomic, std::shared_ptr<XSDAtomicWrap>, bases<Atomic, RDFNode>, boost::noncopyable>
				("XSDAtomic", no_init)
				.def("xsdTypeIRI", &XSDAtomic::xsdTypeIRI)
				.def("xsdType", pure_virtual(&XSDAtomic::xsdType));
	}
}
