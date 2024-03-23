/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include "knowrob/reasoner/ReasonerEvent.h"
#include "knowrob/py/utils.h"

using namespace knowrob;
using namespace knowrob::reasoner;

template<typename T>
void createTriples(std::vector<FramedTriplePtr> &triples) {
	for (auto &triple: triples) {
		triple.ptr = new T();
		triple.owned = true;
	}
}

TripleEvent::TripleEvent(Type eventType, uint32_t tripleCount, bool copy)
		: Event(eventType), triples_(tripleCount) {
	if (copy) {
		createTriples<FramedTripleCopy>(triples_);
	} else {
		createTriples<FramedTripleView>(triples_);
	}
}

namespace knowrob::py {
	template<>
	void createType<reasoner::Event>() {
		using namespace boost::python;
		using ReasonerEvent = reasoner::Event;
		using TripleEvent = reasoner::TripleEvent;
		using AssertionEvent = reasoner::AssertionEvent;
		using RetractionEvent = reasoner::RetractionEvent;
		using ReplacementEvent = reasoner::ReplacementEvent;
		using InvalidationEvent = reasoner::InvalidationEvent;
		using ReasonerEventType = reasoner::Event::Type;

		enum_<ReasonerEventType>("ReasonerEventType")
				.value("Assertion", ReasonerEventType::Assertion)
				.value("Retraction", ReasonerEventType::Retraction)
				.value("Replacement", ReasonerEventType::Replacement)
				.value("Invalidation", ReasonerEventType::Invalidation)
				.export_values();

		class_<ReasonerEvent, std::shared_ptr<ReasonerEvent>, boost::noncopyable>
				("ReasonerEvent", no_init)
				.def("eventType", &ReasonerEvent::eventType);

		class_<TripleEvent, bases<ReasonerEvent>, std::shared_ptr<TripleEvent>, boost::noncopyable>
				("TripleEvent", no_init)
				.def("triples", &TripleEvent::triples, return_value_policy<copy_const_reference>())
				.def("triple", &TripleEvent::triple, return_value_policy<reference_existing_object>());

		class_<AssertionEvent, std::shared_ptr<AssertionEvent>, bases<TripleEvent>, boost::noncopyable>
				("AssertionEvent", init<uint32_t, bool>())
				.def(init<uint32_t>());

		class_<RetractionEvent, std::shared_ptr<RetractionEvent>, bases<TripleEvent>, boost::noncopyable>
				("RetractionEvent", init<uint32_t, bool>())
				.def(init<uint32_t>());

		class_<ReplacementEvent, std::shared_ptr<ReplacementEvent>, bases<TripleEvent>, boost::noncopyable>
				("ReplacementEvent", init<uint32_t, bool>())
				.def(init<uint32_t>());

		class_<InvalidationEvent, std::shared_ptr<InvalidationEvent>, bases<ReasonerEvent>, boost::noncopyable>
				("InvalidationEvent", init<>());
	}
}
