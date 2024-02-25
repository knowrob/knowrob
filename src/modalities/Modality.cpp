/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include "knowrob/modalities/Modality.h"
#include "knowrob/py/utils.h"
#include "knowrob/modalities/EpistemicModality.h"
#include "knowrob/modalities/TemporalModality.h"

using namespace knowrob;

namespace knowrob::py {
	template<>
	void createType<Modality>() {
		using namespace boost::python;
		enum_<ModalityType>("ModalityType")
				.value("Epistemic", ModalityType::Epistemic)
				.value("Temporal_Past", ModalityType::Temporal_Past);
		enum_<TemporalOperator>("TemporalOperator")
				.value("ALWAYS", TemporalOperator::ALWAYS)
				.value("SOMETIMES", TemporalOperator::SOMETIMES);
		enum_<EpistemicOperator>("EpistemicOperator")
				.value("KNOWLEDGE", EpistemicOperator::KNOWLEDGE)
				.value("BELIEF", EpistemicOperator::BELIEF);
		/*
		class_<Modality, std::shared_ptr<Modality>, boost::noncopyable>("Modality", no_init)
			.def("modalityType", python::pure_virtual(&Modality::modalityType))
			.def("parameters", &Modality::parameters, CONST_REF_RETURN)
			;
		class_<EpistemicModality, std::shared_ptr<EpistemicModality>, bases<Modality>>
			("EpistemicModality", init<>())
			.def(init<const std::string_view&>())
			.def("agent", &EpistemicModality::agent, CONST_REF_RETURN)
			.def("necessity_symbol", &EpistemicModality::necessity_symbol)
			.def("possibility_symbol", &EpistemicModality::possibility_symbol);
	 	*/
	}
}
