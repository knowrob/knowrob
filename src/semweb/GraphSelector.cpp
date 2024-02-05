//
// Created by daniel on 31.01.24.
//

#include "iomanip"
#include "knowrob/semweb/GraphSelector.h"
#include "knowrob/knowrob.h"

using namespace knowrob;

bool GraphSelector::mergeWith(const GraphSelector &other) {
	// TODO: not sure how this field should be handled
	static const char *mergedGraph = "merged";
	graph = mergedGraph;

	// agent cannot be changed in merge operation.
	// So GraphSelector can only be merged within a modal in which both are embedded.
	if (agent.has_value()) {
		if (other.agent.has_value()) {
			if (agent.value()->iri() != other.agent.value()->iri()) {
				return false;
			}
		} else {
			return false;
		}
	} else if (other.agent.has_value()) {
		return false;
	}

	// "belief" is more restrictive than "knowledge"
	if (other.epistemicOperator.has_value() && (!epistemicOperator.has_value() ||
												other.epistemicOperator.value() == EpistemicOperator::BELIEF)) {
		epistemicOperator = other.epistemicOperator;
	}

	// "sometimes" is more restrictive than "always"
	if (other.temporalOperator.has_value() && (!temporalOperator.has_value() ||
											   other.temporalOperator.value() == TemporalOperator::SOMETIMES)) {
		temporalOperator = other.temporalOperator;
	}

	// later begin time is more restrictive
	if (other.begin.has_value() && (!begin.has_value() || other.begin.value() > begin.value())) {
		begin = other.begin;
	}

	// earlier end time is more restrictive
	if (other.end.has_value() && (!end.has_value() || other.end.value() > end.value())) {
		end = other.end;
	}

	// smaller confidence is more restrictive
	// FIXME: using the min confidence is not correct, but it is a good approximation for the moment...
	if (other.confidence.has_value() && (!confidence.has_value() || other.confidence.value() < confidence.value())) {
		confidence = other.confidence;
	}

	return true;
}

size_t GraphSelector::hash() const {
	size_t val = 0;
	if (graph) {
		hashCombine(val, std::hash<std::string>{}(graph));
	} else {
		hashCombine(val, 0);
	}
	if (agent.has_value()) {
		hashCombine(val, std::hash<std::string>{}(agent.value()->iri()));
	} else {
		hashCombine(val, 0);
	}
	if (temporalOperator.has_value()) {
		hashCombine(val, uint32_t(temporalOperator.value()));
	} else {
		hashCombine(val, 0);
	}
	if (epistemicOperator.has_value()) {
		hashCombine(val, uint32_t(epistemicOperator.value()));
	} else {
		hashCombine(val, 0);
	}
	hashCombine(val, std::hash<std::optional<double>>{}(end));
	hashCombine(val, std::hash<std::optional<double>>{}(begin));
	hashCombine(val, std::hash<std::optional<double>>{}(confidence));
	return val;
}

std::ostream &GraphSelector::write(std::ostream &os) const {
	os << std::setprecision(1);

	bool hasEpistemicOperator = false;
	if(confidence.has_value()) {
		hasEpistemicOperator = true;
		if(confidence.value() > 0.999) {
			os << 'K';
		}
		else if(epistemicOperator.value() == EpistemicOperator::KNOWLEDGE) {
			os << "B[" << confidence.value() << "]";
		}
	}
	else if(epistemicOperator.has_value()) {
		hasEpistemicOperator = true;
		if(epistemicOperator.value() == EpistemicOperator::BELIEF) {
			os << 'B';
		}
		else if(epistemicOperator.value() == EpistemicOperator::KNOWLEDGE) {
			os << 'K';
		}
	}
	if(agent.has_value() && !agent.value()->isEgoAgent()) {
		if(!hasEpistemicOperator) {
			hasEpistemicOperator = true;
			os << 'K';
		}
		os << '[' << agent.value()->iri() << ']';
	}

	bool hasTemporalOperator = false;
	if(temporalOperator.has_value()) {
		hasTemporalOperator = true;
		if(temporalOperator.value() == TemporalOperator::SOMETIMES) {
			os << 'P';
		}
		else if(temporalOperator.value() == TemporalOperator::ALWAYS) {
			os << 'H';
		}
	}
	if(begin.has_value() || end.has_value()) {
		if(!hasTemporalOperator) {
			hasTemporalOperator = true;
			os << 'H';
		}
		os << '[';
		if(begin.has_value()) {
			os << begin.value();
		}
		os << '-';
		if(end.has_value()) {
			os << end.value();
		}
		os << ']';
	}

	return os;
}

std::ostream &std::operator<<(std::ostream &os, const knowrob::GraphSelector &gs) {
	return gs.write(os);
}
