/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include "knowrob/semweb/Agent.h"
#include "knowrob/py/utils.h"

#define AGENT_EGO_IDENTIFIER "self"

using namespace knowrob;

std::map<std::string_view, std::shared_ptr<Agent>> Agent::agentMap_ = std::map<std::string_view, std::shared_ptr<Agent>>();

Agent::Agent(const std::string_view &iri)
		: semweb::Resource(iri) {}

std::shared_ptr<Agent> Agent::getEgo() {
	static std::shared_ptr<Agent> ego;
	if (ego == nullptr) {
		ego = std::make_shared<Agent>(AGENT_EGO_IDENTIFIER);
		agentMap_[AGENT_EGO_IDENTIFIER] = ego;
		// TODO: also handle a proper agent IRI here. but currently there is no mechanism to tell the KB
		//       who the agent is that runs it. Seems like a good idea for a global settings parameter.
		//       but would restrict to single-agent use of a B instance.
	}
	return ego;
}

std::shared_ptr<Agent> Agent::get(const std::string_view &iri) {
	auto it = agentMap_.find(iri);
	if (it == agentMap_.end()) {
		auto agent = std::make_shared<Agent>(iri);
		agentMap_[iri] = agent;
		return agent;
	} else {
		return it->second;
	}
}

namespace knowrob::py {
	template<>
	void createType<Agent>() {
		using namespace boost::python;
		class_<Agent, std::shared_ptr<Agent>>("Agent", no_init)
				.def("iri", &Agent::iri)
				.def("getEgo", &Agent::getEgo)
				.def("get", &Agent::get);
	}
}
