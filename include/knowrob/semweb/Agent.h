//
// Created by daniel on 23.01.24.
//

#ifndef KNOWROB_AGENT_H
#define KNOWROB_AGENT_H

#include "memory"
#include "map"
#include "Resource.h"

namespace knowrob {

	class Agent : public semweb::Resource {
	public:
		explicit Agent(const std::string_view &iri);

		static std::shared_ptr<Agent> getEgo();

		static std::shared_ptr<Agent> get(const std::string_view &iri);

	protected:
		static std::map<std::string_view, std::shared_ptr<Agent>> agentMap_;
	};

    using AgentPtr = std::shared_ptr<Agent>;

} // knowrob

#endif //KNOWROB_AGENT_H
