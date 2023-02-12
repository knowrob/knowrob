/*
 * Copyright (c) 2022, Daniel Beßler
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include <knowrob/Logger.h>
#include <knowrob/HybridQA.h>
#include <knowrob/Blackboard.h>
#include "knowrob/rdf/PrefixRegistry.h"

using namespace knowrob;

HybridQA::HybridQA(const boost::property_tree::ptree &config)
{
	reasonerManager_ = std::make_shared<ReasonerManager>();
	loadConfiguration(config);
	// create a PrologReasoner used only for parsing queries
	prologReasoner_ = std::make_shared<knowrob::PrologReasoner>("prolog_user");
	prologReasoner_->loadConfiguration(ReasonerConfiguration());
}

void HybridQA::loadConfiguration(const boost::property_tree::ptree &config)
{
    auto semwebTree = config.get_child_optional("semantic-web");
    if(semwebTree) {
        // load RDF URI aliases
        auto prefixesList = semwebTree.value().get_child_optional("prefixes");
        for(const auto &pair : prefixesList.value()) {
            auto alias = pair.second.get("alias","");
            auto uri = pair.second.get("uri","");
            if(!alias.empty() && !uri.empty()) {
                rdf::PrefixRegistry::get().registerPrefix(alias, uri);
            }
            else {
                KB_WARN("Invalid entry in semantic-web::prefixes, 'alias' and 'uri' must be defined.");
            }
        }
    }

	auto reasonerList = config.get_child_optional("reasoner");
	if(reasonerList) {
		for(const auto &pair : reasonerList.value()) {
			try {
				reasonerManager_->loadReasoner(pair.second);
			}
			catch(std::exception& e) {
				KB_ERROR("failed to load a reasoner: {}", e.what());
			}
		}
	}
	else {
		KB_ERROR("configuration has no 'reasoner' key.");
	}
}

std::shared_ptr<const Query> HybridQA::parseQuery(const std::string &queryString)
{
	auto term =prologReasoner_->readTerm(queryString);
	return PrologQuery::toQuery(term);
}

void HybridQA::runQuery(const std::shared_ptr<const Query> &query, QueryResultHandler &handler) {
	auto bbq = std::make_shared<knowrob::QueryResultQueue>();
	auto bb = std::make_shared<Blackboard>(reasonerManager_, bbq, query);
	QueryResultPtr solution;

	bb->start();
	do {
		solution = bbq->pop_front();
		if(QueryResultStream::isEOS(solution)) {
			break;
		}
	} while(handler.pushQueryResult(solution));
}
