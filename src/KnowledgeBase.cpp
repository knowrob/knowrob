/*
 * Copyright (c) 2022, Daniel Beßler
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include <knowrob/Logger.h>
#include <knowrob/KnowledgeBase.h>
#include "knowrob/reasoner/Blackboard.h"
#include "knowrob/graphs/PrefixRegistry.h"

using namespace knowrob;

KnowledgeBase::KnowledgeBase(const boost::property_tree::ptree &config)
{
	reasonerManager_ = std::make_shared<ReasonerManager>();
	loadConfiguration(config);
	// create a PrologReasoner used only for parsing queries
	prologReasoner_ = std::make_shared<knowrob::PrologReasoner>("prolog_user");
	prologReasoner_->loadConfiguration(ReasonerConfiguration());
}

void KnowledgeBase::loadConfiguration(const boost::property_tree::ptree &config)
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

int KnowledgeBase::callPrologDirect(const std::string &queryString)
{
    return prologReasoner_->eval(parseQuery(queryString), "user", false);
}

std::shared_ptr<const Query> KnowledgeBase::parseQuery(const std::string &queryString)
{
	auto term =prologReasoner_->readTerm(queryString);
	return PrologQuery::toQuery(term);
}

void KnowledgeBase::runQuery(const std::shared_ptr<const Query> &query, QueryResultHandler &handler)
{
	auto bbq = std::make_shared<knowrob::AnswerQueue>();
	auto bb = std::make_shared<Blackboard>(reasonerManager_.get(), bbq, query);
	AnswerPtr solution;

	bb->start();
	do {
		solution = bbq->pop_front();
		if(AnswerStream::isEOS(solution)) {
			break;
		}
	} while(handler.pushQueryResult(solution));
}

bool KnowledgeBase::projectIntoEDB(const std::list<Statement> &statements, const std::string &reasonerID)
{
    bool allProjected = true;
    for(auto &x : statements) {
        allProjected = projectIntoEDB(x, reasonerID) && allProjected;
    }
    return allProjected;
}

bool KnowledgeBase::projectIntoEDB(const Statement &statement, const std::string &reasonerID)
{
    if(reasonerID == "*") {
        bool oneSucceeded = false;
        bool oneAttempted = false;
        for(auto &pair : reasonerManager_->reasonerPool()) {
            if(pair.second->reasoner()->hasCapability(CAPABILITY_DYNAMIC_ASSERTIONS)) {
                auto description_n = pair.second->reasoner()->getPredicateDescription(statement.predicate()->indicator());
                if(description_n) {
                    oneSucceeded = pair.second->reasoner()->projectIntoEDB(statement) || oneSucceeded;
                    oneAttempted = true;
                }
            }
        }
        if(!oneSucceeded && !oneAttempted) {
            KB_ERROR("None of the known reasoners is able to project the predicate {}.", *statement.predicate());
        }
        return oneSucceeded;
    }
    else {
        // project into user-specified reasoner backend
        auto definedReasoner = reasonerManager_->getReasonerWithID(reasonerID);
        if(definedReasoner) {
            if(definedReasoner->reasoner()->hasCapability(CAPABILITY_DYNAMIC_ASSERTIONS)) {
                return definedReasoner->reasoner()->projectIntoEDB(statement);
            }
            else {
                KB_ERROR("Reasoner with id '{}' has not the capability to dynamically assert factual knowledge.", reasonerID);
            }
        } else {
            KB_ERROR("No reasoner with id '{}' is known.", reasonerID);
        }
        return false;
    }
}
