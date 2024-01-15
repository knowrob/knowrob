/*
 * Copyright (c) 2023, Sascha Jongebloed
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include "knowrob/reasoner/tf/TFReasoner.h"
#include "knowrob/reasoner/ReasonerManager.h"
#include "knowrob/reasoner/tf/memory.h"
#include "knowrob/reasoner/tf/logger.h"
#include "knowrob/reasoner/tf/publisher.h"
#include "knowrob/reasoner/tf/republisher.h"

using namespace knowrob;

static ros::NodeHandle node;
static TFMemory memory;
static TFPublisher pub(memory);
static TFLogger *tf_logger=NULL;

// TF logger parameter
double vectorial_threshold=0.001;
double angular_threshold=0.001;
double time_threshold=-1.0;
std::string logger_db_name="roslog";

// make reasoner type accessible
REASONER_PLUGIN("","")

// map of functors to corresponding map
std::map<std::string, computableFunc> COMPUTABLES = {
		{"is_at",&knowrob::TFReasoner::tfGetPose},
		{"tf_get_pose",&knowrob::TFReasoner::tfGetPose}
};

TFReasoner::TFReasoner(std::string reasonerID)
{
	functors = COMPUTABLES;
}

void TFReasoner::setDataBackend(const KnowledgeGraphPtr &knowledgeGraph) {}
unsigned long TFReasoner::getCapabilities() const {return CAPABILITY_TOP_DOWN_EVALUATION;}

// Load the settings specified for this reasoner
// TODO: Maybe have a format to express for reasoners what
// the avaiable fields are?  E.g. README.md
bool TFReasoner::loadConfiguration(const ReasonerConfiguration &cfg)  {

	// load properties into the reasoner module.
	// this is needed mainly for the `reasoner_setting/2` that provides reasoner instance specific settings.
	for(auto &pair : cfg.settings) {
		std::stringstream key;
		key << pair.first;
		if(key.str() == "log_tf", pair.second->type() == TermType::INT32) {
			auto intTerm = (Integer32Term*) pair.second.get();
			if (intTerm->value()) {
				tf_logger = new TFLogger(node,memory);
				tf_logger->set_db_name(logger_db_name);
				tf_logger->set_time_threshold(time_threshold);
				tf_logger->set_vectorial_threshold(vectorial_threshold);
				tf_logger->set_angular_threshold(angular_threshold);
			}
		}
	}

	return true;
}



// Functions for the computables of this reasoner

void TFReasoner::tfGetPose(const std::shared_ptr<knowrob::RDFLiteral> &literal,
						   const std::shared_ptr<AnswerStream::Channel> &outputChannel) {
	auto p = std::static_pointer_cast<StringTerm>(literal->propertyTerm()) ;
	std::string frame(p->value().c_str());
	double stamp;
	PlTerm pose_term;
	auto answer = std::make_shared<Answer>();
	if(memory.get_pose_term(frame,&pose_term,&stamp)) {
		auto v = *literal->objectTerm()->getVariables().begin();
		answer->substitution()->set(*v, std::make_shared<StringTerm>((char*) pose_term));
	}
}

void TFReasoner::tfMemClear(const std::shared_ptr<knowrob::RDFLiteral> &literal,
						   const std::shared_ptr<AnswerStream::Channel> &outputChannel) {
	memory.clear();
}