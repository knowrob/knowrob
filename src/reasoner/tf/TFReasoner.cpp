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
// KNOWROB_BUILTIN_REASONER("TF", TFReasoner)

// map of functors to corresponding map
std::map<std::string, computableFunc> COMPUTABLES;

TFReasoner::TFReasoner(std::string reasonerID)
{
	COMPUTABLES["is_at"] = &knowrob::TFReasoner::tfGetPose;
	functors = COMPUTABLES;
}

bool loadConfiguration(const ReasonerConfiguration &cfg)  { return true; }
void setDataBackend(const KnowledgeGraphPtr &knowledgeGraph) {}
unsigned long getCapabilities() { return CAPABILITY_TOP_DOWN_EVALUATION; };

AnswerBufferPtr TFReasoner::submitQuery(const RDFLiteralPtr &literal, int queryFlags) {
	bool sendEOS = true;
	auto answerBuffer = std::make_shared<AnswerBuffer>();
	auto outputChannel = AnswerStream::Channel::create(answerBuffer);

	auto p = std::static_pointer_cast<StringTerm>(literal->propertyTerm()) ;

	computableFunc f = functors[p->value()];
	(*f)(literal, outputChannel);

	if (p->value() == "is_at")  {
		tfGetPose(literal, outputChannel);
	} else if (p->value() == "tf_get_pose") {
		tfGetPose(literal, outputChannel);
	}

	outputChannel->push(AnswerStream::eos());
	return answerBuffer;
}

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