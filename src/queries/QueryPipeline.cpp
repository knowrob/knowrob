#include "knowrob/queries/QueryPipeline.h"
#include "iostream"

using namespace knowrob;

QueryPipeline::~QueryPipeline()
{
    for(auto &stage : stages_) {
        stage->close();
    }
    stages_.clear();
}

void QueryPipeline::addStage(const std::shared_ptr<AnswerStream> &stage)
{
    stages_.push_back(stage);
}
