//
// Created by daniel on 27.03.23.
//

#include "knowrob/Logger.h"
#include "knowrob/queries/QueryPipeline.h"
#include "knowrob/queries/QueryResultCombiner.h"
#include "knowrob/queries/DependencyGraph.h"
#include "knowrob/queries/QueryError.h"

using namespace knowrob;

namespace knowrob {
    // used to sort nodes in a priority queue.
    // the priority value is used to determine which nodes should be evaluated first.
    struct CompareNodes {
        bool operator()(const DependencyNodePtr &a, const DependencyNodePtr &b) const {
            // note: using ">" in return statements means that smaller element appears before larger.
            if (a->numVariables() != b->numVariables()) {
                // prefer node with less variables
                return a->numVariables() > b->numVariables();
            }
            if(a->numNeighbors() != b->numNeighbors()) {
                // prefer node with less neighbors
                return a->numNeighbors() > b->numNeighbors();
            }
            return a<b;
        }
    };

    // represents a possible step in the query pipeline
    struct QueryPipelineNode {
        explicit QueryPipelineNode(const DependencyNodePtr &node) : node_(node) {
            // add all nodes to a priority queue
            for(auto &neighbor : node->neighbors()) {
                neighbors_.push(neighbor);
            }
        }
        const DependencyNodePtr &node_;
        std::list<std::shared_ptr<QueryPipelineNode>> successors_;
        std::priority_queue<DependencyNodePtr, std::vector<DependencyNodePtr>, CompareNodes> neighbors_;
    };
    using QueryNodePtr = std::shared_ptr<QueryPipelineNode>;
}


QueryPipelineStage::QueryPipelineStage(const std::shared_ptr<QueryResultStream> &outStream,
                                       const std::list<LiteralPtr> &literals,
                                       const ModalityLabelPtr &label)
        : literals_(literals),
          label_(label),
          outChan_(QueryResultStream::Channel::create(outStream)),
          isQueryOpened_(true),
          hasStopRequest_(false)
{
}

QueryPipelineStage::~QueryPipelineStage()
{
    stop();
}

void QueryPipelineStage::stop()
{
    // TODO: how can the graphQueryEngine_ be notified about the stop request?
    hasStopRequest_ = true;
    close();
}

std::shared_ptr<QueryResultBroadcaster> QueryPipelineStage::submitQuery()
{
    // TODO: call some component to start a graph query here.
    //       best to create some interface dedicated just for the graph queries
    //       that can be implemented in different ways.
    // the parameters we have here are: std::list<LiteralPtr>, ModalityLabelPtr
    //return graphQueryEngine_->submitQuery(literals_, label_);
    return {};
}

void QueryPipelineStage::push(const QueryResultPtr &partialResult)
{
    if(QueryResultStream::isEOS(partialResult)) {
        if(isQueryOpened()) {
            isQueryOpened_ = false;
        }
        // TODO eos need to be propagated? no not directly only if immediate stop is requested.
        //      else only send eos once done with all graph queries plus eos received as input.
    }
    else if(!isQueryOpened()) {
        KB_WARN("ignoring attempt to write to a closed stream.");
    }
    else {
        // compute dependencies
        DependencyGraph dg;
        for(auto &literal : literals_) {
            dg.insert(literal);
        }

        // iterate over dependency groups
        for(auto &literalGroup : dg) {
            std::vector<LiteralPtr> literalInstances(literalGroup.member_.size());

            // apply the substitution mapping
            unsigned long nextInstance=0;
            if(partialResult->substitution()->empty()) {
                for(auto &literalNode : literalGroup.member_) {
                    auto &literal = ((LiteralDependencyNode*)literalNode.get())->literal();
                    literalInstances[nextInstance++] = literal;
                }
            }
            else {
                for(auto &literalNode : literalGroup.member_) {
                    auto &literal = ((LiteralDependencyNode*)literalNode.get())->literal();
                    literalInstances[nextInstance++] = literal->applySubstitution(*partialResult->substitution());
                }
            }

            auto queryResultStream = submitQuery();
            // TODO: msgs coming from queryResultStream need to be combined with partialResult before
            //       being published on outChan_!
            //       - could be done via a combiner node without unification that generates all permutations
            //         which is unproblematic in this case
            //queryResultStream->addSubscriber(outChan_);
        }
    }
}


QueryPipeline::QueryPipeline(const std::shared_ptr<QueryResultQueue> &outputQueue)
        : outputQueue_(outputQueue)
{
    inputStream_ = std::make_shared<QueryResultBroadcaster>();
    inputChannel_ = QueryResultStream::Channel::create(inputStream_);
    // TODO: could use a simpler variant that does not use unification!
    outBroadcaster_ = std::make_shared<QueryResultCombiner>();
    // create a channel of outputQueue_, and add it as subscriber such that messages from
    // outBroadcaster_ are added to the queue.
    auto queueChannel = QueryResultStream::Channel::create(outputQueue_);
    outBroadcaster_->addSubscriber(queueChannel);
}

void QueryPipeline::addDependencyGroup(const std::list<DependencyNodePtr> &dependencyGroup)
{
    if(dependencyGroup.empty()) return;

    // Pick a node to start with.
    // For now pick the one with minimum number of neighbors is picked.
    DependencyNodePtr first;
    unsigned long minNumNeighbors = 0;
    for(auto &n : dependencyGroup) {
        if(!first || n->numNeighbors() < minNumNeighbors) {
            minNumNeighbors = n->numNeighbors();
            first = n;
        }
    }

    // remember visited nodes, needed for circular dependencies
    // all nodes added to the queue should also be added to this set.
    std::set<DependencyNodePtr> visited;
    visited.insert(first);

    // start with a FIFO queue only containing first node
    std::deque<QueryNodePtr> queue;
    auto qn0 = queue.emplace_front(std::make_shared<QueryPipelineNode>(first));

    // loop until queue is empty and process exactly one successor of
    // the top element in the FIFO in each step. If an element has no
    // more successors, it can be removed from queue.
    // Each successor creates an additional node added to the top of the FIFO.
    while(!queue.empty()) {
        auto front = queue.front();

        // get top successor node that has not been visited yet
        DependencyNodePtr topNext;
        while(!front->neighbors_.empty()) {
            auto topNeighbor = front->neighbors_.top();
            front->neighbors_.pop();

            if(visited.count(topNeighbor) == 0) {
                topNext = topNeighbor;
                break;
            }
        }
        // pop element from queue if all neighbors were processed
        if(front->neighbors_.empty()) queue.pop_front();

        if(topNext) {
            // push a new node onto FIFO
            auto &qn_next = queue.emplace_front(std::make_shared<QueryPipelineNode>(topNext));
            front->successors_.push_back(qn_next);
            visited.insert(topNext);
        }
    }

    // now the pipeline can be build starting from qn0 following the successor relation.
    generate(qn0, inputStream_, outBroadcaster_);
}

void QueryPipeline::generate(const QueryNodePtr &qn, //NOLINT
                             const std::shared_ptr<QueryResultBroadcaster> &qnInput,
                             const std::shared_ptr<QueryResultBroadcaster> &pipelineOutput)
{
    // create an output stream for this node
    auto qnOutput = (qn->successors_.empty() ? pipelineOutput : std::make_shared<QueryResultBroadcaster>());

    // create a pipeline stage
    std::shared_ptr<QueryPipelineStage> stage;
    auto modalNode = std::dynamic_pointer_cast<ModalDependencyNode>(qn->node_);
    if(modalNode) {
        stage = std::make_shared<QueryPipelineStage>(qnOutput,
                                                     modalNode->literals(),
                                                     modalNode->label());
    }
    else {
        auto literalNode = std::dynamic_pointer_cast<LiteralDependencyNode>(qn->node_);
        if(literalNode) {
            stage = std::make_shared<QueryPipelineStage>(
                    qnOutput, std::list<LiteralPtr>({ literalNode->literal() }));
        }
        else {
            throw QueryError("unexpected dependency node type: not a modal or literal node!");
        }
    }
    stages_.push_back(stage);

    // link input stream to the stage
    std::shared_ptr<QueryResultStream::Channel> qnInChan =
            QueryResultStream::Channel::create(stage);
    qnInput->addSubscriber(qnInChan);

    // continue for successors
    for(auto &successor : qn->successors_) {
        generate(successor, qnOutput, pipelineOutput);
    }
}

void QueryPipeline::run()
{
    // push empty message into in stream, and close the channel
    inputChannel_->push(QueryResultStream::bos());
    inputChannel_->close();
}
