//
// Created by daniel on 27.03.23.
//

#include "knowrob/Logger.h"
#include "knowrob/queries/QueryPipeline.h"
#include "knowrob/queries/QueryResultCombiner.h"
#include "knowrob/queries/DependencyGraph.h"

using namespace knowrob;

using Node = ModalDependencyNode;

QueryPipeline::QueryPipeline(const KnowledgeBasePtr &kb,
                             const std::shared_ptr<QueryResultQueue> &outputQueue)
        : kb_(kb),
          outputQueue_(outputQueue)
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

void QueryPipeline::run()
{
    // push empty message into in stream, and close the channel
    inputChannel_->push(QueryResultStream::bos());
    inputChannel_->close();
}

void QueryPipeline::addDependencyGroup(const std::list<Node*> &dependencyGroup)
{
    if(dependencyGroup.empty()) return;

    // Pick a node to start with.
    // For now pick the one with minimum number of neighbors is picked.
    const Node *first = nullptr;
    unsigned long minNumNeighbors = 0;
    for(auto *n : dependencyGroup) {
        if(!first || n->numNeighbors() < minNumNeighbors) {
            minNumNeighbors = n->numNeighbors();
            first = n;
        }
    }

    // remember visited nodes, needed for circular dependencies
    // all nodes added to the queue should also be added to this set.
    std::set<const Node*> visited;
    visited.insert(first);

    // start with a FIFO queue only containing first node
    std::deque<QueryNodePtr> queue;
    auto qn0 = queue.emplace_front(std::make_shared<QueryNode>(first));

    // loop until queue is empty and process exactly one successor of
    // the top element in the FIFO in each step. If an element has no
    // more successors, it can be removed from queue.
    // Each successor creates an additional node added to the top of the FIFO.
    while(!queue.empty()) {
        auto front = queue.front();

        // get top successor node that has not been visited yet
        Node *topNext = nullptr;
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
            auto &qn_next = queue.emplace_front(std::make_shared<QueryNode>(topNext));
            front->successors_.push_back(qn_next);
            visited.insert(topNext);
        }
    }

    // now the pipeline can be build starting from qn0 following the successor relation.
    createPipeline(qn0, inputStream_, outBroadcaster_);
}

void QueryPipeline::createPipeline(QueryNodePtr &qn, //NOLINT
                                   const std::shared_ptr<QueryResultBroadcaster> &qnInput,
                                   const std::shared_ptr<QueryResultBroadcaster> &pipelineOutput)
{
    // create an output stream for this node
    auto qnOutput = (qn->successors_.empty() ? pipelineOutput : std::make_shared<QueryResultBroadcaster>());

    std::shared_ptr<QueryPipeline::Stream> qnInStream;
    std::shared_ptr<QueryResultStream::Channel> qnInChan, qnOutChan;
    // create IO channels
    qnOutChan = QueryResultStream::Channel::create(qnOutput);
    qnInStream = std::make_shared<QueryPipeline::Stream>(kb_,qn,qnOutChan);
    qnInChan = QueryResultStream::Channel::create(qnInStream);
    qnInput->addSubscriber(qnInChan);
    // create a new segment
    queryStreams_.push_back(qnInStream);

    // continue for successors
    for(auto &successor : qn->successors_) {
        createPipeline(successor, qnOutput, pipelineOutput);
    }
}


bool QueryPipeline::CompareNodes::operator()(Node *a, Node *b) const
{
    // note: using ">" in return statements means that smaller element appears before larger.
    if (a->numVariables() != b->numVariables()) {
        // prefer node with less variables
        return a->numVariables() > b->numVariables();
    }
    if(a->numNeighbors() != b->numNeighbors()) {
        // prefer node with less neighbors
        return a->numNeighbors() > b->numNeighbors();
    }
    // some additional metrics:
    /*
    if (a->numLiteralNodes() != b->numLiteralNodes()) {
        // prefer node with larger number of literals
        return a->numLiteralNodes() < b->numLiteralNodes();
    }
    if (a->numLiteralGroups() != b->numLiteralGroups()) {
        return a->numLiteralGroups() < b->numLiteralGroups();
    }
    */
    return a<b;
}

QueryPipeline::QueryNode::QueryNode(const Node *node)
        : node_(node)
{
    for(auto &neighbor : node->neighbors()) {
        neighbors_.push(neighbor);
    }
}


QueryPipeline::Stream::Stream(const KnowledgeBasePtr &kb,
                              const QueryNodePtr &qn,
                              const std::shared_ptr<QueryResultStream::Channel> &outChan)
        : kb_(kb),
          qn_(qn),
          outChan_(outChan),
          isQueryOpened_(true),
          hasStopRequest_(false)
{
}

QueryPipeline::Stream::~Stream()
{
    stop();
}

void QueryPipeline::Stream::stop()
{
    // TODO: notify ongoing graph queries to stop?
    hasStopRequest_ = true;
    close();
}

void QueryPipeline::Stream::push(const QueryResultPtr &msg)
{
    if(QueryResultStream::isEOS(msg)) {
        if(isQueryOpened()) {
            isQueryOpened_ = false;
        }
    }
    else if(!isQueryOpened()) {
        KB_WARN("ignoring attempt to write to a closed stream.");
    }
    else {
        // TODO: literal groups should be computed here. Because we have a substitution here that
        //        might change dependency relations!
        //        so better generate a dependency for literals here and above rather one for modal formulas only.
        for(auto &literalGroup : qn_->node_->literalGroups()) {
            std::vector<std::shared_ptr<Literal>> literalInstances(literalGroup.member_.size());

            // apply the substitution mapping
            unsigned long nextInstance=0;
            if(msg->substitution()->empty()) {
                for(auto &literalNode : literalGroup.member_) {
                    literalInstances[nextInstance++] = literalNode->literal();
                }
            }
            else {
                for(auto &literalNode : literalGroup.member_) {
                    literalInstances[nextInstance++] = literalNode->literal()->applySubstitution(*msg->substitution());
                }
            }

/*
            // FIXME: so currently the idea would be that
            // results to graph queries should be published using outChan_.
            // the QueryResult should contain all substitutions so far, so cannot be generate given
            // the literals.
            // so effectively a hook is needed that applies the new instantiations to the substitution.
            auto graphQueryOutput = kb_->submitQuery(literalInstances, qn_->node_->label());
            // TODO: it would be nice if kb_ could generate just a partial instantiation. I would like to write:
            //outChan_ << graphQueryOutput;
            // meaning to feed all messages of RHS stream into LHS channel
            // TODO can't we just add a subscriber? nope does not combine with msg
            // TODO: I wonder answer generated super quick vanish here?
            graphQueryOutput->addSubscriber(outChan_);
            */
        }
    }
}
