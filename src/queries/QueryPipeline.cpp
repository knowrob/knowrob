//
// Created by daniel on 27.03.23.
//

#include <gtest/gtest.h>

#include "knowrob/Logger.h"
#include "knowrob/queries/QueryPipeline.h"
#include "knowrob/queries/AnswerCombiner.h"
#include "knowrob/queries/DependencyGraph.h"
#include "knowrob/queries/QueryError.h"
#include "knowrob/queries/QueryParser.h"
#include "knowrob/modalities/KnowledgeModality.h"

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
        const DependencyNodePtr node_;
        std::list<std::shared_ptr<QueryPipelineNode>> successors_;
        std::priority_queue<DependencyNodePtr, std::vector<DependencyNodePtr>, CompareNodes> neighbors_;
    };
    using QueryPipelineNodePtr = std::shared_ptr<QueryPipelineNode>;
}


QueryPipelineStage::QueryPipelineStage(const std::list<LiteralPtr> &literals,
                                       const ModalityLabelPtr &label)
        : AnswerBroadcaster(),
          literals_(literals),
          label_(label),
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

std::shared_ptr<BufferedAnswerStream> QueryPipelineStage::submitQuery()
{
    // TODO: call some component to start a graph query here.
    //       best to create some interface dedicated just for the graph queries
    //       that can be implemented in different ways.
    // the parameters we have here are: std::list<LiteralPtr>, ModalityLabelPtr
    //return graphQueryEngine_->submitQuery(literals_, label_);
    return {};
}

void QueryPipelineStage::push(const AnswerPtr &partialResult)
{
    if(AnswerStream::isEOS(partialResult)) {
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

            auto queryAnswers = submitQuery();
            // TODO: msgs coming from queryResultStream need to be combined with partialResult before
            //       being published on outChan_!
            //       - could be done via a combiner node without unification that generates all permutations
            //         which is unproblematic in this case
            //queryResultStream->addSubscriber(outChan_);
        }
    }
}

void QueryPipelineStage::handleAnswer(const AnswerPtr &answer)
{
    pushToBroadcast(answer);
}


QueryPipeline::QueryPipeline(const std::shared_ptr<AnswerQueue> &outputQueue)
        : outputQueue_(outputQueue ? outputQueue : std::make_shared<AnswerQueue>())
{
    inputStream_ = std::make_shared<AnswerBroadcaster>();
    inputChannel_ = AnswerStream::Channel::create(inputStream_);
    // TODO: could use a simpler variant that does not use unification!
    outBroadcaster_ = std::make_shared<AnswerCombiner>();
    // feed broadcast into queue
    outBroadcaster_ >> outputQueue_;
}

void QueryPipeline::add(const std::list<DependencyNodePtr> &dependencyGroup)
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
    std::set<DependencyNode*> visited;
    visited.insert(first.get());

    // start with a FIFO queue only containing first node
    std::deque<QueryPipelineNodePtr> queue;
    auto qn0 = std::make_shared<QueryPipelineNode>(first);
    queue.push_front(qn0);

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

            if(visited.count(topNeighbor.get()) == 0) {
                topNext = topNeighbor;
                break;
            }
        }
        // pop element from queue if all neighbors were processed
        if(front->neighbors_.empty()) {
            queue.pop_front();
        }

        if(topNext) {
            // push a new node onto FIFO
            auto qn_next = std::make_shared<QueryPipelineNode>(topNext);
            queue.push_front(qn_next);
            front->successors_.push_back(qn_next);
            visited.insert(topNext.get());
        }
    }

    // now the pipeline can be build starting from qn0 following the successor relation.
    generate(qn0, inputStream_, outBroadcaster_);
}

void QueryPipeline::generate(const QueryPipelineNodePtr &qn, //NOLINT
                             const std::shared_ptr<AnswerBroadcaster> &qnInput,
                             const std::shared_ptr<AnswerBroadcaster> &pipelineOutput)
{
    // create an output stream for this node
    auto qnOutput = (qn->successors_.empty() ? pipelineOutput : std::make_shared<AnswerBroadcaster>());

    if(!qn->node_) {
        throw QueryError("invalid input: a node is null!");
    }

    // create a pipeline stage
    std::shared_ptr<QueryPipelineStage> stage;
    std::shared_ptr<ModalDependencyNode> modalNode;
    std::shared_ptr<LiteralDependencyNode> literalNode;
    if((modalNode = std::dynamic_pointer_cast<ModalDependencyNode>(qn->node_))) {
        stage = std::make_shared<QueryPipelineStage>(modalNode->literals(), modalNode->label());
    }
    else if((literalNode = std::dynamic_pointer_cast<LiteralDependencyNode>(qn->node_))) {
        stage = std::make_shared<QueryPipelineStage>(std::list<LiteralPtr>({ literalNode->literal() }));
    }
    else {
        throw QueryError("unexpected node type");
    }
    stages_.push_back(stage);
    // link stage with input and output
    qnInput >> stage;
    stage >> qnOutput;

    // continue for successors
    for(auto &successor : qn->successors_) {
        generate(successor, qnOutput, pipelineOutput);
    }
}

void QueryPipeline::run()
{
    // push empty message into in stream, and close the channel
    inputChannel_->push(AnswerStream::bos());
    inputChannel_->close();
}


// fixture class for testing
class QueryPipelineTest : public ::testing::Test {
protected:
    void SetUp() override {}
    void TearDown() override {}

    static std::shared_ptr<LiteralDependencyNode> make_literal(const std::string &literalString, bool isNegative) {
        return std::make_shared<LiteralDependencyNode>(std::make_shared<Literal>(
                QueryParser::parsePredicate(literalString), isNegative));
    }
    static std::shared_ptr<LiteralDependencyNode> literal_p(const std::string &literalString) {
        return make_literal(literalString, false);
    }
    static std::shared_ptr<LiteralDependencyNode> literal_n(const std::string &literalString) {
        return make_literal(literalString, true);
    }

    static void addDependency(std::shared_ptr<LiteralDependencyNode> &a,
                              std::shared_ptr<LiteralDependencyNode> &b) {
        a->addDependency(b);
        b->addDependency(a);
    }
};

TEST_F(QueryPipelineTest, OneLiteral)
{
    // create dependency group of literals with shared variables
    auto n11 = literal_p("p(a,X)");
    // generate a querying pipeline
    QueryPipeline qp;
    EXPECT_NO_THROW(qp.add({n11}));
    EXPECT_EQ(qp.numStages(), 1);
}

TEST_F(QueryPipelineTest, Path)
{
    // create dependency group of literals with shared variables
    auto n11 = literal_p("p(a,X)");
    auto n12 = literal_p("q(X,Y)");
    auto n13 = literal_p("r(Y,z)");
    addDependency(n11, n12);
    addDependency(n12, n13);
    // generate a querying pipeline
    QueryPipeline qp;
    EXPECT_NO_THROW(qp.add({n11, n12, n13}));
    EXPECT_EQ(qp.numStages(), 3);
}

TEST_F(QueryPipelineTest, Circle)
{
    // create dependency group of literals with shared variables
    auto n11 = literal_p("p(U,X)");
    auto n12 = literal_p("q(X,Y)");
    auto n13 = literal_p("r(Y,U)");
    addDependency(n11, n12);
    addDependency(n12, n13);
    addDependency(n13, n11);
    // generate a querying pipeline
    QueryPipeline qp;
    EXPECT_NO_THROW(qp.add({n11, n12, n13}));
    EXPECT_EQ(qp.numStages(), 3);
}

TEST_F(QueryPipelineTest, IndependentSubPaths)
{
    // create dependency group of literals with shared variables
    auto n11 = literal_p("p(a,X)");
    auto n12 = literal_p("q(X,X1)");
    auto n14 = literal_p("q(X1,b)");
    auto n13 = literal_p("r(X,Y1)");
    auto n15 = literal_p("r(Y1,c)");
    addDependency(n11, n12);
    addDependency(n11, n13);
    addDependency(n12, n14);
    addDependency(n13, n15);
    // generate a querying pipeline
    QueryPipeline qp;
    EXPECT_NO_THROW(qp.add({n11, n12, n13, n14, n15}));
    EXPECT_EQ(qp.numStages(), 5);
}
