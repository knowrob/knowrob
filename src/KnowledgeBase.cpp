/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include <thread>
#include <utility>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <utility>

#include <knowrob/Logger.h>
#include <knowrob/KnowledgeBase.h>
#include <knowrob/URI.h>
#include <filesystem>
#include "knowrob/semweb/PrefixRegistry.h"
#include "knowrob/queries/QueryParser.h"
#include "knowrob/queries/QueryTree.h"
#include "knowrob/queries/ConjunctiveBroadcaster.h"
#include "knowrob/queries/IDBStage.h"
#include "knowrob/queries/EDBStage.h"
#include "knowrob/queries/NegationStage.h"
#include "knowrob/queries/ModalStage.h"
#include "knowrob/queries/QueryError.h"
#include "knowrob/KnowledgeBaseError.h"
#include "knowrob/queries/RedundantAnswerFilter.h"
#include "knowrob/queries/DisjunctiveBroadcaster.h"
#include "knowrob/queries/AnswerYes.h"
#include "knowrob/semweb/TripleFormat.h"
#include "knowrob/db/OntologyFile.h"
#include "knowrob/semweb/TripleContainer.h"
#include "knowrob/semweb/rdf.h"
#include "knowrob/semweb/owl.h"
#include "knowrob/semweb/rdfs.h"
#include "knowrob/db/DataTransaction.h"
#include "knowrob/semweb/OntologyLanguage.h"
#include "knowrob/db/OntologyParser.h"

#define KB_DEFAULT_TRIPLE_BATCH_SIZE 1000

using namespace std::chrono_literals;
using namespace knowrob;

namespace knowrob {
	class AnswerBuffer_WithReference : public TokenBuffer {
	public:
		explicit AnswerBuffer_WithReference(const std::shared_ptr<QueryPipeline> &pipeline)
				: TokenBuffer(), pipeline_(pipeline) {}

	protected:
		std::shared_ptr<QueryPipeline> pipeline_;
	};
}

KnowledgeBase::KnowledgeBase(const boost::property_tree::ptree &config)
		: vocabulary_(std::make_shared<semweb::Vocabulary>()),
		  importHierarchy_(std::make_unique<semweb::ImportHierarchy>()),
		  tripleBatchSize_(KB_DEFAULT_TRIPLE_BATCH_SIZE) {
	backendManager_ = std::make_unique<BackendManager>(this);
	reasonerManager_ = std::make_unique<ReasonerManager>(this, backendManager_);
	init(config);
}

KnowledgeBase::KnowledgeBase(const std::string_view &configFile)
		: vocabulary_(std::make_shared<semweb::Vocabulary>()),
		  importHierarchy_(std::make_unique<semweb::ImportHierarchy>()),
		  tripleBatchSize_(KB_DEFAULT_TRIPLE_BATCH_SIZE) {
	backendManager_ = std::make_unique<BackendManager>(this);
	reasonerManager_ = std::make_unique<ReasonerManager>(this, backendManager_);
	boost::property_tree::ptree config;
	boost::property_tree::read_json(URI::resolve(configFile), config);
	init(config);
}

KnowledgeBase::KnowledgeBase()
		: vocabulary_(std::make_shared<semweb::Vocabulary>()),
		  importHierarchy_(std::make_unique<semweb::ImportHierarchy>()),
		  tripleBatchSize_(KB_DEFAULT_TRIPLE_BATCH_SIZE) {
	backendManager_ = std::make_unique<BackendManager>(this);
	reasonerManager_ = std::make_unique<ReasonerManager>(this, backendManager_);
}

void KnowledgeBase::init(const boost::property_tree::ptree &config) {
	loadConfiguration(config);

	// load common ontologies
	for(auto &ontoPath : { "owl/rdf-schema.xml", "owl/owl.rdf" }) {
		loadDataSource(std::make_shared<OntologyFile>(URI(ontoPath), "rdf-xml"));
	}

	startReasoner();
}

KnowledgeBase::~KnowledgeBase() {
	// stop all reasoners
	stopReasoner();
}

const std::map<std::string, std::shared_ptr<DefinedReasoner>> &KnowledgeBase::reasonerPool() const {
	return reasonerManager_->reasonerPool();
}

void KnowledgeBase::startReasoner() {
	for (auto &pair: reasonerManager_->reasonerPool()) {
		try {
			pair.second->reasoner()->start();
		}
		catch (std::exception &e) {
			KB_ERROR("failed to start reasoner '{}': {}", pair.first, e.what());
		}
	}
}

void KnowledgeBase::stopReasoner() {
	for (auto &pair: reasonerManager_->reasonerPool()) {
		pair.second->reasoner()->stop();
	}
}

void KnowledgeBase::loadConfiguration(const boost::property_tree::ptree &config) {
	auto semwebTree = config.get_child_optional("semantic-web");
	if (semwebTree) {
		// load RDF URI aliases
		auto prefixesList = semwebTree.value().get_child_optional("prefixes");
		for (const auto &pair: prefixesList.value()) {
			auto alias = pair.second.get("alias", "");
			auto uri = pair.second.get("uri", "");
			if (!alias.empty() && !uri.empty()) {
				semweb::PrefixRegistry::get().registerPrefix(alias, uri);
			} else {
				KB_WARN("Invalid entry in semantic-web::prefixes, 'alias' and 'uri' must be defined.");
			}
		}
	}

	// initialize data backends from configuration
	auto backendList = config.get_child_optional("data-backends");
	if (backendList) {
		for (const auto &pair: backendList.value()) {
			try {
				backendManager_->loadBackend(pair.second);
			}
			catch (std::exception &e) {
				KB_ERROR("failed to load a knowledgeGraph: {}", e.what());
			}
		}
	} else {
		KB_ERROR("configuration has no 'backends' key.");
	}

	// get the central backend, ensure it implements KnowledgeGraph class.
	// TODO: remove notion of central backend entirely? some configurations may not require one.
	auto centralBackendName = config.get_optional<std::string>("central-backend");
	if (centralBackendName) {
		auto centralBackend = backendManager_->getBackendWithID(centralBackendName.value());
		if (!centralBackend) {
			KB_ERROR("failed to find central backend with ID '{}'.", centralBackendName.value());
		}
		centralKG_ = std::dynamic_pointer_cast<KnowledgeGraph>(centralBackend->backend());
		if (!centralKG_) {
			KB_ERROR("backend '{}' is not a type of KnowledgeGraph.", centralBackendName.value());
		}
	} else {
		KB_ERROR("configuration has no 'central-backend' key.");
	}
	if (!centralKG_) {
		throw KnowledgeBaseError("failed to initialize central knowledge graph.");
	}

	// TODO: support initial synch of persistent with non-persistent data backends.
	//       e.g. mongo KG could be filled with facts initially, these should be mirrored into
	//       other backends.

	// load reasoners from configuration
	auto reasonerList = config.get_child_optional("reasoner");
	if (reasonerList) {
		for (const auto &pair: reasonerList.value()) {
			try {
				auto definedReasoner = reasonerManager_->loadReasoner(pair.second);
				// if reasoner implements DataBackend class, add it to the backend manager
				auto reasonerBackend = std::dynamic_pointer_cast<DataBackend>(definedReasoner->reasoner());
				if (reasonerBackend) {
					backendManager_->addBackend(definedReasoner->name(), reasonerBackend);
				}
			}
			catch (std::exception &e) {
				KB_ERROR("failed to load a reasoner: {}", e.what());
			}
		}
	} else {
		KB_ERROR("configuration has no 'reasoner' key.");
	}

	// share vocabulary and import hierarchy with backends
	for (auto &pair: backendManager_->backendPool()) {
		auto definedBackend = pair.second;
		definedBackend->backend()->setVocabulary(vocabulary_);
		definedBackend->backend()->setImportHierarchy(importHierarchy_);
	}

	// load the "global" data sources.
	// these are data sources that are loaded into all backends, however
	// the backends may decide to ignore some of the data sources.
	auto dataSourcesList = config.get_child_optional("data-sources");
	if (dataSourcesList) {
		for (const auto &pair: dataSourcesList.value()) {
			auto &subtree = pair.second;
			auto dataSource = createDataSource(subtree);

			// TODO: read data source transformation setting here.
			//       add key "transformation", with sub-keys "path", "language", "format", ...
			// TODO: support data transformations for ontology languages
			//       basically load into a separate raptor world, transform this raptor world, then push it into
			//       all backends. Actually, some renaming transformations could be done in the event handler.
			//       but often we would rather add additional triples for alignment, e.g. if the alignment language is
			//       RDFS, then we can simply load the alignment ontology into the raptor world.
			// TODO: create semweb::Transformation object based on "transformation" setting
			//       and apply it to the data source loaded into a raptor world.

			if (!dataSource) {
				KB_ERROR("failed to create a data source");
			} else if (!loadDataSource(dataSource)) {
				KB_ERROR("failed to load a data source: {}", dataSource->uri());
			}
		}
	}
}

DataSourcePtr KnowledgeBase::createDataSource(const boost::property_tree::ptree &subtree) {
	static const std::string formatDefault = {};

	// read data source settings
	URI dataSourceURI(subtree);
	auto dataSourceFormat = subtree.get("format", formatDefault);
	auto o_dataSourceLanguage = subtree.get_optional<std::string>("language");
	auto dataSourceType = getDataSourceType(dataSourceFormat, o_dataSourceLanguage,
											subtree.get_optional<std::string>("type"));
	// TODO: support graph selector specification in settings
	//auto tripleFrame_ptree = subtree.get_child_optional("frame");
	//auto tripleFrame = tripleFrame_ptree ?
	//		std::make_shared<GraphSelector>(tripleFrame_ptree.value()) : nullptr;

	switch (dataSourceType) {
		case DataSourceType::ONTOLOGY: {
			auto ontoFile = std::make_shared<OntologyFile>(dataSourceURI, dataSourceFormat);
			if (o_dataSourceLanguage.has_value()) {
				ontoFile->setOntologyLanguage(semweb::ontologyLanguageFromString(o_dataSourceLanguage.value()));
			}
			//if (tripleFrame) {
			//	ontoFile->setFrame(tripleFrame);
			//}
			return ontoFile;
		}
		case DataSourceType::SPARQL:
			// TODO: support SPARQL services as data sources
			KB_WARN("sparql data sources are not supported yet.");
			return nullptr;
		case DataSourceType::UNSPECIFIED:
			return std::make_shared<DataFile>(dataSourceURI, dataSourceFormat);
	}

	return nullptr;
}

bool KnowledgeBase::isMaterializedInEDB(std::string_view property) const {
	return centralKG()->isDefinedProperty(property);
}

std::vector<RDFComputablePtr> KnowledgeBase::createComputationSequence(
		const std::list<DependencyNodePtr> &dependencyGroup) const {
	// Pick a node to start with.
	auto comparator = IDBComparator(vocabulary());
	DependencyNodePtr first;
	RDFComputablePtr firstComputable;
	for (auto &n: dependencyGroup) {
		auto computable_n =
				std::static_pointer_cast<RDFComputable>(n->literal());
		if (!first || comparator(firstComputable, computable_n)) {
			first = n;
			firstComputable = computable_n;
		}
	}

	// remember visited nodes, needed for circular dependencies
	// all nodes added to the queue should also be added to this set.
	std::set<DependencyNode *> visited;
	visited.insert(first.get());

	std::vector<RDFComputablePtr> sequence;
	sequence.push_back(firstComputable);

	// start with a FIFO queue only containing first node
	std::deque<std::shared_ptr<DependencyNodeQueue>> queue;
	auto qn0 = std::make_shared<DependencyNodeQueue>(first);
	queue.push_front(qn0);

	// loop until queue is empty and process exactly one successor of
	// the top element in the FIFO in each step. If an element has no
	// more successors, it can be removed from queue.
	// Each successor creates an additional node added to the top of the FIFO.
	while (!queue.empty()) {
		auto front = queue.front();

		// get top successor node that has not been visited yet
		DependencyNodePtr topNext;
		while (!front->neighbors_.empty()) {
			auto topNeighbor = front->neighbors_.top();
			front->neighbors_.pop();

			if (visited.count(topNeighbor.get()) == 0) {
				topNext = topNeighbor;
				break;
			}
		}
		// pop element from queue if all neighbors were processed
		if (front->neighbors_.empty()) {
			queue.pop_front();
		}

		if (topNext) {
			// push a new node onto FIFO
			auto qn_next = std::make_shared<DependencyNodeQueue>(topNext);
			queue.push_front(qn_next);
			sequence.push_back(std::static_pointer_cast<RDFComputable>(topNext->literal()));
			visited.insert(topNext.get());
		}
	}

	return sequence;
}

void KnowledgeBase::createComputationPipeline(
		const std::shared_ptr<QueryPipeline> &pipeline,
		const std::vector<RDFComputablePtr> &computableLiterals,
		const std::shared_ptr<TokenBroadcaster> &pipelineInput,
		const std::shared_ptr<TokenBroadcaster> &pipelineOutput,
		const QueryContextPtr &ctx) const {
	// This function generates a query pipeline for literals that can be computed
	// (EDB-only literals are processed separately). The literals are part of one dependency group.
	// They are sorted, and also evaluated in this order. For each computable literal there is at
	// least one reasoner that can compute the literal. However, instances of the literal may also
	// occur in the EDB. Hence, computation results must be combined with results of an EDB query
	// for each literal.

	auto lastOut = pipelineInput;
	auto edb = centralKG();

	for (auto &lit: computableLiterals) {
		auto stepInput = lastOut;
		auto stepOutput = std::make_shared<TokenBroadcaster>();
		pipeline->addStage(stepOutput);

		// --------------------------------------
		// Construct a pipeline that grounds the literal in the EDB.
		// But only add an EDB stage if the predicate was materialized in EDB before,
		// or if the predicate is a variable.
		// --------------------------------------
		bool isEDBStageNeeded = true;
		if (lit->propertyTerm() && lit->propertyTerm()->type() == TermType::STRING) {
			auto st = (StringTerm *) lit->propertyTerm().get();
			isEDBStageNeeded = isMaterializedInEDB(st->value());
		}
		if (isEDBStageNeeded) {
			auto edbStage = std::make_shared<EDBStage>(edb, lit, ctx);
			edbStage->selfWeakRef_ = edbStage;
			stepInput >> edbStage;
			edbStage >> stepOutput;
			pipeline->addStage(edbStage);
		}

		// --------------------------------------
		// Construct a pipeline that grounds the literal in the IDB.
		// To this end add an IDB stage for each reasoner that defines the literal.
		// --------------------------------------
		for (auto &r: lit->reasonerList()) {
			auto idbStage = std::make_shared<IDBStage>(r, lit, ctx);
			idbStage->selfWeakRef_ = idbStage;
			stepInput >> idbStage;
			idbStage >> stepOutput;
			pipeline->addStage(idbStage);
			// TODO: what about the materialization of the predicate in EDB?
			//       there is also the QUERY_FLAG_PERSIST_SOLUTIONS flag
		}

		// add a stage that consolidates the results of the EDB and IDB stages.
		// in particular the case needs to be handled where none of the stages return
		// 'true'. Also print a warning if two stages disagree but state they are confident.
		auto consolidator = std::make_shared<DisjunctiveBroadcaster>();
		stepOutput >> consolidator;
		// TODO: are all these addStage calls really needed?
		pipeline->addStage(consolidator);

		// --------------------------------------
		// Optionally add a stage to the pipeline that drops all redundant result.
		// The filter is applied here to remove redundancies early on directly after IDB and EDB
		// results are combined.
		// --------------------------------------
		if (ctx->queryFlags_ & QUERY_FLAG_UNIQUE_SOLUTIONS) {
			auto filterStage = std::make_shared<RedundantAnswerFilter>();
			stepOutput >> filterStage;
			lastOut = filterStage;
		} else {
			lastOut = stepOutput;
		}
	}

	lastOut >> pipelineOutput;
}

TokenBufferPtr KnowledgeBase::submitQuery(const ConjunctiveQueryPtr &graphQuery) {
	auto allLiterals = graphQuery->literals();

	// --------------------------------------
	// Construct a pipeline that holds references to stages.
	// --------------------------------------
	auto pipeline = std::make_shared<QueryPipeline>();

	// --------------------------------------
	// Pick a Knowledge Graph for EDB queries
	// --------------------------------------
	std::shared_ptr<KnowledgeGraph> kg = centralKG();

	// --------------------------------------
	// split input literals into positive and negative literals.
	// negative literals are evaluated in parallel after all positive literals.
	// --------------------------------------
	std::vector<RDFLiteralPtr> positiveLiterals, negativeLiterals;
	for (auto &l: allLiterals) {
		if (l->isNegated()) negativeLiterals.push_back(l);
		else positiveLiterals.push_back(l);
	}

	// --------------------------------------
	// split positive literals into edb-only and computable.
	// also associate list of reasoner to computable literals.
	// --------------------------------------
	std::vector<RDFLiteralPtr> edbOnlyLiterals;
	std::vector<RDFComputablePtr> computableLiterals;
	for (auto &l: positiveLiterals) {
		std::vector<std::shared_ptr<Reasoner>> l_reasoner;
		for (auto &pair: reasonerManager_->reasonerPool()) {
			auto &r = pair.second->reasoner();
			auto descr = r->getLiteralDescription(*l);
			if (descr) {
				// TODO: check descr, e.g. about materialization in EDB
				l_reasoner.push_back(r);
			}
		}
		if (l_reasoner.empty()) {
			edbOnlyLiterals.push_back(l);
			auto l_p = l->propertyTerm();
			if (l_p && l_p->type() == TermType::STRING &&
				!isMaterializedInEDB(std::static_pointer_cast<StringTerm>(l_p)->value())) {
				// generate a "don't know" message and return.
				auto out = std::make_shared<TokenBuffer>();
				auto channel = TokenStream::Channel::create(out);
				auto dontKnow = std::make_shared<AnswerDontKnow>();
				KB_WARN("Predicate {} is neither materialized in EDB nor defined by a reasoner.", *l->predicate());
				channel->push(dontKnow);
				channel->push(EndOfEvaluation::get());
				return out;
			}
		} else {
			computableLiterals.push_back(std::make_shared<RDFComputable>(*l, l_reasoner));
		}
	}

	// --------------------------------------
	// sort positive literals.
	// --------------------------------------
	std::sort(edbOnlyLiterals.begin(), edbOnlyLiterals.end(), EDBComparator(vocabulary()));

	// --------------------------------------
	// run EDB query with all edb-only literals.
	// --------------------------------------
	std::shared_ptr<TokenBuffer> edbOut;
	if (edbOnlyLiterals.empty()) {
		edbOut = std::make_shared<TokenBuffer>();
		auto channel = TokenStream::Channel::create(edbOut);
		channel->push(GenericYes());
		channel->push(EndOfEvaluation::get());
	} else {
		edbOut = kg->submitQuery(
				std::make_shared<ConjunctiveQuery>(edbOnlyLiterals, graphQuery->ctx()));
	}
	pipeline->addStage(edbOut);

	// --------------------------------------
	// handle positive IDB literals.
	// --------------------------------------
	std::shared_ptr<TokenBroadcaster> idbOut;
	if (computableLiterals.empty()) {
		idbOut = edbOut;
	} else {
		idbOut = std::make_shared<TokenBroadcaster>();
		pipeline->addStage(idbOut);
		// --------------------------------------
		// Compute dependency groups of computable literals.
		// --------------------------------------
		DependencyGraph dg;
		dg.insert(computableLiterals.begin(), computableLiterals.end());

		// --------------------------------------
		// Construct a pipeline for each dependency group.
		// --------------------------------------
		if (dg.numGroups() == 1) {
			auto &literalGroup = *dg.begin();
			createComputationPipeline(
					pipeline,
					createComputationSequence(literalGroup.member_),
					edbOut,
					idbOut,
					graphQuery->ctx());
		} else {
			// there are multiple dependency groups. They can be evaluated in parallel.

			// combines sub-answers computed in different parallel steps
			auto answerCombiner = std::make_shared<ConjunctiveBroadcaster>();
			// create a parallel step for each dependency group
			for (auto &literalGroup: dg) {
				// --------------------------------------
				// Construct a pipeline for each dependency group.
				// --------------------------------------
				createComputationPipeline(
						pipeline,
						createComputationSequence(literalGroup.member_),
						edbOut,
						answerCombiner,
						graphQuery->ctx());
			}
			answerCombiner >> idbOut;
			pipeline->addStage(answerCombiner);
		}
	}

	// --------------------------------------
	// Evaluate all negative literals in parallel.
	// --------------------------------------
	std::shared_ptr<TokenBroadcaster> lastStage;
	if (!negativeLiterals.empty()) {
		// run a dedicated stage where negated literals can be evaluated in parallel
		auto negStage = std::make_shared<LiteralNegationStage>(
				this, graphQuery->ctx(), negativeLiterals);
		idbOut >> negStage;
		lastStage = negStage;
	} else {
		lastStage = idbOut;
	}

	auto out = std::make_shared<AnswerBuffer_WithReference>(pipeline);
	lastStage >> out;
	edbOut->stopBuffering();
	return out;
}

TokenBufferPtr KnowledgeBase::submitQuery(const LiteralPtr &literal, const QueryContextPtr &ctx) {
	auto rdfLiteral = std::make_shared<RDFLiteral>(
			literal->predicate(), literal->isNegated(), ctx->selector_);
	return submitQuery(std::make_shared<ConjunctiveQuery>(
			ConjunctiveQuery({rdfLiteral}, ctx)));
}

TokenBufferPtr KnowledgeBase::submitQuery(const FormulaPtr &phi, const QueryContextPtr &ctx) {
	auto outStream = std::make_shared<TokenBuffer>();

	auto pipeline = std::make_shared<QueryPipeline>();
	pipeline->addStage(outStream);

	// --------------------------------------
	// Pick a Knowledge Graph for EDB queries
	// --------------------------------------
	std::shared_ptr<KnowledgeGraph> kg = centralKG();

	// decompose input formula into parts that are considered in disjunction,
	// and thus can be evaluated in parallel.
	QueryTree qt(phi);
	for (auto &path: qt) {
		// each node in a path is either a predicate, a negated predicate,
		// a modal formula, or the negation of a modal formula.

		std::vector<RDFLiteralPtr> posLiterals, negLiterals;
		std::vector<std::shared_ptr<ModalFormula>> posModals, negModals;

		for (auto &node: path.nodes()) {
			switch (node->type()) {
				case FormulaType::PREDICATE:
					posLiterals.push_back(std::make_shared<RDFLiteral>(
							std::static_pointer_cast<Predicate>(node),
							false, ctx->selector_));
					break;

				case FormulaType::MODAL:
					posModals.push_back(std::static_pointer_cast<ModalFormula>(node));
					break;

				case FormulaType::NEGATION: {
					auto negation = (Negation *) node.get();
					auto negated = negation->negatedFormula();
					switch (negated->type()) {
						case FormulaType::PREDICATE:
							negLiterals.push_back(std::make_shared<RDFLiteral>(
									std::static_pointer_cast<Predicate>(negated),
									true, ctx->selector_));
							break;
						case FormulaType::MODAL:
							negModals.push_back(std::static_pointer_cast<ModalFormula>(negated));
							break;
						default:
							throw QueryError("Unexpected negated formula type {} in QueryTree.", (int) negated->type());
					}
					break;
				}
				default:
					throw QueryError("Unexpected formula type {} in QueryTree.", (int) node->type());
			}
		}

		std::shared_ptr<TokenBroadcaster> lastStage;
		std::shared_ptr<TokenBuffer> firstBuffer;

		// first evaluate positive literals if any
		// note that the first stage is buffered, so that the next stage can be added to the pipeline
		// and only after stopping the buffering messages will be forwarded to the next stage.
		if (posLiterals.empty()) {
			// if there are none, we still need to indicate begin and end of stream for the rest of the pipeline.
			// so we just push `bos` (an empty substitution) followed by `eos` and feed these messages to the next stage.
			firstBuffer = std::make_shared<TokenBuffer>();
			lastStage = firstBuffer;
			auto channel = TokenStream::Channel::create(lastStage);
			channel->push(GenericYes());
			channel->push(EndOfEvaluation::get());
			pipeline->addStage(lastStage);
		} else {
			auto pathQuery = std::make_shared<ConjunctiveQuery>(posLiterals, ctx);
			firstBuffer = submitQuery(pathQuery);
			lastStage = firstBuffer;
			pipeline->addStage(lastStage);
		}

		// --------------------------------------
		// Evaluate all positive modals in sequence.
		// TODO: compute dependency between modals, evaluate independent modals in parallel.
		//       they could also be independent in evaluation context only, we could check which variables receive
		//       grounding already before in posLiteral query!
		//       this is effectively what is done in submitQuery(pathQuery)
		// --------------------------------------
		for (auto &posModal: posModals) {
			// FIXME: use of this pointer below. only ok if destructor makes sure to destroy all pipelines.
			//        maybe a better way would be having another class generating pipelines with the ability
			//        to create reference pointer on a KnowledgeBase
			auto modalStage = std::make_shared<ModalStage>(this, posModal, ctx);
			modalStage->selfWeakRef_ = modalStage;
			lastStage >> modalStage;
			lastStage = modalStage;
			pipeline->addStage(lastStage);
		}

		// --------------------------------------
		// Evaluate all negative literals in parallel.
		// --------------------------------------
		if (!negLiterals.empty()) {
			// run a dedicated stage where negated literals can be evaluated in parallel
			auto negLiteralStage = std::make_shared<LiteralNegationStage>(
					this, ctx, negLiterals);
			lastStage >> negLiteralStage;
			lastStage = negLiteralStage;
			pipeline->addStage(lastStage);
		}

		// --------------------------------------
		// Evaluate all negative modals in parallel.
		// --------------------------------------
		if (!negModals.empty()) {
			// run a dedicated stage where negated modals can be evaluated in parallel
			auto negModalStage = std::make_shared<ModalNegationStage>(
					this, ctx, negModals);
			lastStage >> negModalStage;
			lastStage = negModalStage;
			pipeline->addStage(lastStage);
		}

		lastStage >> outStream;
		firstBuffer->stopBuffering();
		pipeline->addStage(lastStage);
	}
	// At this point outStream could already contain solutions, but these are buffered
	// such that they won't be lost during pipeline creation.

	// if there were multiple paths, consolidate answers from them.
	// e.g. if one yields no and the other true, the no should be ignored.
	std::shared_ptr<TokenBroadcaster> finalStage;
	if (qt.numPaths() > 1) {
		auto consolidator = std::make_shared<DisjunctiveBroadcaster>();
		outStream >> consolidator;
		finalStage = consolidator;
	} else {
		finalStage = outStream;
	}

	// Finally, wrap output into AnswerBuffer_WithReference object.
	// Note that the AnswerBuffer_WithReference object is used such that the caller can
	// destroy the whole pipeline by de-referencing the returned AnswerBufferPtr.
	auto out = std::make_shared<AnswerBuffer_WithReference>(pipeline);
	finalStage >> out;
	outStream->stopBuffering();
	return out;
}

DataBackendPtr KnowledgeBase::findSourceBackend(const StatementData &triple) {
	if (!triple.graph) return {};

	auto definedBackend = backendManager_->getBackendWithID(triple.graph);
	if (definedBackend) return definedBackend->backend();

	auto definedReasoner = reasonerManager_->getReasonerWithID(triple.graph);
	if (definedReasoner) {
		return reasonerManager_->getReasonerBackend(definedReasoner);
	}

	return {};
}

template<typename T>
static inline std::shared_ptr<T> createTransaction(
		const std::shared_ptr<DefinedBackend> &definedBackend,
		const semweb::TripleContainerPtr &triples) {
	auto backend = definedBackend->backend();
	auto backendID = definedBackend->name();

	// create a worker goal that performs the transaction
	auto transaction = std::make_shared<T>(backend, backendID.c_str(), triples);
	// push goal to thread pool
	DefaultThreadPool()->pushWork(transaction,
								  [backendID](const std::exception &exc) {
									  KB_ERROR("transaction failed for backend '{}': {}", backendID, exc.what());
								  });
	return transaction;
}

static inline std::shared_ptr<ThreadPool::Runner> pushPerTripleWork(
		const semweb::TripleContainerPtr &triples,
		const std::function<void(const StatementData &)> &fn) {
	auto perTripleWorker =
			std::make_shared<ThreadPool::LambdaRunner>([fn, triples](const ThreadPool::LambdaRunner::StopChecker&) {
				std::for_each(triples->begin(), triples->end(), [fn](const StatementData &triple) {
					fn(triple);
				});
			});
	DefaultThreadPool()->pushWork(perTripleWorker,
								  [](const std::exception &exc) {
									  KB_ERROR("failed to update vocabulary: {}", exc.what());
								  });
	return perTripleWorker;
}

bool KnowledgeBase::insertOne(const StatementData &triple) {
	//if (!centralKG_->insertOne(triple)) {
	//	KB_WARN("assertion of triple into central backend failed!");
	//	return false;
	//}

	// find the source backend, if any
	auto sourceBackend = findSourceBackend(triple);
	// insert into all other backends
	for (auto &definedBackend: backendManager_->backendPool()) {
		// skip the central backend (handled above)
		if (definedBackend.second->backend() == centralKG_) continue;
		// skip the source backend
		if (definedBackend.second->backend() == sourceBackend) continue;

		if (!definedBackend.second->backend()->insertOne(triple)) {
			KB_WARN("assertion of triple into backend '{}' failed!", definedBackend.first);
		}
	}
	updateVocabularyInsert(triple);
	return true;
}

bool KnowledgeBase::removeOne(const StatementData &triple) {
	//if (!centralKG_->removeOne(triple)) {
	//	KB_WARN("deletion of triple from central backend failed!");
	//	return false;
	//}

	// find the source backend, if any
	auto sourceBackend = findSourceBackend(triple);
	// insert into all other backends
	for (auto &definedBackend: backendManager_->backendPool()) {
		// skip the central backend (handled above)
		//if (definedBackend.second->backend() == centralKG_) continue;
		// skip the source backend
		if (definedBackend.second->backend() == sourceBackend) continue;

		if (!definedBackend.second->backend()->removeOne(triple)) {
			KB_WARN("deletion of triple from backend '{}' failed!", definedBackend.first);
		}
	}
	updateVocabularyRemove(triple);
	return true;
}

bool KnowledgeBase::insertAll(const semweb::TripleContainerPtr &triples) {
	// TODO: maybe it would be best to at least enforce that transactions succeed in the central
	//       backend, and if not, rollback the transaction in the other backends. For this,
	//       all backends should be able to rollback the transactions.
	// TODO: apply filter before inserting into backends. each backend should be able to specify GraphSelector
	//       used for the filtering. e.g. historic triples should not be inserted into backends that do not support time.
	if (triples->empty()) return true;

	// find the source backend, if any.
	auto sourceBackend = findSourceBackend(*triples->begin());
	// push a worker goal that updates the vocabulary
	auto vocabWorker = pushPerTripleWork(triples,
										 [this](const StatementData &triple) {
											 updateVocabularyInsert(triple);
										 });

	// insert into all other backends. currently only a warning is printed if insertion fails for a backend.
	std::vector<std::shared_ptr<DataTransaction>> transactions;
	for (auto &definedBackend: backendManager_->backendPool()) {
		// skip the source backend
		if (definedBackend.second->backend() == sourceBackend) continue;
		transactions.push_back(createTransaction<DataInsertion>(definedBackend.second, triples));
	}

	// wait for all transactions to finish
	vocabWorker->join();
	for (auto &transaction: transactions) transaction->join();

	return true;
}

bool KnowledgeBase::removeAll(const semweb::TripleContainerPtr &triples) {
	if (triples->empty()) return true;

	// find the source backend, if any.
	auto sourceBackend = findSourceBackend(*triples->begin());
	// push a worker goal that updates the vocabulary
	auto vocabWorker = pushPerTripleWork(triples,
										 [this](const StatementData &triple) {
											 updateVocabularyRemove(triple);
										 });

	// remove from all backends. currently only a warning is printed if removal fails for a backend.
	std::vector<std::shared_ptr<DataTransaction>> transactions;
	for (auto &definedBackend: backendManager_->backendPool()) {
		if (definedBackend.second->backend() == sourceBackend) continue;
		transactions.push_back(createTransaction<DataRemoval>(definedBackend.second, triples));
	}

	// wait for all transactions to finish
	vocabWorker->join();
	for (auto &transaction: transactions) transaction->join();

	return true;
}

class ReasonerTripleContainer : public semweb::TripleContainer {
public:
	explicit ReasonerTripleContainer(const std::vector<StatementData> *triples) : triples_(triples) {}
	const std::vector<StatementData> &asVector() const override { return *triples_; }
protected:
	const std::vector<StatementData> *triples_;
};

bool KnowledgeBase::insertAll(const std::vector<StatementData> &triples) {
	// Note: insertAll blocks until the triples are inserted, so it is safe to use the triples vector as a pointer.
	return insertAll(std::make_shared<ReasonerTripleContainer>(&triples));
}

bool KnowledgeBase::removeAll(const std::vector<StatementData> &triples) {
	return removeAll(std::make_shared<ReasonerTripleContainer>(&triples));
}

bool KnowledgeBase::removeAllWithOrigin(std::string_view origin) {
	// remove all triples with a given origin from all backends.
	std::vector<std::shared_ptr<ThreadPool::Runner>> transactions;
	for (auto &it: backendManager_->backendPool()) {
		auto definedBackend = it.second;
		// create a worker goal that performs the transaction
		auto transaction = std::make_shared<ThreadPool::LambdaRunner>(
			[definedBackend, origin](const ThreadPool::LambdaRunner::StopChecker&) {
				if (!definedBackend->backend()->removeAllWithOrigin(origin)) {
					KB_WARN("removal of triples with origin '{}' from backend '{}' failed!", origin,
							definedBackend->name());
				}
			});
		// push goal to thread pool
		DefaultThreadPool()->pushWork(
				transaction,
				[definedBackend](const std::exception &exc) {
					KB_ERROR("transaction failed for backend '{}': {}", definedBackend->name(), exc.what());
				});
		transactions.push_back(transaction);
	}

	// TODO: also update vocabulary

	// wait for all transactions to finish
	for (auto &transaction: transactions) transaction->join();

	return true;
}

bool KnowledgeBase::removeAllMatching(const RDFLiteral &query) {
	bool all_succeed = true;
	for (auto &kg: backendManager_->backendPool()) {
		all_succeed = kg.second->backend()->removeAllMatching(query) && all_succeed;
	}
	return all_succeed;
}

void KnowledgeBase::updateVocabularyInsert(const StatementData &tripleData) {
	// keep track of imports, subclasses, and subproperties
	if (semweb::isSubClassOfIRI(tripleData.predicate)) {
		auto sub = vocabulary_->defineClass(tripleData.subject);
		auto sup = vocabulary_->defineClass(tripleData.object);
		sub->addDirectParent(sup);
	} else if (semweb::isSubPropertyOfIRI(tripleData.predicate)) {
		auto sub = vocabulary_->defineProperty(tripleData.subject);
		auto sup = vocabulary_->defineProperty(tripleData.object);
		sub->addDirectParent(sup);
	} else if (semweb::isTypeIRI(tripleData.predicate)) {
		vocabulary_->addResourceType(tripleData.subject, tripleData.object);
	} else if (semweb::isInverseOfIRI(tripleData.predicate)) {
		auto p = vocabulary_->defineProperty(tripleData.subject);
		auto q = vocabulary_->defineProperty(tripleData.object);
		p->setInverse(q);
		q->setInverse(p);
	} else if (semweb::isPropertyIRI(tripleData.predicate) || semweb::isClassIRI(tripleData.predicate)) {
		// increase assertion counter which is used in ordering metrics
		vocabulary_->increaseFrequency(tripleData.predicate);
	} else if (semweb::owl::imports == tripleData.predicate) {
		auto resolvedImport = URI::resolve(tripleData.object);
		auto importedGraph = DataSource::getNameFromURI(resolvedImport);
		if (tripleData.graph) {
			importHierarchy_->addDirectImport(tripleData.graph, importedGraph);
		} else {
			KB_WARN("import statement without graph");
		}
	}
}

void KnowledgeBase::updateVocabularyRemove(const StatementData &tripleData) {
	// TODO: implement
}

/**************************************************/
/*************** DATA SOURCES *********************/
/**************************************************/

DataSourceType KnowledgeBase::getDataSourceType(const std::string &format, const boost::optional<std::string> &language,
												const boost::optional<std::string> &type) {
	if (type) {
		if (type.value() == "ontology") return DataSourceType::ONTOLOGY;
		if (type.value() == "sparql") return DataSourceType::SPARQL;
	}
	if (language) {
		if (semweb::isOntologyLanguageString(language.value())) return DataSourceType::ONTOLOGY;
	}
	if (semweb::isTripleFormatString(format)) return DataSourceType::ONTOLOGY;
	return DataSourceType::UNSPECIFIED;
}

bool KnowledgeBase::loadDataSource(const DataSourcePtr &source) {
	switch (source->type()) {
		case DataSourceType::ONTOLOGY:
			return loadOntologyFile(std::static_pointer_cast<OntologyFile>(source));
		case DataSourceType::SPARQL:
			return loadSPARQLDataSource(source);
		case DataSourceType::UNSPECIFIED:
			return loadNonOntologySource(source);
	}
	return false;
}

bool KnowledgeBase::loadOntologyFile(const std::shared_ptr<OntologyFile> &source, bool followImports) {
	std::queue<std::string> ontologyURIs;
	// TODO resolve needed here, DS does it already or?
	ontologyURIs.push(URI::resolve(source->uri()));

	while (!ontologyURIs.empty()) {
		auto resolved = ontologyURIs.front();
		ontologyURIs.pop();
		auto origin = DataSource::getNameFromURI(resolved);
		auto newVersion = DataSource::getVersionFromURI(resolved);

		// check if ontology is already loaded
		if(centralKG_) {
			auto currentVersion = centralKG_->getVersionOfOrigin(origin);
			if (currentVersion) {
				// ontology was loaded before
				if (currentVersion == newVersion) continue;
				// delete old triples if a new version is loaded
				removeAllWithOrigin(origin);
			}
		}

		// some OWL files are downloaded compile-time via CMake,
		// they are downloaded into owl/external e.g. there are SOMA.owl and DUL.owl.
		// TODO: rework handling of cmake-downloaded ontologies, e.g. should also work when installed
		auto p = std::filesystem::path(KNOWROB_SOURCE_DIR) / "owl" / "external" /
				 std::filesystem::path(resolved).filename();
		const std::string *importURI = (exists(p) ? &p.native() : &resolved);

		KB_INFO("Loading ontology at '{}' with version "
				"\"{}\" and origin \"{}\".", *importURI, newVersion, origin);

		OntologyParser parser(*importURI, source->tripleFormat(), tripleBatchSize_);
		parser.setOrigin(origin);
		parser.setFrame(source->frame());
		// filter is called for each triple, if it returns false, the triple is skipped
		parser.setFilter([this](const StatementData &triple) {
			return !vocabulary_->isAnnotationProperty(triple.predicate);
		});
		// define a prefix for naming blank nodes
		parser.setBlankPrefix(std::string("_") + origin);
		auto result = parser.run([this](const semweb::TripleContainerPtr &tripleContainer) {
			insertAll(tripleContainer);
		});
		if (!result) {
			KB_WARN("Failed to parse ontology {} ({})", *importURI, source->uri());
			return false;
		}

		// update the version triple
		if(centralKG_) {
			centralKG_->setVersionOfOrigin(origin, newVersion);
		}

		// add direct import
		// TODO: why not build the import hierarchy recursively?
		if(source->parentOrigin().has_value()) {
			importHierarchy_->addDirectImport(source->parentOrigin().value(), origin);
		} else {
			importHierarchy_->addDirectImport(importHierarchy_->defaultGraph(), origin);
		}

		// load imported ontologies
		if (followImports) {
			for (auto &imported: parser.imports()) {
				ontologyURIs.push(URI::resolve(imported));
			}
		}
	}

	return true;
}

bool KnowledgeBase::loadSPARQLDataSource(const std::shared_ptr<DataSource> &source) {
	// TODO: support SPARQL services as data sources
	KB_WARN("SPARQL data sources are not supported yet.");
	return false;
}

bool KnowledgeBase::loadNonOntologySource(const DataSourcePtr &source) const {
	bool hasHandler = false;
	bool allSucceeded = true;

	for (auto &kg_pair: backendManager_->backendPool()) {
		auto backend = kg_pair.second->backend();
		if (backend->hasDataHandler(source)) {
			if (!backend->loadDataSource(source)) {
				allSucceeded = false;
				KB_WARN("backend '{}' failed to load data source '{}'", kg_pair.first, source->uri());
			}
			hasHandler = true;
		}
	}

	if (!hasHandler) {
		KB_WARN("no data handler for data source format '{}'", source->format());
	}

	return hasHandler && allSucceeded;
}

/**************************************************/
/************* ORDERING METRICS *******************/
/**************************************************/

KnowledgeBase::DependencyNodeQueue::DependencyNodeQueue(const DependencyNodePtr &node)
		: node_(node) {
	// add all nodes to a priority queue
	for (auto &neighbor: node->neighbors()) {
		neighbors_.push(neighbor);
	}
}

bool KnowledgeBase::DependencyNodeComparator::operator()(const DependencyNodePtr &a, const DependencyNodePtr &b) const {
	// - prefer node with less variables
	if (a->numVariables() != b->numVariables()) {
		return a->numVariables() > b->numVariables();
	}
	// - prefer node with less neighbors
	if (a->numNeighbors() != b->numNeighbors()) {
		return a->numNeighbors() > b->numNeighbors();
	}
	return a < b;
}

bool KnowledgeBase::EDBComparator::operator()(const RDFLiteralPtr &a, const RDFLiteralPtr &b) const {
	// - prefer evaluation of literals with fewer variables
	auto numVars_a = a->numVariables();
	auto numVars_b = b->numVariables();
	if (numVars_a != numVars_b) return (numVars_a > numVars_b);

	// - prefer literals with grounded predicate
	bool hasProperty_a = (a->propertyTerm() && a->propertyTerm()->type() == TermType::STRING);
	bool hasProperty_b = (b->propertyTerm() && b->propertyTerm()->type() == TermType::STRING);
	if (hasProperty_a != hasProperty_b) return (hasProperty_a < hasProperty_b);

	// - prefer properties that appear less often in the EDB
	if (hasProperty_a) {
		auto numAsserts_a = vocabulary_->frequency(
				std::static_pointer_cast<StringTerm>(a->propertyTerm())->value());
		auto numAsserts_b = vocabulary_->frequency(
				std::static_pointer_cast<StringTerm>(b->propertyTerm())->value());
		if (numAsserts_a != numAsserts_b) return (numAsserts_a > numAsserts_b);
	}

	return (a < b);
}

bool KnowledgeBase::IDBComparator::operator()(const RDFComputablePtr &a, const RDFComputablePtr &b) const {
	// - prefer evaluation of literals with fewer variables
	auto numVars_a = a->numVariables();
	auto numVars_b = b->numVariables();
	if (numVars_a != numVars_b) return (numVars_a > numVars_b);

	// - prefer literals with grounded predicate
	bool hasProperty_a = (a->propertyTerm() && a->propertyTerm()->type() == TermType::STRING);
	bool hasProperty_b = (b->propertyTerm() && b->propertyTerm()->type() == TermType::STRING);
	if (hasProperty_a != hasProperty_b) return (hasProperty_a < hasProperty_b);

	// - prefer literals with EDB assertions over literals without
	if (hasProperty_a) {
		auto hasEDBAssertion_a = vocabulary_->isDefinedProperty(
				std::static_pointer_cast<StringTerm>(a->propertyTerm())->value());
		auto hasEDBAssertion_b = vocabulary_->isDefinedProperty(
				std::static_pointer_cast<StringTerm>(b->propertyTerm())->value());
		if (hasEDBAssertion_a != hasEDBAssertion_b) return (hasEDBAssertion_a < hasEDBAssertion_b);
	}

	// - prefer properties that appear less often in the EDB
	if (hasProperty_a) {
		auto numAsserts_a = vocabulary_->frequency(
				std::static_pointer_cast<StringTerm>(a->propertyTerm())->value());
		auto numAsserts_b = vocabulary_->frequency(
				std::static_pointer_cast<StringTerm>(b->propertyTerm())->value());
		if (numAsserts_a != numAsserts_b) return (numAsserts_a > numAsserts_b);
	}

	// - prefer literals with more reasoner
	auto numReasoner_a = a->reasonerList().size();
	auto numReasoner_b = b->reasonerList().size();
	if (numReasoner_a != numReasoner_b) return (numReasoner_a < numReasoner_b);

	return (a < b);
}
