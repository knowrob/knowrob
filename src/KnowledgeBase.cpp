/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include <thread>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

#include <knowrob/Logger.h>
#include <knowrob/KnowledgeBase.h>
#include <knowrob/URI.h>
#include <filesystem>
#include "knowrob/semweb/PrefixRegistry.h"
#include "knowrob/queries/QueryParser.h"
#include "knowrob/queries/QueryTree.h"
#include "knowrob/queries/ConjunctiveBroadcaster.h"
#include "knowrob/queries/QueryStage.h"
#include "knowrob/queries/NegationStage.h"
#include "knowrob/queries/ModalStage.h"
#include "knowrob/queries/QueryError.h"
#include "knowrob/queries/RedundantAnswerFilter.h"
#include "knowrob/queries/DisjunctiveBroadcaster.h"
#include "knowrob/queries/AnswerYes.h"
#include "knowrob/triples/TripleFormat.h"
#include "knowrob/sources/OntologyFile.h"
#include "knowrob/sources/SPARQLService.h"
#include "knowrob/triples/TripleContainer.h"
#include "knowrob/semweb/rdf.h"
#include "knowrob/semweb/owl.h"
#include "knowrob/semweb/rdfs.h"
#include "knowrob/backend/BackendTransaction.h"
#include "knowrob/semweb/OntologyLanguage.h"
#include "knowrob/sources/OntologyParser.h"
#include "knowrob/backend/BackendInterface.h"
#include "knowrob/KnowRobError.h"
#include "knowrob/py/PythonError.h"

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

KnowledgeBase::KnowledgeBase()
		: isInitialized_(false) {
	vocabulary_ = std::make_shared<Vocabulary>();
	// use "system" as default origin until initialization completed
	vocabulary_->importHierarchy()->setDefaultGraph(ImportHierarchy::ORIGIN_SYSTEM);
	backendManager_ = std::make_shared<BackendManager>(vocabulary_);
	reasonerManager_ = std::make_shared<ReasonerManager>(this, backendManager_);
	edb_ = std::make_shared<BackendInterface>(backendManager_);
}

KnowledgeBase::KnowledgeBase(const boost::property_tree::ptree &config) : KnowledgeBase() {
	configure(config);
	init();
}

KnowledgeBase::KnowledgeBase(const std::string_view &configFile) : KnowledgeBase() {
	boost::property_tree::ptree config;
	boost::property_tree::read_json(URI::resolve(configFile), config);
	configure(config);
	init();
}

KnowledgeBase::~KnowledgeBase() {
	stopReasoner();
}

void KnowledgeBase::init() {
	isInitialized_ = true;
	vocabulary_->importHierarchy()->setDefaultGraph(ImportHierarchy::ORIGIN_USER);
	initBackends();
	synchronizeBackends();
	initVocabulary();
	startReasoner();
}

void KnowledgeBase::initBackends() {
	for (auto &pair: backendManager_->backendPool()) {
		auto definedBackend = pair.second;
		definedBackend->backend()->setVocabulary(vocabulary_);
	}
}

void KnowledgeBase::synchronizeBackends() {
	// TODO: support initial synch of persistent with non-persistent data backends.
	//       e.g. mongo KG could be filled with facts initially, these should be mirrored into
	//       other backends.
	//       at least make sure no persistent backend has an outdated version of an ontology
}

void KnowledgeBase::initVocabulary() {
	auto v_s = std::make_shared<Variable>("?s");
	auto v_o = std::make_shared<Variable>("?o");

	for (auto &it: backendManager_->persistent()) {
		auto backend = it.second;

		// initialize the import hierarchy
		for (auto &origin: backend->getOrigins()) {
			vocabulary_->importHierarchy()->addDirectImport(vocabulary_->importHierarchy()->ORIGIN_SYSTEM, origin);
		}

		// iterate over all rdf:type assertions and add them to the vocabulary
		backend->match(FramedTriplePattern(v_s, rdf::type, v_o),
					   [this](const FramedTriple &triple) {
						   vocabulary_->addResourceType(triple.subject(), triple.valueAsString());
					   });
		// iterate over all rdfs::subClassOf assertions and add them to the vocabulary
		backend->match(FramedTriplePattern(v_s, rdfs::subClassOf, v_o),
					   [this](const FramedTriple &triple) {
						   vocabulary_->addSubClassOf(triple.subject(), triple.valueAsString());
					   });
		// iterate over all rdfs::subPropertyOf assertions and add them to the vocabulary
		backend->match(FramedTriplePattern(v_s, rdfs::subPropertyOf, v_o),
					   [this](const FramedTriple &triple) {
						   vocabulary_->addSubPropertyOf(triple.subject(), triple.valueAsString());
					   });
		// iterate over all owl::inverseOf assertions and add them to the vocabulary
		backend->match(FramedTriplePattern(v_s, owl::inverseOf, v_o),
					   [this](const FramedTriple &triple) {
						   vocabulary_->setInverseOf(triple.subject(), triple.valueAsString());
					   });

		// query number of assertions of each property/class.
		// this is useful information for optimizing the query planner.
		std::vector<semweb::PropertyPtr> reifiedProperties;
		backend->count([this, &reifiedProperties](std::string_view resource, uint64_t count) {
			// special handling for reified relations: they are concepts, but do also increase the relation counter
			auto reifiedProperty = vocabulary_->getDefinedReification(resource);
			if (reifiedProperty) reifiedProperties.push_back(reifiedProperty);
			vocabulary_->setFrequency(resource, count);
		});
		for (auto &p: reifiedProperties) {
			vocabulary_->increaseFrequency(p->iri());
		}
	}
}

void KnowledgeBase::configure(const boost::property_tree::ptree &config) {
	configurePrefixes(config);
	// initialize data backends from configuration
	configureBackends(config);
	// load reasoners from configuration
	configureReasoner(config);
	// share vocabulary and import hierarchy with backends
	initBackends();
	// load common ontologies
	loadCommon();
	// load the "global" data sources.
	// these are data sources that are loaded into all backends, however
	// the backends may decide to ignore some of the data sources.
	configureDataSources(config);
}

void KnowledgeBase::configurePrefixes(const boost::property_tree::ptree &config) {
	auto semwebTree = config.get_child_optional("semantic-web");
	if (semwebTree) {
		// load RDF URI aliases
		auto prefixesList = semwebTree.value().get_child_optional("prefixes");
		for (const auto &pair: prefixesList.value()) {
			auto alias = pair.second.get("alias", "");
			auto uri = pair.second.get("uri", "");
			if (!alias.empty() && !uri.empty()) {
				PrefixRegistry::registerPrefix(alias, uri);
			} else {
				KB_WARN("Invalid entry in semantic-web::prefixes, 'alias' and 'uri' must be defined.");
			}
		}
	}
}

void KnowledgeBase::configureBackends(const boost::property_tree::ptree &config) {
	auto backendList = config.get_child_optional("data-backends");
	if (backendList) {
		for (const auto &pair: backendList.value()) {
			KB_LOGGED_TRY_CATCH(pair.first, "load", {
				backendManager_->loadBackend(pair.second);
			});
		}
	} else {
		KB_ERROR("configuration has no 'backends' key.");
	}
}

void KnowledgeBase::configureReasoner(const boost::property_tree::ptree &config) {
	auto reasonerList = config.get_child_optional("reasoner");
	if (reasonerList) {
		for (const auto &pair: reasonerList.value()) {
			KB_LOGGED_TRY_CATCH(pair.first, "load", {
				auto definedReasoner = reasonerManager_->loadReasoner(pair.second);
				// if reasoner implements DataBackend class, add it to the backend manager
				auto reasonerBackend = std::dynamic_pointer_cast<DataBackend>(definedReasoner->reasoner());
				if (reasonerBackend) {
					backendManager_->addBackend(definedReasoner->name(), reasonerBackend);
				}
			});
		}
	} else {
		KB_ERROR("configuration has no 'reasoner' key.");
	}
}

void KnowledgeBase::configureDataSources(const boost::property_tree::ptree &config) {
	auto dataSourcesList = config.get_child_optional("data-sources");
	if (dataSourcesList) {
		for (const auto &pair: dataSourcesList.value()) {
			auto &subtree = pair.second;
			auto dataSource = createDataSource(subtree);

			// TODO: read data source transformation setting here.
			//       add key "transformation", with sub-keys "path", "language", "format", ...
			//       create semweb::Transformation object based on "transformation" setting
			//       and apply it to the data source loaded into a raptor world.

			if (!dataSource) {
				KB_ERROR("failed to create a data source");
			} else if (!loadDataSource(dataSource)) {
				KB_ERROR("failed to load a data source: {}", dataSource->uri());
			}
		}
	}
}

void KnowledgeBase::loadCommon() {
	for (auto &ontoPath: {"owl/rdf-schema.xml", "owl/owl.rdf"}) {
		loadDataSource(std::make_shared<OntologyFile>(URI(ontoPath), "rdf-xml"));
	}
}

void KnowledgeBase::startReasoner() {
	for (auto &pair: reasonerManager_->reasonerPool()) {
		// TODO: it would be better to remove reasoner and backends if they throw an exception.
		//       but doing this for e.g. query evaluation is more difficult where exception occur in a worker thread
		//       as part of a complex query evaluation pipeline.
		KB_LOGGED_TRY_CATCH(pair.first, "start", {
			pair.second->reasoner()->start();
		});
	}
}

void KnowledgeBase::stopReasoner() {
	for (auto &pair: reasonerManager_->reasonerPool()) {
		KB_LOGGED_TRY_CATCH(pair.first, "stop", {
			pair.second->reasoner()->stop();
		});
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
	// TODO: support graph selector specification in settings?
	//          then a data transformer could be created based on the setting.
	//          an idea would be e.g. to have a reasoner running for other perspectives.
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
		case DataSourceType::SPARQL: {
			auto sparqlService = std::make_shared<SPARQLService>(dataSourceURI, dataSourceFormat);
			//if (tripleFrame) {
			//	ontoFile->setFrame(tripleFrame);
			//}
			return sparqlService;
		}
		case DataSourceType::UNSPECIFIED:
			return std::make_shared<DataFile>(dataSourceURI, dataSourceFormat);
	}

	return nullptr;
}

bool KnowledgeBase::isMaterializedInEDB(std::string_view property) const {
	return vocabulary_->frequency(property) > 0;
}

QueryableBackendPtr KnowledgeBase::getBackendForQuery() const {
	auto &queryable = backendManager_->queryable();
	if (queryable.empty()) {
		KB_WARN("No queryable backends available.");
		return nullptr;
	} else {
		return queryable.begin()->second;
	}
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
		if (lit->propertyTerm() && lit->propertyTerm()->termType() == TermType::ATOMIC) {
			isEDBStageNeeded = isMaterializedInEDB(
					std::static_pointer_cast<Atomic>(lit->propertyTerm())->stringForm());
		}
		if (isEDBStageNeeded) {
			auto edb = getBackendForQuery();
			auto edbStage = std::make_shared<TypedQueryStage<FramedTriplePattern>>(
					ctx,
					lit,
					[&](const FramedTriplePatternPtr &q) {
						return edb_->getAnswerCursor(edb, std::make_shared<GraphPathQuery>(q, ctx));
					});
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
			auto idbStage = std::make_shared<TypedQueryStage<FramedTriplePattern>>(
					ctx, lit,
					[&r,&ctx](const FramedTriplePatternPtr &q) {
						return r->submitQuery(q, ctx);
					});
			idbStage->selfWeakRef_ = idbStage;
			stepInput >> idbStage;
			idbStage >> stepOutput;
			pipeline->addStage(idbStage);
			// TODO: what about the materialization of the predicate in EDB?
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
		if (ctx->queryFlags & QUERY_FLAG_UNIQUE_SOLUTIONS) {
			auto filterStage = std::make_shared<RedundantAnswerFilter>();
			stepOutput >> filterStage;
			lastOut = filterStage;
		} else {
			lastOut = stepOutput;
		}
	}

	lastOut >> pipelineOutput;
}

TokenBufferPtr KnowledgeBase::submitQuery(const GraphPathQueryPtr &graphQuery) {
	auto &allLiterals = graphQuery->path();

	// --------------------------------------
	// Construct a pipeline that holds references to stages.
	// --------------------------------------
	auto pipeline = std::make_shared<QueryPipeline>();

	// --------------------------------------
	// split input literals into positive and negative literals.
	// negative literals are evaluated in parallel after all positive literals.
	// --------------------------------------
	std::vector<FramedTriplePatternPtr> positiveLiterals, negativeLiterals;
	for (auto &l: allLiterals) {
		if (l->isNegated()) negativeLiterals.push_back(l);
		else positiveLiterals.push_back(l);
	}

	// --------------------------------------
	// split positive literals into edb-only and computable.
	// also associate list of reasoner to computable literals.
	// --------------------------------------
	std::vector<FramedTriplePatternPtr> edbOnlyLiterals;
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
			if (l_p && l_p->termType() == TermType::ATOMIC &&
				!isMaterializedInEDB(std::static_pointer_cast<Atomic>(l_p)->stringForm())) {
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
		auto edb = getBackendForQuery();
		edbOut = edb_->getAnswerCursor(edb,
									   std::make_shared<GraphPathQuery>(edbOnlyLiterals, graphQuery->ctx()));
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
		auto negStage = std::make_shared<PredicateNegationStage>(
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

TokenBufferPtr KnowledgeBase::submitQuery(const FirstOrderLiteralPtr &literal, const QueryContextPtr &ctx) {
	auto rdfLiteral = std::make_shared<FramedTriplePattern>(
			literal->predicate(), literal->isNegated());
	rdfLiteral->setTripleFrame(ctx->selector);
	return submitQuery(std::make_shared<GraphPathQuery>(
			GraphPathQuery({rdfLiteral}, ctx)));
}

TokenBufferPtr KnowledgeBase::submitQuery(const FormulaPtr &phi, const QueryContextPtr &ctx) {
	auto outStream = std::make_shared<TokenBuffer>();

	auto pipeline = std::make_shared<QueryPipeline>();
	pipeline->addStage(outStream);

	// decompose input formula into parts that are considered in disjunction,
	// and thus can be evaluated in parallel.
	QueryTree qt(phi);
	for (auto &path: qt) {
		// each node in a path is either a predicate, a negated predicate,
		// a modal formula, or the negation of a modal formula.

		std::vector<FramedTriplePatternPtr> posLiterals, negLiterals;
		std::vector<std::shared_ptr<ModalFormula>> posModals, negModals;

		for (auto &node: path.nodes()) {
			switch (node->type()) {
				case FormulaType::PREDICATE: {
					auto pat = std::make_shared<FramedTriplePattern>(
							std::static_pointer_cast<Predicate>(node), false);
					pat->setTripleFrame(ctx->selector);
					posLiterals.push_back(pat);
					break;
				}

				case FormulaType::MODAL:
					posModals.push_back(std::static_pointer_cast<ModalFormula>(node));
					break;

				case FormulaType::NEGATION: {
					auto negation = (Negation *) node.get();
					auto negated = negation->negatedFormula();
					switch (negated->type()) {
						case FormulaType::PREDICATE: {
							auto pat = std::make_shared<FramedTriplePattern>(
									std::static_pointer_cast<Predicate>(negated), true);
							pat->setTripleFrame(ctx->selector);
							negLiterals.push_back(pat);
							break;
						}
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
			auto pathQuery = std::make_shared<GraphPathQuery>(posLiterals, ctx);
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
			//        to create reference pointer on a KnowledgeBase, or use a weak ptr here
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
			auto negLiteralStage = std::make_shared<PredicateNegationStage>(
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

bool KnowledgeBase::insertOne(const FramedTriple &triple) {
	auto sourceBackend = findSourceBackend(triple);
	auto transaction = edb_->createTransaction(
			getBackendForQuery(),
			BackendInterface::Insert,
			BackendInterface::Excluding,
			{sourceBackend});
	return transaction->commit(triple);
}

bool KnowledgeBase::insertAll(const TripleContainerPtr &triples) {
	auto sourceBackend = findSourceBackend(**triples->begin());
	auto transaction = edb_->createTransaction(
			getBackendForQuery(),
			BackendInterface::Insert,
			BackendInterface::Excluding,
			{sourceBackend});
	return transaction->commit(triples);
}

bool KnowledgeBase::removeOne(const FramedTriple &triple) {
	auto sourceBackend = findSourceBackend(triple);
	auto transaction = edb_->createTransaction(
			getBackendForQuery(),
			BackendInterface::Remove,
			BackendInterface::Excluding,
			{sourceBackend});
	return transaction->commit(triple);
}

bool KnowledgeBase::removeAll(const TripleContainerPtr &triples) {
	auto sourceBackend = findSourceBackend(**triples->begin());
	auto transaction = edb_->createTransaction(
			getBackendForQuery(),
			BackendInterface::Remove,
			BackendInterface::Excluding,
			{sourceBackend});
	return transaction->commit(triples);
}

bool KnowledgeBase::insertAll(const std::vector<FramedTriplePtr> &triples) {
	// Note: insertAll blocks until the triples are inserted, so it is safe to use the triples vector as a pointer.
	return insertAll(std::make_shared<ProxyTripleContainer>(&triples));
}

bool KnowledgeBase::removeAll(const std::vector<FramedTriplePtr> &triples) {
	return removeAll(std::make_shared<ProxyTripleContainer>(&triples));
}

bool KnowledgeBase::removeAllWithOrigin(std::string_view origin) {
	return edb_->removeAllWithOrigin(origin);
}

std::shared_ptr<DefinedBackend> KnowledgeBase::findSourceBackend(const FramedTriple &triple) {
	if (!triple.graph()) return {};

	auto definedBackend_withID = backendManager_->getBackendWithID(triple.graph().value());
	if (definedBackend_withID) return definedBackend_withID;

	auto definedReasoner = reasonerManager_->getReasonerWithID(triple.graph().value());
	if (definedReasoner) {
		auto reasonerBackend = reasonerManager_->getReasonerBackend(definedReasoner);
		if (reasonerBackend) {
			for (auto &it: backendManager_->backendPool()) {
				auto &definedBackend_ofReasoner = it.second;
				if (definedBackend_ofReasoner->backend() == reasonerBackend) {
					return definedBackend_ofReasoner;
				}
			}
		}
	}

	return {};
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

std::optional<std::string> KnowledgeBase::getVersionOfOrigin(
		const std::shared_ptr<DefinedBackend> &definedBackend, std::string_view origin) const {
	// check if the origin was loaded before in this session
	auto runtimeVersion = definedBackend->getVersionOfOrigin(origin);
	if (runtimeVersion) return runtimeVersion;
	// otherwise check if the backend is persistent and if so, ask the persistent backend
	auto persistentBackend = backendManager_->persistent().find(definedBackend->name());
	if (persistentBackend != backendManager_->persistent().end()) {
		return persistentBackend->second->getVersionOfOrigin(origin);
	}
	return {};
}

std::vector<std::shared_ptr<DefinedBackend>>
KnowledgeBase::prepareLoad(std::string_view origin, std::string_view newVersion) const {
	std::vector<std::shared_ptr<DefinedBackend>> backendsToLoad;
	for (auto &it: backendManager_->backendPool()) {
		// check if the ontology is already loaded by the backend,
		// and if so whether it has the right version.
		auto definedBackend = it.second;
		auto currentVersion = getVersionOfOrigin(definedBackend, origin);
		if (currentVersion.has_value()) {
			if (currentVersion.value() != newVersion) {
				backendsToLoad.emplace_back(it.second);
				// TODO: rather include this operation as part of the transaction below
				definedBackend->backend()->removeAllWithOrigin(origin);
			}
		} else {
			backendsToLoad.emplace_back(it.second);
		}
	}
	return backendsToLoad;
}

void KnowledgeBase::finishLoad(const std::shared_ptr<OntologySource> &source, std::string_view origin,
							   std::string_view newVersion) {
	// update the version triple
	for (auto &it: backendManager_->backendPool()) {
		it.second->setVersionOfOrigin(origin, newVersion);
	}
	for (auto &it: backendManager_->persistent()) {
		auto persistentBackend = it.second;
		persistentBackend->setVersionOfOrigin(origin, newVersion);
	}

	// add direct import
	if (source->parentOrigin().has_value()) {
		vocabulary_->importHierarchy()->addDirectImport(source->parentOrigin().value(), origin);
	} else {
		vocabulary_->importHierarchy()->addDirectImport(vocabulary_->importHierarchy()->defaultGraph(), origin);
	}
}

bool KnowledgeBase::loadOntologyFile(const std::shared_ptr<OntologyFile> &source, bool followImports) {
	std::queue<std::string> ontologyURIs;
	ontologyURIs.push(URI::resolve(source->uri()));

	while (!ontologyURIs.empty()) {
		auto resolved = ontologyURIs.front();
		ontologyURIs.pop();
		auto origin = DataSource::getNameFromURI(resolved);
		auto newVersion = DataSource::getVersionFromURI(resolved);

		// check which backends need to load the ontology.
		auto backendsToLoad = prepareLoad(origin, newVersion);
		if (backendsToLoad.empty()) continue;

		// some OWL files are downloaded compile-time via CMake,
		// they are downloaded into owl/external e.g. there are SOMA.owl and DUL.owl.
		// TODO: rework handling of cmake-downloaded ontologies, e.g. should also work when installed
		auto p = std::filesystem::path(KNOWROB_SOURCE_DIR) / "owl" / "external" /
				 std::filesystem::path(resolved).filename();
		const std::string *importURI = (exists(p) ? &p.native() : &resolved);

		KB_INFO("Loading ontology at '{}' with version "
				"\"{}\" and origin \"{}\".", *importURI, newVersion, origin);

		OntologyParser parser(*importURI, source->tripleFormat());
		parser.setOrigin(origin);
		parser.setFrame(source->frame());
		// filter is called for each triple, if it returns false, the triple is skipped
		parser.setFilter([this](const FramedTriple &triple) {
			return !vocabulary_->isAnnotationProperty(triple.predicate());
		});
		// define a prefix for naming blank nodes
		parser.setBlankPrefix(std::string("_") + origin);
		auto result = parser.run([this, &backendsToLoad](const TripleContainerPtr &triples) {
			auto transaction = edb_->createTransaction(
					getBackendForQuery(),
					BackendInterface::Insert,
					BackendInterface::Including,
					backendsToLoad);
			transaction->commit(triples);
		});
		if (!result) {
			KB_WARN("Failed to parse ontology {} ({})", *importURI, source->uri());
			return false;
		}
		finishLoad(source, origin, newVersion);

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
	auto service = std::static_pointer_cast<SPARQLService>(source);
	auto serviceURI = URI::resolve(source->uri());
	auto origin = service->origin();
	// SPARQL does not have versioning. Some endpoints may store the version as a triple,
	// but this is not standardized. Some may encode version in the URI which we try to extract
	// below. Otherwise, we just use the current day as version causing a re-load every day.
	auto newVersion = DataSource::getVersionFromURI(serviceURI);

	// get all backends that do not have the data loaded yet
	auto backendsToLoad = prepareLoad(origin, newVersion);
	if (backendsToLoad.empty()) {
		// data is already loaded
		return true;
	}
	KB_INFO("Loading data from SPARQL endpoint at '{}' with version "
			"\"{}\" and origin \"{}\".", serviceURI, newVersion, origin);

	auto result = service->load([this, &backendsToLoad](const TripleContainerPtr &triples) {
		auto transaction = edb_->createTransaction(
				getBackendForQuery(),
				BackendInterface::Insert,
				BackendInterface::Including,
				backendsToLoad);
		transaction->commit(triples);
	});
	if (!result) {
		KB_WARN("Failed to load data from SPARQL service at {}.", serviceURI);
		return false;
	}
	finishLoad(service, origin, newVersion);

	return true;
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

bool KnowledgeBase::EDBComparator::operator()(const FramedTriplePatternPtr &a, const FramedTriplePatternPtr &b) const {
	// - prefer evaluation of literals with fewer variables
	auto numVars_a = a->numVariables();
	auto numVars_b = b->numVariables();
	if (numVars_a != numVars_b) return (numVars_a > numVars_b);

	// - prefer literals with grounded predicate
	bool hasProperty_a = (a->propertyTerm() && a->propertyTerm()->termType() == TermType::ATOMIC);
	bool hasProperty_b = (b->propertyTerm() && b->propertyTerm()->termType() == TermType::ATOMIC);
	if (hasProperty_a != hasProperty_b) return (hasProperty_a < hasProperty_b);

	// - prefer properties that appear less often in the EDB
	if (hasProperty_a) {
		auto numAsserts_a = vocabulary_->frequency(
				std::static_pointer_cast<Atomic>(a->propertyTerm())->stringForm());
		auto numAsserts_b = vocabulary_->frequency(
				std::static_pointer_cast<Atomic>(b->propertyTerm())->stringForm());
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
	bool hasProperty_a = (a->propertyTerm() && a->propertyTerm()->termType() == TermType::ATOMIC);
	bool hasProperty_b = (b->propertyTerm() && b->propertyTerm()->termType() == TermType::ATOMIC);
	if (hasProperty_a != hasProperty_b) return (hasProperty_a < hasProperty_b);

	// - prefer literals with EDB assertions over literals without
	if (hasProperty_a) {
		auto hasEDBAssertion_a = vocabulary_->isDefinedProperty(
				std::static_pointer_cast<Atomic>(a->propertyTerm())->stringForm());
		auto hasEDBAssertion_b = vocabulary_->isDefinedProperty(
				std::static_pointer_cast<Atomic>(b->propertyTerm())->stringForm());
		if (hasEDBAssertion_a != hasEDBAssertion_b) return (hasEDBAssertion_a < hasEDBAssertion_b);
	}

	// - prefer properties that appear less often in the EDB
	if (hasProperty_a) {
		auto numAsserts_a = vocabulary_->frequency(
				std::static_pointer_cast<Atomic>(a->propertyTerm())->stringForm());
		auto numAsserts_b = vocabulary_->frequency(
				std::static_pointer_cast<Atomic>(b->propertyTerm())->stringForm());
		if (numAsserts_a != numAsserts_b) return (numAsserts_a > numAsserts_b);
	}

	// - prefer literals with more reasoner
	auto numReasoner_a = a->reasonerList().size();
	auto numReasoner_b = b->reasonerList().size();
	if (numReasoner_a != numReasoner_b) return (numReasoner_a < numReasoner_b);

	return (a < b);
}
