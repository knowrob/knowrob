//
// Created by daniel on 12.03.23.
//

#include <gtest/gtest.h>
#include <fstream>
#include <filesystem>
#include <boost/spirit/include/qi.hpp>
#include <boost/spirit/include/phoenix_operator.hpp>
#include <utility>

#include "knowrob/Logger.h"
#include "knowrob/semweb/KnowledgeGraph.h"
#include "knowrob/semweb/xsd.h"
#include "knowrob/queries/QueryError.h"

namespace fs = std::filesystem;
namespace qi = boost::spirit::qi;
namespace ascii = boost::spirit::ascii;
using namespace std::chrono_literals;
using namespace knowrob;

namespace knowrob {
    class GraphQueryRunner : public ThreadPool::Runner {
    public:
        KnowledgeGraph *kg_;
        GraphQueryPtr query_;
        AnswerBufferPtr result_;

        // FIXME: use of unsmart pointer for kg is unsafe
        GraphQueryRunner(
                KnowledgeGraph *kg,
                GraphQueryPtr query,
                AnswerBufferPtr &result)
        : kg_(kg), query_(std::move(query)), result_(result), ThreadPool::Runner()
        {}

        void run() override { kg_->evaluateQuery(query_, result_); }
    };
}

template <typename TP> std::time_t to_time_t(TP tp) {
    // needed to convert file modification time to string :/
    using namespace std::chrono;
    auto sctp = time_point_cast<system_clock::duration>(tp - TP::clock::now() + system_clock::now());
    return system_clock::to_time_t(sctp);
}

struct TripleHandler {
    explicit TripleHandler(ITripleLoader *loader) : loader(loader), triple() {}
    ITripleLoader *loader;
    TripleData triple;
};

static RDFType getRDFTypeFromURI(const char *typeURI)
{
    if(!typeURI || xsd::isStringType(typeURI))
        return RDF_STRING_LITERAL;
    else if(xsd::isIntegerType(typeURI))
        return RDF_INT64_LITERAL;
    else if(xsd::isDoubleType(typeURI))
        return RDF_DOUBLE_LITERAL;
    else if(xsd::isBooleanType(typeURI))
        return RDF_BOOLEAN_LITERAL;
    else {
        KB_WARN("Unknown data type {} treated as string.", typeURI);
        return RDF_STRING_LITERAL;
    }
}

static void raptor_log(void*, raptor_log_message *message) {
    switch(message->level) {
        case RAPTOR_LOG_LEVEL_NONE:
        case RAPTOR_LOG_LEVEL_TRACE:
        case RAPTOR_LOG_LEVEL_DEBUG:
        case RAPTOR_LOG_LEVEL_INFO:
            break;
        case RAPTOR_LOG_LEVEL_WARN:
            KB_WARN("[raptor] {}", message->text);
            break;
        case RAPTOR_LOG_LEVEL_ERROR:
        case RAPTOR_LOG_LEVEL_FATAL:
            KB_ERROR("[raptor] {}", message->text);
            break;
    }
}

static void processTriple(void* user_data, raptor_statement* triple)
{
    auto *handler = (TripleHandler*)user_data;

    if(!triple->subject || !triple->predicate || !triple->object) {
        return;
    }

    // read predicate
    handler->triple.predicate = (const char*) raptor_uri_as_string(triple->predicate->value.uri);
    // read subject
    if(triple->subject->type == RAPTOR_TERM_TYPE_BLANK)
        handler->triple.subject = (const char*) triple->subject->value.blank.string;
    else
        handler->triple.subject = (const char*) raptor_uri_as_string(triple->subject->value.uri);

    // read object
    if(triple->object->type == RAPTOR_TERM_TYPE_BLANK) {
        handler->triple.object = (const char*) triple->object->value.blank.string;
        handler->triple.objectType = RDF_RESOURCE;
    }
    else if(triple->object->type == RAPTOR_TERM_TYPE_LITERAL) {
        handler->triple.object = (const char*) triple->object->value.literal.string;
        // parse literal type
        if(triple->object->value.literal.datatype) {
            handler->triple.objectType = getRDFTypeFromURI((const char*)
                    raptor_uri_as_string(triple->object->value.literal.datatype));
        }
        else {
            handler->triple.objectType = RDF_STRING_LITERAL;
        }
        switch(handler->triple.objectType) {
            case RDF_RESOURCE:
            case RDF_STRING_LITERAL:
                break;
            case RDF_DOUBLE_LITERAL:
                handler->triple.objectDouble = strtod(handler->triple.object, nullptr);
                break;
            case RDF_INT64_LITERAL:
                handler->triple.objectInteger = strtol(handler->triple.object, nullptr, 10);
                break;
            case RDF_BOOLEAN_LITERAL: {
                bool objectValue;
                std::istringstream(handler->triple.object) >> objectValue;
                handler->triple.objectInteger = objectValue;
                break;
            }
        }
    }
    else {
        handler->triple.object = (const char*) raptor_uri_as_string(triple->object->value.uri);
        handler->triple.objectType = RDF_RESOURCE;
    }

    handler->loader->loadTriple(handler->triple);
}


KnowledgeGraph::KnowledgeGraph(ThreadPool *threadPool)
: raptorWorld_(raptor_new_world()),
  vocabulary_(std::make_shared<semweb::Vocabulary>()),
  threadPool_(threadPool)
{
    // TODO: reconsider handling of raptor worlds, e.g. some reasoner could
    //       access data from raptor API, but in some cases it could be desired
    //       to keep all the data only in some database
    raptor_world_set_log_handler(raptorWorld_, nullptr, raptor_log);
    // TODO: raptor can report namespaces
    //raptor_parser_set_namespace_handler(rdf_parser, user_data, namespaces_handler);
    //void namespaces_handler(void* user_data, raptor_namespace *nspace) { }

    if(raptor_world_open(raptorWorld_) != 0) {
        KB_WARN("failed to initialize raptor library.");
    }
}

KnowledgeGraph::~KnowledgeGraph()
{
    raptor_free_world(raptorWorld_);
}

bool KnowledgeGraph::isDefinedResource(const std::string_view &iri)
{
    return isDefinedClass(iri) || isDefinedProperty(iri);
}

bool KnowledgeGraph::isDefinedProperty(const std::string_view &iri)
{
    return vocabulary_->isDefinedProperty(iri);
}

bool KnowledgeGraph::isDefinedClass(const std::string_view &iri)
{
    return vocabulary_->isDefinedClass(iri);
}

AnswerBufferPtr KnowledgeGraph::submitQuery(const GraphQueryPtr &query)
{
    AnswerBufferPtr result = std::make_shared<AnswerBuffer>();
    auto runner = std::make_shared<GraphQueryRunner>(this, query, result);
    threadPool_->pushWork(runner);
    return result;
}

bool KnowledgeGraph::loadURI(ITripleLoader &loader,
                             const std::string &uriString,
                             std::string &blankPrefix,
                             TripleFormat format)
{
    // create a raptor parser
    raptor_parser *parser;
    switch(format) {
        case RDF_XML:
            parser = raptor_new_parser(raptorWorld_, "rdfxml");
            break;
        case TURTLE:
            parser = raptor_new_parser(raptorWorld_, "turtle");
            break;
        case N_TRIPLES:
            parser = raptor_new_parser(raptorWorld_, "ntriples");
            break;
    }

    // pass TripleHandler as user data to statement_handler of raptor
    TripleHandler handler(&loader);
    raptor_parser_set_statement_handler(parser, &handler, processTriple);
    // make sure blanks are generated with proper prefix.
    // FIXME: changing this globally makes it impossible to call this function from multiple threads.
    //      - add a mutex
    //      - any way this can be changed on per-parser level?
    //      - else one could have short living raptor worlds, one for each call
    raptor_world_set_generate_bnodeid_parameters(raptorWorld_, blankPrefix.data(), 1);

    raptor_uri *uri, *base_uri;
    int result;
    if(fs::exists(uriString)) {
        auto escapedString = raptor_uri_filename_to_uri_string(uriString.c_str());
        uri = raptor_new_uri(raptorWorld_, (unsigned char*)escapedString);
        // Parse the content of a file URI
        base_uri = raptor_uri_copy(uri);
        result = raptor_parser_parse_file(parser, uri, base_uri);
        raptor_free_memory(escapedString);
    }
    else {
        uri = raptor_new_uri(raptorWorld_, (const unsigned char *)uriString.c_str());
        // Parse the content from a URI
        base_uri = raptor_uri_copy(uri);
        result = raptor_parser_parse_uri(parser, uri, base_uri);
    }

    // cleanup
    loader.flush();
    raptor_free_parser(parser);
    raptor_free_uri(uri);
    raptor_free_uri(base_uri);

    // raptor returns 0 on success
    return (result==0);
}

std::string KnowledgeGraph::getNameFromURI(const std::string &uriString)
{
    return fs::path(uriString).stem();
}

std::string KnowledgeGraph::getVersionFromURI(const std::string &uriString)
{
    fs::path p(uriString);

    // check if it is a local existing file and use file modification time
    // as version in this case.
    // FIXME: creates confusion if a local file is switched to a new remote version.
    if(exists(p)) {
        std::ostringstream oss;
        auto stamp = last_write_time(p);
        auto tt = to_time_t(stamp);
        auto tm = *std::localtime(&tt);
        oss << std::put_time(&tm, "%c");
        return oss.str();
    }

    // try to extract version from URI
    auto versionString = p.parent_path().filename();
    if(isVersionString(versionString)) {
        return versionString;
    }

    // fallback to use the current day as version, thus causing
    // a reload each day.
    {
        std::ostringstream oss;
        auto t = std::time(nullptr);
        auto tm = *std::localtime(&t);
        oss << std::put_time(&tm, "%d-%m-%Y");
        return oss.str();
    }
}

bool KnowledgeGraph::isVersionString(const std::string &versionString)
{
    // parser rules
    qi::rule<std::string::const_iterator> numbers = (
            (+qi::digit >> '.' >> +qi::digit >> '.' >> +qi::digit) |
            (+qi::digit >> '.' >> +qi::digit));
    // parse input
    auto first = versionString.begin();
    auto last  = versionString.end();
    bool r = qi::phrase_parse(first,
                              last,
                              (('v' >> numbers) | numbers),
                              ascii::space);
    // return true if parser succeeded
    return (first == last && r);
}


// fixture class for testing
class KnowledgeGraphTest : public ::testing::Test {
protected:
    // void SetUp() override {}
    // void TearDown() override {}
};

TEST_F(KnowledgeGraphTest, IsGraphVersionString)
{
    EXPECT_TRUE(KnowledgeGraph::isVersionString("v1.1"));
    EXPECT_TRUE(KnowledgeGraph::isVersionString("v10.1.54"));
    EXPECT_TRUE(KnowledgeGraph::isVersionString("1.1"));
    EXPECT_TRUE(KnowledgeGraph::isVersionString("10.1.54"));
    EXPECT_FALSE(KnowledgeGraph::isVersionString("10"));
    EXPECT_FALSE(KnowledgeGraph::isVersionString("x10.54.3"));
    EXPECT_FALSE(KnowledgeGraph::isVersionString("x.y.z"));
}

TEST_F(KnowledgeGraphTest, GraphNameFromURI)
{
    EXPECT_EQ(KnowledgeGraph::getNameFromURI("https://www.ontologydesignpatterns.org/ont/dul/DUL.owl"), "DUL");
    EXPECT_EQ(KnowledgeGraph::getNameFromURI("file:///owl/SOMA.owl"), "SOMA");
    EXPECT_EQ(KnowledgeGraph::getNameFromURI("./ont/SOMA.owl"), "SOMA");
    EXPECT_EQ(KnowledgeGraph::getNameFromURI("SOMA.owl"), "SOMA");
    EXPECT_EQ(KnowledgeGraph::getNameFromURI("SOMA"), "SOMA");
}

TEST_F(KnowledgeGraphTest, GraphVersionFromURI)
{
    EXPECT_EQ(KnowledgeGraph::getVersionFromURI("https://foo/v1.2.2/owl"), "v1.2.2");
}
