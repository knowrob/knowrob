//
// Created by daniel on 12.03.23.
//

#include <gtest/gtest.h>
#include <fstream>
#include <filesystem>
#include <boost/spirit/include/qi.hpp>
#include <boost/spirit/include/phoenix_operator.hpp>

#include "knowrob/Logger.h"
#include "knowrob/graphs/KnowledgeGraph.h"
#include "knowrob/graphs/xsd.h"

namespace fs = std::filesystem;
namespace qi = boost::spirit::qi;
namespace ascii = boost::spirit::ascii;
using namespace std::chrono_literals;
using namespace knowrob;

template <typename TP> std::time_t to_time_t(TP tp) {
    // needed to convert file modification time to string :/
    using namespace std::chrono;
    auto sctp = time_point_cast<system_clock::duration>(tp - TP::clock::now() + system_clock::now());
    return system_clock::to_time_t(sctp);
}

struct TripleHandler {
    explicit TripleHandler(TripleLoader *loader) : loader(loader) {}
    TripleLoader *loader;
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
    handler->triple.predicate = (const char*)
            raptor_uri_as_string(triple->predicate->value.uri);

    // read subject
    if(triple->subject->type == RAPTOR_TERM_TYPE_BLANK) {
        handler->triple.subject = (const char*) triple->subject->value.blank.string;
        handler->triple.subjectType = RDF_BLANK;
    }
    else {
        handler->triple.subject = (const char*)
                raptor_uri_as_string(triple->subject->value.uri);
        handler->triple.subjectType = RDF_RESOURCE;
    }

    // read object
    if(triple->object->type == RAPTOR_TERM_TYPE_BLANK) {
        handler->triple.object = (const char*) triple->object->value.blank.string;
        handler->triple.objectType = RDF_BLANK;
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
    }
    else {
        handler->triple.object = (const char*) raptor_uri_as_string(triple->object->value.uri);
        handler->triple.objectType = RDF_RESOURCE;
    }

    handler->loader->loadTriple(handler->triple);
}

KnowledgeGraph::KnowledgeGraph()
: raptorWorld_(raptor_new_world())
{
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

bool KnowledgeGraph::loadURI(TripleLoader &loader, const std::string &uriString, TripleFormat format)
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

std::string KnowledgeGraph::getGraphNameFromURI(const std::string &uriString)
{
    return fs::path(uriString).stem();
}

std::string KnowledgeGraph::getGraphVersionFromURI(const std::string &uriString)
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
    if(isGraphVersionString(versionString)) {
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

bool KnowledgeGraph::isGraphVersionString(const std::string &versionString)
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
    EXPECT_TRUE(KnowledgeGraph::isGraphVersionString("v1.1"));
    EXPECT_TRUE(KnowledgeGraph::isGraphVersionString("v10.1.54"));
    EXPECT_TRUE(KnowledgeGraph::isGraphVersionString("1.1"));
    EXPECT_TRUE(KnowledgeGraph::isGraphVersionString("10.1.54"));
    EXPECT_FALSE(KnowledgeGraph::isGraphVersionString("10"));
    EXPECT_FALSE(KnowledgeGraph::isGraphVersionString("x10.54.3"));
    EXPECT_FALSE(KnowledgeGraph::isGraphVersionString("x.y.z"));
}

TEST_F(KnowledgeGraphTest, GraphNameFromURI)
{
    EXPECT_EQ(KnowledgeGraph::getGraphNameFromURI("https://www.ontologydesignpatterns.org/ont/dul/DUL.owl"), "DUL");
    EXPECT_EQ(KnowledgeGraph::getGraphNameFromURI("file:///owl/SOMA.owl"), "SOMA");
    EXPECT_EQ(KnowledgeGraph::getGraphNameFromURI("./ont/SOMA.owl"), "SOMA");
    EXPECT_EQ(KnowledgeGraph::getGraphNameFromURI("SOMA.owl"), "SOMA");
    EXPECT_EQ(KnowledgeGraph::getGraphNameFromURI("SOMA"), "SOMA");
}

TEST_F(KnowledgeGraphTest, GraphVersionFromURI)
{
    EXPECT_EQ(KnowledgeGraph::getGraphVersionFromURI("https://foo/v1.2.2/owl"), "v1.2.2");
}
