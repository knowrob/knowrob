//
// Created by daniel on 03.04.23.
//

#ifndef KNOWROB_MONGO_GRAPH_HIERARCHY_H
#define KNOWROB_MONGO_GRAPH_HIERARCHY_H

#include <mongoc.h>
#include <string>

static inline bson_t* mngUpdateKGHierarchyO(const std::string &collectionName,
                                            const char *relation,
                                            const char *newChild,
                                            const char *newParent)
{
    return BCON_NEW (
            "pipeline", "[",
            // lookup parent hierarchy
            "{", "$lookup", "{",
                "from", BCON_UTF8(collectionName.c_str()),
                "as", BCON_UTF8("parents"),
                "pipeline", "[",
                    "{", "$match", "{", "s", BCON_UTF8(newParent), "p", BCON_UTF8(relation), "}", "}",
                    "{", "$project", "{", "o*", BCON_INT32(1), "}", "}",
                    "{", "$unwind", BCON_UTF8("$o*"), "}",
                "]",
            "}", "}",
            // convert "parents" field from list of documents to list of strings.
            "{", "$set", "{", "parents", "{", "$map", "{",
                "input", BCON_UTF8("$parents"),
                "in", BCON_UTF8("$$this.o*"),
            "}", "}", "}", "}",
            // add newParent to parents array
            "{", "$set", "{", "parents", "{", "$concatArrays", "[",
                BCON_UTF8("$parents"), "[", BCON_UTF8(newParent), "]",
            "]", "}", "}", "}",
            // lookup documents that include the child in the o* (or p*) field
            "{", "$lookup", "{",
                "from", BCON_UTF8(collectionName.c_str()),
                "as", BCON_UTF8("doc"),
                "pipeline", "[",
                    "{", "$match", "{", "$or", "[",
                        "{", "s",  BCON_UTF8(newChild),
                             "p",  BCON_UTF8(relation),
                             "o",  BCON_UTF8(newParent), "}",
                        "{", "o*", BCON_UTF8(newChild), "}",
                    "]", "}", "}",
                    "{", "$project", "{", "o*", BCON_INT32(1), "}", "}",
                "]",
            "}", "}",
            "{", "$unwind", BCON_UTF8("$doc"), "}",
            // update the o*/p* field
            "{", "$set", "{", "doc.o*", "{", "$setUnion", "[",
                BCON_UTF8("$parents"), "{", "$cond", "{",
                    "if", "{", "$not", "[", BCON_UTF8("$doc.o*"), "]", "}",
                    "then", "[", "]",
                    "else", BCON_UTF8("$doc.o*"),
                "}", "}",
            "]", "}", "}", "}",
            // make the "doc" field the new root
            "{", "$replaceRoot", "{", "newRoot", BCON_UTF8("$doc"), "}", "}",
            // merge the result into the collection
            "{", "$merge", "{",
                "into", BCON_UTF8(collectionName.c_str()),
                "on", BCON_UTF8("_id"),
                "whenMatched", BCON_UTF8("merge"),
            "}", "}",
            "]");
}

static inline bson_t* mngUpdateKGHierarchyP(const std::string &collectionName,
                                            const char *relation,
                                            const char *newChild)
{
    return BCON_NEW (
            "pipeline", "[",
            // lookup hierarchy
            "{", "$lookup", "{",
                "from", BCON_UTF8(collectionName.c_str()),
                "as", BCON_UTF8("parents"),
                "pipeline", "[",
                    "{", "$match", "{", "s", BCON_UTF8(newChild), "p", BCON_UTF8(relation), "}", "}",
                    "{", "$project", "{", "o*", BCON_INT32(1), "}", "}",
                    "{", "$unwind", BCON_UTF8("$o*"), "}",
                "]",
            "}", "}",
            // convert "parents" field from list of documents to list of strings.
            "{", "$set", "{", "parents", "{", "$map", "{",
                "input", BCON_UTF8("$parents"),
                "in", BCON_UTF8("$$this.o*"),
            "}", "}", "}", "}",
            // lookup documents that include the child in the p* field
            "{", "$lookup", "{",
                "from", BCON_UTF8(collectionName.c_str()),
                "as", BCON_UTF8("doc"),
                "pipeline", "[",
                    "{", "$match", "{", "p*", BCON_UTF8(newChild), "}", "}",
                    "{", "$project", "{", "p*", BCON_INT32(1), "}", "}",
                "]",
            "}", "}",
            "{", "$unwind", BCON_UTF8("$doc"), "}",
            // update the o*/p* field
            "{", "$set", "{", "doc.p*", "{", "$setUnion", "[",
                BCON_UTF8("$parents"), "{", "$cond", "{",
                    "if", "{", "$not", "[", BCON_UTF8("$doc.p*"), "]", "}",
                    "then", "[", "]",
                    "else", BCON_UTF8("$doc.p*"),
                "}", "}",
            "]", "}", "}", "}",
            // make the "doc" field the new root
            "{", "$replaceRoot", "{", "newRoot", BCON_UTF8("$doc"), "}", "}",
            // merge the result into the collection
            "{", "$merge", "{",
                "into", BCON_UTF8(collectionName.c_str()),
                "on", BCON_UTF8("_id"),
                "whenMatched", BCON_UTF8("merge"),
            "}", "}",
            "]");
}

#endif //KNOWROB_MONGO_GRAPH_HIERARCHY_H
