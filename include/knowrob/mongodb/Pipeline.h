//
// Created by daniel on 06.04.23.
//

#ifndef KNOWROB_PIPELINE_H
#define KNOWROB_PIPELINE_H

#include <mongoc.h>
#include <list>

namespace knowrob::mongo::aggregation {

    class Pipeline {
    public:
        /**
         * @param arrayDocument an initialized array document.
         */
        explicit Pipeline(bson_t *arrayDocument);

        const auto arrayDocument() const { return arrayDocument_; }

        bson_t* appendStageBegin();
        bson_t* appendStageBegin(const char* stageOperator);
        void appendStageEnd(bson_t *stage);

        void unwind(const std::string_view &field);

        void replaceRoot(const std::string_view &newRootField);

        void merge(const std::string_view &collection);

        void project(const std::string_view &field);

        void project(const std::vector<std::string_view> &fields);

        void setUnion(const std::string_view &field, const std::vector<std::string_view> &sets);

        void addToArray(const std::string_view &key, const std::string_view &arrayKey, const std::string_view &elementKey);

    protected:
        bson_t *arrayDocument_;
        uint32_t numStages_;
        std::list<bson_t> stages_;
        std::list<bson_t> stageOperators_;
        std::list<bson_t> subDocuments_;

        bson_t *lastStage_;
        bson_t *lastOperator_;
    };

} // knowrob

#endif //KNOWROB_PIPELINE_H
