//
// Created by daniel on 06.04.23.
//

#ifndef KNOWROB_PIPELINE_H
#define KNOWROB_PIPELINE_H

#include <mongoc.h>
#include <list>
#include <map>
#include <string_view>
#include "bson-helper.h"

namespace knowrob::mongo::aggregation {
    /**
     * A aggregation pipeline.
     */
    class Pipeline {
    public:
        /**
         * @param arrayDocument an initialized array document.
         */
        explicit Pipeline(bson_t *arrayDocument);

        /**
         * @return the pipeline array.
         */
        const auto* arrayDocument() const { return arrayDocument_; }

        bson_t* appendStageBegin();
        bson_t* appendStageBegin(const char* stageOperator);
        void appendStageEnd(bson_t *stage);

        /**
         * Append a $limit stage.
         * @param maxDocuments limit of resulting documents.
         */
        void limit(uint32_t maxDocuments);

        /**
         * Append a $unwind stage.
         * @param field an array value
         */
        void unwind(const std::string_view &field);

        /**
         * Append a $unset stage.
         * @param field a document field
         */
        void unset(const std::string_view &field);

        /**
         * Append a $project stage.
         * @param field a document field to include in output documents
         */
        void project(const std::string_view &field);

        /**
         * Append a $project stage.
         * @param field document fields to include in output documents
         */
        void project(const std::vector<std::string_view> &fields);

        /**
         * Append a $replaceRoot stage.
         * @param newRootField a document field
         */
        void replaceRoot(const std::string_view &newRootField);

        /**
         * Append a $merge stage.
         * @param collection the output collection
         */
        void merge(const std::string_view &collection);

        /**
         * Append a $sort stage with ascending sort order.
         * @param newRootField a document field
         */
        void sortAscending(const std::string_view &field);

        /**
         * Append a $sort stage with descending sort order.
         * @param newRootField a document field
         */
        void sortDescending(const std::string_view &field);

        /**
         * Append a ($set o $setUnion) stage.
         * @param field a field to store the union of array
         * @param sets list of array values
         */
        void setUnion(const std::string_view &field, const std::vector<std::string_view> &sets);

        /**
         * Add an element to an array.
         * @param key the output filed
         * @param arrayKey input array field
         * @param elementKey field of an additional element
         */
        void addToArray(const std::string_view &key, const std::string_view &arrayKey, const std::string_view &elementKey);

        /**
		 * Load a pipeline from a JSON file.
		 * @param filename the file name
		 * @param parameters a map of parameters
		 * @return a pipeline
		 */
        static bson_t* loadFromJSON(std::string_view filename, const std::map<std::string, std::string> &parameters);

    protected:
        bson_t *arrayDocument_;
        uint32_t numStages_;
        std::list<bson_wrapper> stages_;
        std::list<bson_wrapper> stageOperators_;

        bson_t *lastStage_;
        bson_t *lastOperator_;
    };

} // knowrob

#endif //KNOWROB_PIPELINE_H
