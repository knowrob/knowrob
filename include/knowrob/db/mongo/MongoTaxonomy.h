/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_MONGO_TAXONOMY_H
#define KNOWROB_MONGO_TAXONOMY_H

#include <vector>
#include <string>
#include <memory>
#include "Pipeline.h"
#include "Collection.h"

namespace knowrob::mongo {
	/**
	 * A class to update the taxonomy in a MongoDB.
	 */
	class MongoTaxonomy {
	public:
		using StringPair = std::pair<std::string_view, std::string_view>;

		MongoTaxonomy(const std::shared_ptr<mongo::Collection> &tripleCollection,
					  const std::shared_ptr<mongo::Collection> &oneCollection);

		void update(
				const std::vector<StringPair> &subClassAssertions,
				const std::vector<StringPair> &subPropertyAssertions);

	protected:
		std::shared_ptr<mongo::Collection> tripleCollection_;
		std::shared_ptr<mongo::Collection> oneCollection_;

		static void lookupParents(
				mongo::Pipeline &pipeline,
				const std::string_view &collectionName,
				const std::string_view &entity,
				const std::string_view &hierarchyRelation);

		static void updateHierarchyP(
				mongo::Pipeline &pipeline,
				const std::string_view &collection,
				const std::string_view &relation,
				const std::string_view &newChild);

		static void updateHierarchyO(
				mongo::Pipeline &pipeline,
				const std::string_view &collectionName,
				const std::string_view &relation,
				const std::string_view &newChild,
				const std::string_view &newParent);
	};

} // knowrob

#endif //KNOWROB_MONGO_TAXONOMY_H
