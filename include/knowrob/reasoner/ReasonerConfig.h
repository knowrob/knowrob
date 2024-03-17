/*
 * Copyright (c) 2022, Daniel Beßler
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_REASONER_CONFIGURATION_H_
#define KNOWROB_REASONER_CONFIGURATION_H_

#include <list>
#include <map>
#include <memory>
#include <optional>
#include <boost/property_tree/ptree.hpp>
#include "knowrob/terms/Term.h"
#include "knowrob/sources/DataSource.h"

namespace knowrob {
	/**
	 * Properties of a reasoner.
	 */
	class ReasonerConfig {
	public:
		ReasonerConfig();

		/**
		 * Load a reasoner configuration from a property tree.
		 * @param ptree a property tree.
		 */
        explicit ReasonerConfig(const boost::property_tree::ptree *ptree);

		/**
		 * key-value pairs of settings where the key uses "." as separator of levels in the tree.
		 * @param key a key.
		 * @return the value associated with the key.
		 */
		std::string_view get(std::string_view key, std::string_view defaultValue) const;

		/**
		 * Generate a term from a key string.
		 * @param key a key
		 * @param separator the separator used in the key term.
		 * @return a term representing the key.
		 */
		TermPtr createKeyTerm(std::string_view key, std::string_view separator) const;

		/**
		 * @return the begin iterator of the key-value pairs.
		 */
		auto begin() const { return properties_.begin(); }
		/**
		 * @return the end iterator of the key-value pairs.
		 */
		auto end() const { return properties_.end(); }

		/**
		 * @return the list of data sources that should be imported into the reasoner backend.
		 */
		auto& dataSources() const { return dataSources_; }

		/**
		 * @return the property tree used to generate this configuration.
		 */
		auto ptree() const { return ptree_; }

	private:
		void loadProperty(const std::string &key, const boost::property_tree::ptree &ptree);
		void loadRemainder(const std::string &key, const boost::property_tree::ptree &ptree);

		std::map<std::string, std::string> properties_;
		std::list<std::shared_ptr<DataSource>> dataSources_;
		const boost::property_tree::ptree *ptree_;
	};
}

#endif //KNOWROB_REASONER_CONFIGURATION_H_
