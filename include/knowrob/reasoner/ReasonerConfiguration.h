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
#include <memory>
#include <boost/property_tree/ptree.hpp>
#include "knowrob/terms/Term.h"
#include "knowrob/DataSource.h"

namespace knowrob {
	/**
	 * A configuration of a reasoner.
	 * Each instance of a reasoner type may have its own configuration.
	 */
	class ReasonerConfiguration {
	public:
		/**
		 * Load a reasoner configuration from a property tree.
		 * @param ptree a property tree.
		 */
		void loadPropertyTree(const boost::property_tree::ptree &ptree);

		std::list<std::pair<TermPtr,TermPtr>> settings;
		std::list<std::shared_ptr<DataSource>> dataSources;
	private:
		void loadSettings(const TermPtr &key_t, const boost::property_tree::ptree &ptree);
	};
}

#endif //KNOWROB_REASONER_CONFIGURATION_H_
