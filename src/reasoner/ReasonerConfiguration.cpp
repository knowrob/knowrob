/*
 * Copyright (c) 2022, Daniel Beßler
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include "knowrob/Logger.h"
#include "knowrob/reasoner/ReasonerConfiguration.h"
#include "knowrob/terms/Constant.h"
#include "knowrob/terms/Predicate.h"

using namespace knowrob;

void ReasonerConfiguration::loadPropertyTree(const boost::property_tree::ptree &config)
{
	static const std::string formatDefault = {};

	// load all key-value pairs into settings list
	for(const auto& key_val : config) {
		auto key_t = std::make_shared<StringTerm>(key_val.first);
		if(key_val.second.empty()) {
			settings.emplace_back(key_t, std::make_shared<StringTerm>(key_val.second.get_value<std::string>()));
		}
		else {
			loadSettings(key_t, key_val.second);
		}
	}

	auto data_sources = config.get_child_optional("data-sources");
	if(data_sources) {
		for(const auto &pair : data_sources.value()) {
			auto &subtree = pair.second;

			auto fileValue = subtree.get_optional<std::string>("file");
			if(fileValue.has_value()) {
				auto fileFormat = subtree.get("format",formatDefault);
				dataFiles.push_back(std::make_shared<DataFile>(fileValue.value(), fileFormat));
			}
			else {
				KB_WARN("Ignoring data source without \"file\" key.");
			}
		}
	}
}

void ReasonerConfiguration::loadSettings( //NOLINT
		const TermPtr &key1, const boost::property_tree::ptree &ptree)
{
	static auto colon_f = std::make_shared<PredicateIndicator>(":", 2);

	for(const auto& key_val : ptree) {
		if(key_val.first.empty()) {
			// this indicates a list
			// TODO: handle list values here, needs an interface to translate a subtree to a term.
			//		recursive call of loadSettings wouldn't do the trick.
			/*
			auto &listData = key_val.second;
			std::vector<TermPtr> elements(listData.size());
			int index = 0;
			for(const auto& l_key_val : listData) {
				elements[index++] = std::make_shared<StringTerm>(l_key_val.second.get_value<std::string>());
			}
			settings.emplace_back(key1, std::make_shared<ListTerm>(elements));
			*/
			continue;
		}
		else {
			auto key2 = std::make_shared<StringTerm>(key_val.first);
			auto key_t = std::make_shared<Predicate>(Predicate(colon_f, { key1, key2 }));

			if(key_val.second.empty()) {
				settings.emplace_back(key_t, std::make_shared<StringTerm>(key_val.second.get_value<std::string>()));
			}
			else {
				loadSettings(key_t, key_val.second);
			}
		}
	}
}
