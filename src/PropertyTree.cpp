/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include "knowrob/PropertyTree.h"
#include "knowrob/formulas/Predicate.h"
#include "knowrob/terms/Atom.h"
#include "knowrob/terms/Function.h"

using namespace knowrob;

PropertyTree::PropertyTree()
		: ptree_(nullptr) {}

PropertyTree::PropertyTree(const boost::property_tree::ptree *ptree)
		: ptree_(ptree) {
	static const std::string formatDefault = {};

	auto config = *ptree;

	// load all key-value pairs into settings map
	for (const auto &key_val: config) {
		loadProperty(key_val.first, key_val.second);
	}

	// process list of data sources that should be imported into the reasoner backend.
	auto data_sources = config.get_child_optional("imports");
	if (data_sources) {
		for (const auto &pair: data_sources.value()) {
			auto &subtree = pair.second;
			URI dataSourceURI(subtree);
			auto dataFormat = subtree.get("format", formatDefault);
			auto source = std::make_shared<DataSource>(dataSourceURI, dataFormat);
			dataSources_.push_back(source);
		}
	}
}

void PropertyTree::loadProperty( //NOLINT
		const std::string &key, const boost::property_tree::ptree &remainder) {
	if (remainder.empty()) {
		properties_.emplace(key, remainder.get_value<std::string>());
	} else {
		loadRemainder(key, remainder);
	}
}

void PropertyTree::loadRemainder( //NOLINT
		const std::string &key1, const boost::property_tree::ptree &remainder) {
	static auto colon_f = std::make_shared<PredicateIndicator>(":", 2);

	for (const auto &key_val: remainder) {
		if (key_val.first.empty()) {
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
		} else {
			//auto key2 = std::make_shared<StringTerm>(key_val.first);
			//auto key_t = std::make_shared<Predicate>(Predicate(colon_f, { key1, key2 }));
			auto key_t = key1 + ":" + std::string(key_val.first);
			loadProperty(key_t, key_val.second);
		}
	}
}

std::string_view PropertyTree::get(std::string_view key, std::string_view defaultValue) const {
	auto it = properties_.find(key.data());
	if (it != properties_.end()) {
		return it->second;
	} else {
		return defaultValue;
	}
}

TermPtr PropertyTree::createKeyTerm(std::string_view key, std::string_view delimiter) const {
	TermPtr last_key, next_key;

	size_t pos_start = 0, pos_end, delim_len = delimiter.length();
	std::string_view token;
	std::vector<std::string_view> res;

	while ((pos_end = key.find(delimiter, pos_start)) != std::string::npos) {
		token = key.substr(pos_start, pos_end - pos_start);
		pos_start = pos_end + delim_len;
		next_key = Atom::Tabled(token);
		if (last_key) {
			last_key = std::make_shared<Function>(Function(delimiter, {last_key, next_key}));
		} else {
			last_key = next_key;
		}
		res.push_back(token);
	}
	if (!last_key) {
		last_key = Atom::Tabled(key);
	}

	return last_key;
}
