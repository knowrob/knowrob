/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include "knowrob/PropertyTree.h"
#include "knowrob/terms/Function.h"
#include "knowrob/py/utils.h"
#include "knowrob/terms/String.h"
#include "knowrob/terms/ListTerm.h"

using namespace knowrob;

PropertyTree::PropertyTree()
		: ptree_(nullptr),
		  delimiter_(".") {}

PropertyTree::PropertyTree(const boost::property_tree::ptree *ptree)
		: ptree_(ptree) {
	static const std::string formatDefault = {};

	// load all key-value pairs into settings map
	for (const auto &key_val: *ptree) {
		loadProperty(key_val.first, key_val.second);
	}

	// process list of data sources that should be imported into the reasoner backend.
	auto data_sources = ptree->get_child_optional("imports");
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

void PropertyTree::loadProperty( //NOLINT(misc-no-recursion)
		const std::string &key, const boost::property_tree::ptree &remainder) {
	if (remainder.empty()) {
		auto stringVal = remainder.get_value<std::string>();
		properties_.emplace(key, std::make_shared<String>(stringVal));
	} else {
		for (const auto &key_val: remainder) {
			if (key_val.first.empty()) {
				// this indicates a list. Elements of lists must be atomic values.
				auto &listData = key_val.second;
				std::vector<TermPtr> elements(listData.size());
				int index = 0;
				for(const auto& l_key_val : listData) {
					elements[index++] = std::make_shared<String>(l_key_val.second.get_value<std::string>());
				}
				properties_.emplace(key, std::make_shared<ListTerm>(elements));
			} else {
				auto key_t = key + delimiter_ + std::string(key_val.first);
				loadProperty(key_t, key_val.second);
			}
		}
	}
}

TermPtr PropertyTree::get(std::string_view key, const TermPtr &defaultValue) const {
	auto it = properties_.find(key.data());
	if (it != properties_.end()) {
		return it->second;
	} else {
		return defaultValue;
	}
}

TermPtr PropertyTree::createKeyTerm(std::string_view key) const {
	TermPtr last_key, next_key;

	size_t pos_start = 0, pos_end, delim_len = delimiter_.length();
	std::string_view token;
	std::vector<std::string_view> res;

	while ((pos_end = key.find(delimiter_, pos_start)) != std::string::npos) {
		token = key.substr(pos_start, pos_end - pos_start);
		pos_start = pos_end + delim_len;
		next_key = Atom::Tabled(token);
		if (last_key) {
			last_key = std::make_shared<Function>(Function(delimiter_, {last_key, next_key}));
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

namespace knowrob::py {
	template<>
	void createType<PropertyTree>() {
		using namespace boost::python;
		class_<PropertyTree, std::shared_ptr<PropertyTree>>("PropertyTree", init<>())
				.def("__iter__", range(&PropertyTree::begin, &PropertyTree::end))
				.def("get", &PropertyTree::get)
				.def("dataSources", &PropertyTree::dataSources, return_value_policy<copy_const_reference>());
	}
}
