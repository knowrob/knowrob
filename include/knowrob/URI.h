/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_URI_H
#define KNOWROB_URI_H

#include <string>
#include <boost/property_tree/ptree.hpp>

namespace knowrob {
	class URI {
	public:
		explicit URI(std::string_view path);

		explicit URI(const boost::property_tree::ptree &property_tree);

		URI(std::string path, std::string protocol, std::string host, int port);

		static std::string resolve(const std::string_view &uriString);

		/**
		 * @return URI string of this data source.
		 */
		const auto &operator()() const { return uri_; }

		/**
		 * @return string identifier of the data format.
		 */
		const std::string &path() const { return path_; }

	protected:
		std::string uri_;
		std::string path_;
		boost::optional<std::string> protocol_;
		boost::optional<std::string> host_;
		boost::optional<int> port_;

		void updateURI();
	};
}

#endif //KNOWROB_URI_H
