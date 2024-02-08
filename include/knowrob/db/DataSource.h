/*
 * Copyright (c) 2022, Daniel Beßler
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_DATA_SOURCES_H_
#define KNOWROB_DATA_SOURCES_H_

#include <boost/property_tree/ptree.hpp>
#include <string>
#include <memory>

namespace knowrob {
	class DataSource {
    public:
        static const std::string RDF_XML_FORMAT;
        static const std::string RDF_TURTLE_FORMAT;
        static const std::string PROLOG_FORMAT;

        explicit DataSource(std::string dataFormat);

        explicit DataSource(std::string dataFormat, std::string path);

        /**
         * Load data source settings from property tree.
         * @param property_tree
         */
        virtual void loadSettings(const boost::property_tree::ptree &property_tree);

        /**
         * @return string identifier of the data format.
         */
        const std::string& dataFormat() const {  return dataFormat_; }

        /**
         * @return URI of this data source.
         */
        const std::string& uri() const {  return uri_; }

        /**
         * @return string identifier of the data format.
         */
        const std::string& path() const {  return path_; }

    protected:
        std::string dataFormat_;
        boost::optional<std::string> protocol_;
        std::string path_;
        boost::optional<std::string> host_;
        boost::optional<int> port_;
        std::string uri_;

        void updateURI();
	};
    // aliases
	using DataSourcePtr = std::shared_ptr<DataSource>;
    using DataSourceLoader = std::function<bool(const DataSourcePtr&)>;
}

#endif //KNOWROB_DATA_SOURCES_H_
