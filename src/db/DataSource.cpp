//
// Created by daniel on 10.02.23.
//

#include <utility>

#include "knowrob/db/DataSource.h"

using namespace knowrob;

const std::string DataSource::RDF_XML_FORMAT    = "rdf-xml";
const std::string DataSource::RDF_TURTLE_FORMAT = "turtle";
const std::string DataSource::PROLOG_FORMAT     = "prolog";

DataSource::DataSource(std::string dataFormat)
: dataFormat_(std::move(dataFormat))
{
}

DataSource::DataSource(std::string dataFormat, std::string path)
: dataFormat_(std::move(dataFormat)),
  path_(std::move(path))
{
    updateURI();
}

void DataSource::loadSettings(const boost::property_tree::ptree &property_tree) {
    protocol_ = property_tree.get_optional<std::string>("protocol");
    path_ = property_tree.get("path", "/");
    host_ = property_tree.get_optional<std::string>("host");
    port_ = property_tree.get_optional<int>("port");
    updateURI();
}

void DataSource::updateURI()
{
    std::stringstream ss;
    if(protocol_ && protocol_.value() != "file") {
        ss << protocol_ << "://";
    }
    if(host_) {
        ss << host_.value();
        if(port_) ss << ':' << port_.value();
    }
    ss << path_;
    uri_ = ss.str();
}
