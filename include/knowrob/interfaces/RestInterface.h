/*
 * Copyright (c) 2024, Sascha Jongebloed
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

//
// Rest Interface
//

#ifndef KNOWROB_RESTINTERFACE_H
#define KNOWROB_RESTINTERFACE_H

#include "knowrob/interfaces/Interface.h"
// include boost beast libraries for rest
#include <boost/beast/core.hpp>
#include <boost/beast/http.hpp>
#include <boost/beast/version.hpp>
#include <boost/asio/bind_executor.hpp>
#include <boost/asio/ip/tcp.hpp>
#include <boost/asio/strand.hpp>
#include <nlohmann/json.hpp>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <string>
#include <thread>

using json = nlohmann::json;

namespace knowrob {
	class RestInterface {


		RestInterface(const boost::property_tree::ptree &ptree);
	};
}


#endif //KNOWROB_RESTINTERFACE_H
