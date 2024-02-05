/*
 * Copyright (c) 2024, Sascha Jongebloed
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include "knowrob/interfaces/RestInterface.h"

namespace knowrob {

	RestInterface::RestInterface(const boost::property_tree::ptree &ptree) {

	}



	// Function to parse JSON to GraphQuery
	GraphQuery parseGraphQuery(std::string json_str) {
		try {
			// Parse the JSON string into a json object
			json j = json::parse(json_str);

			// Create a GraphQueryMessage instance
			GraphQueryFields query;

			// Assign values from the json object to the fields of GraphQueryMessage
			// Use the get method for type-safe extraction
			query.queryString = j["queryString"].get<std::string>();
			query.epistemicOperator = j["epistemicOperator"].get<uint8_t>();
			query.aboutAgentIRI = j["aboutAgentIRI"].get<std::string>();
			query.aboutSimulationIRI = j["aboutSimulationIRI"].get<std::string>();
			query.temporalOperator = j["temporalOperator"].get<uint8_t>();
			query.minPastTimestamp = j["minPastTimestamp"].get<double>();
			query.maxPastTimestamp = j["maxPastTimestamp"].get<double>();
			query.confidence = j["confidence"].get<double>();

			return query;

		} catch (const json::parse_error &e) {
			// TODO Handle parsing errors (e.g., invalid JSON format)
			std::cerr << e.what() << std::endl;
		} catch (const json::type_error &e) {
			// TODO: Handle type errors (e.g., accessing a string as an int)
			std::cerr << e.what() << std::endl;
		}
	}

	std::string serializeGraphAnswerMessage(const std::vector<GraphAnswerFields>& messages) {
		nlohmann::json j;

		nlohmann::json j_array = nlohmann::json::array();
		for (const auto& message : messages) {
			nlohmann::json j_message;
			nlohmann::json j_substitution = nlohmann::json::array();

			for (const auto& kv : message.substitution) {
				j_substitution.push_back({{"key", kv.key}, {"value", kv.value}});
			}

			j_message["substitution"] = j_substitution;
			j_message["instantiation"] = message.instantiation;
			j_message["confidence"] = message.confidence;
			j_message["startTimestamp"] = message.startTimestamp;
			j_message["endTimestamp"] = message.endTimestamp;

			j_array.push_back(j_message);
		}

		j["answers"] = j_array;
		if (messages.empty()) {
			j["status"] = "NO_SOLUTIONS";
		} else {
			j["status"] = "OK";
		}
		j["status"] = status;

		return j.dump();  // Convert the JSON object to a string
	}

	GraphAnswerFields processAskAllQuery(GraphQuery query);

	void handle_request(boost::beast::http::request<boost::beast::http::string_body> req,
						boost::beast::http::response<boost::beast::http::string_body> &res) {
		// Check if the request is a POST request to the '/query' endpoint
		if (req.method() != boost::beast::http::verb::post || req.target() != "/query") {
			// Respond with 404 Not Found if the endpoint or method is not correct
			res = boost::beast::http::response<boost::beast::http::string_body>{
					boost::beast::http::status::not_found, req.version()};
			res.set(boost::beast::http::field::server, BOOST_BEAST_VERSION_STRING);
			res.set(boost::beast::http::field::content_type, "text/plain");
			res.keep_alive(req.keep_alive());
			res.body() = "Endpoint not found";
			res.prepare_payload();
			return;
		}

		try {
			if (req.target() == "/askAll") {
				// Parse the request body into a GraphQueryMessage
				GraphQuery query = parseGraphQuery(req.body());

				// Process the query and obtain an answer (implement this according to your application logic)
				GraphAnswerFields answer = processAskAllQuery(query); // Placeholder for query processing

				// Serialize the answer into a JSON string
				std::string response_body = serializeGraphAnswerMessage(answer);

				// Set up the response
				res.result(boost::beast::http::status::ok);
				res.set(boost::beast::http::field::server, BOOST_BEAST_VERSION_STRING);
				res.set(boost::beast::http::field::content_type, "application/json");
				res.body() = response_body;
				res.prepare_payload();
			} else if (req.target() == "/askOnce") {
				// Handle /askOnce endpoint
				// [Implement your logic for /askOnce endpoint here]
			} else {
				// Respond with 404 Not Found for unknown endpoints
				res = {boost::beast::http::status::not_found, req.version()};
				res.set(boost::beast::http::field::content_type, "text/plain");
				res.keep_alive(req.keep_alive());
				res.body() = "Not Found";
				res.prepare_payload();
			}
		} catch (const std::exception &e) {
			// Handle any exceptions and respond with 500 Internal Server Error
			res = boost::beast::http::response<boost::beast::http::string_body>{
					boost::beast::http::status::internal_server_error, req.version()};
			res.set(boost::beast::http::field::server, BOOST_BEAST_VERSION_STRING);
			res.set(boost::beast::http::field::content_type, "text/plain");
			res.body() = std::string("Internal Server Error: ") + e.what();
			res.prepare_payload();
		}
	}

	int main() {
		try {
			// The io_context is required for all I/O
			boost::asio::io_context ioc;

			// The acceptor receives incoming connections
			boost::asio::ip::tcp::acceptor acceptor(ioc, {boost::asio::ip::tcp::v4(), 8080});

			while (true) {
				// This will receive the new connection
				boost::asio::ip::tcp::socket socket(ioc);

				// Block until we get a connection
				acceptor.accept(socket);

				// This buffer is used for reading and must be persisted
				boost::beast::flat_buffer buffer;

				// This will hold the incoming request
				boost::beast::http::request<boost::beast::http::string_body> req;

				// Read a request
				boost::beast::http::read(socket, buffer, req);

				// Handle the request
				boost::beast::http::response<boost::beast::http::string_body> res;
				handle_request(std::move(req), res);

				// Send the response
				boost::beast::http::write(socket, res);

				// Gracefully close the socket
				boost::beast::error_code ec;
				socket.shutdown(boost::asio::ip::tcp::socket::shutdown_send, ec);

				// not_connected happens sometimes so don't bother reporting it.
				if (ec && ec != boost::beast::errc::not_connected) {
					throw boost::beast::system_error{ec};
				}
			}
		} catch (const std::exception& e) {
			std::cerr << "Error in REST Server: " << e.what() << std::endl;
			return EXIT_FAILURE;
		}

		return EXIT_SUCCESS;
	}
}