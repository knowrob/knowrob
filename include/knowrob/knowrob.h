/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_H_
#define KNOWROB_H_

#include <cstdlib>
#include <sstream>
#include <string>

namespace knowrob {
	/**
	 * Static initialization of the knowledge base.
	 * Note that it is important that argv[0] holds the name
	 * of the executable.
	 * @param argc number of arguments in argv.
	 * @param argv array of program arguments, argv[0] is the name of the binary.
	 */
	void InitKnowledgeBase(int argc, char **argv);

	/**
	 * Shutdown the knowledge base.
	 * This will ensure that all worker threads in the global DefaultThreadPool
	 * are joined.
	 * It is best to call this before exiting an application to avoid
	 * shutdown-crashes due to static resources associated with the main thread
	 * being destroyed before the worker threads (this is the case for spdlog).
	 */
	void ShutdownKnowledgeBase();

	/**
	 * @return the name of the executable in which the knowledge base is running.
	 */
	char *getNameOfExecutable();

	/**
	 * Combine a hash value with another value.
	 * @param seed a hash value.
	 * @param v a value to combine with the seed.
	 */
	void hashCombine(std::size_t &seed, const std::size_t &v);

	/**
	 * Read a string representation of an object using the << operator.
	 * @tparam T the type of the object.
	 * @param obj the object to read.
	 * @return a string representation of the object.
	 */
	template<typename T>
	std::string readString(const T &obj) {
		std::ostringstream oss;
		oss << obj;
		return oss.str();
	}

	/**
	 * Global settings for the knowledge base.
	 */
	class GlobalSettings {
	public:
		/**
		 * @return the batch size.
		 */
		static uint32_t batchSize() { return batchSize_; }

		/**
		 * The size of the container used when triples are processed in batches.
		 * @param batchSize the batch size for the backend.
		 */
		static void setBatchSize(uint32_t batchSize) { batchSize_ = batchSize; }

	protected:
		static uint32_t batchSize_;
	};
}

#endif //KNOWROB_H_
