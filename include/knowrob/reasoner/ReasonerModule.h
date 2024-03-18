/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_REASONER_MODULE_H_
#define KNOWROB_REASONER_MODULE_H_

#include <string>
#include <memory>
#include <boost/python.hpp>
#include "knowrob/reasoner/ReasonerFactory.h"

namespace knowrob {
	/**
	 * A reasoner module is a reasoner factory that creates reasoner objects
	 * that are implemented in Python code.
	 */
	class ReasonerModule : public ReasonerFactory {
	public:
		/**
		 * @param modulePath the name or path of the Python module.
		 * @param reasonerType the name of the reasoner type.
		 */
		ReasonerModule(std::string_view modulePath, std::string_view reasonerType);

		~ReasonerModule() override;

		/**
		 * Cannot be copy-assigned.
		 */
		ReasonerModule(const ReasonerModule &) = delete;

		/**
		 * @return true if the module was loaded successfully.
		 */
		bool isLoaded();

		/**
		 * Try loading the module from filesystem.
		 * @return true on success.
		 */
		bool loadModule();

		// Override ReasonerFactory
		std::shared_ptr<Reasoner> createReasoner(std::string_view reasonerID) override;

		// Override ReasonerFactory
		std::string_view name() const override { return name_; };

	protected:
		const std::string modulePath_;
		const std::string reasonerType_;
		boost::python::object pyModule_;
		boost::python::object pyReasonerType_;
		std::string name_;
		std::set<std::filesystem::path> moduleDirectories_;

		static std::string resolveModulePath(std::string_view modulePath);
	};
}

#endif //KNOWROB_REASONER_MODULE_H_
