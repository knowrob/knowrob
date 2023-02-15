/*
 * Copyright (c) 2022, Daniel Beßler
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_OPTION_LIST_H_
#define KNOWROB_OPTION_LIST_H_

#include <map>
#include <string>
#include "Term.h"

namespace knowrob {
	/**
	 * A list of options, where each option is represented as a term.
	 */
	class OptionList {
	public:
		/**
		 * Constructs an option list from a term.
		 * The term may be a list of options, or a single option value.
		 * Option terms have either the form `Key = Value` or `Key(Value)`.
		 * @param t a term from which options are read.
		 */
		explicit OptionList(const TermPtr &t);

		/**
		 * @return the option map.
		 */
		const std::map<std::string, TermPtr>& options() const { return options_; }

		/**
		 * @param key key of option.
		 * @return true is this list isMoreGeneralThan the key.
		 */
		bool contains(const std::string &key) const;

		/**
		 * @param key an option key
		 * @param defaultValue a default value
		 * @return the option value, or the default value
		 */
		const TermPtr& get(const std::string &key, const TermPtr &defaultValue) const;

		/**
		 * Read option value as a string.
		 * @param key an option key
		 * @param defaultValue a default value
		 * @return the option value, or the default value
		 */
		const std::string& getString(const std::string &key, const std::string &defaultValue) const;

		/**
		 * Read option value as a long.
		 * @param key an option key
		 * @param defaultValue a default value
		 * @return the option value, or the default value
		 */
		long getLong(const std::string &key, long defaultValue) const;

	protected:
		std::map<std::string, TermPtr> options_;

		void readOption(const TermPtr &option);
	};
}

#endif //KNOWROB_OPTION_LIST_H_
