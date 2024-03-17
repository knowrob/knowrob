/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_PARSERS_COMMON_H
#define KNOWROB_PARSERS_COMMON_H

#include "memory"
#include <boost/spirit/include/phoenix.hpp>

namespace knowrob::parsers {
	// code-snippet found on the web, needed to create smart pointers in parser rules.
	// effectively here `ptr_` is defined and can be used to create std::shared_ptr's
	// in parser rules.
	template<typename T>
	struct make_shared_f {
		template<typename... A>
		struct result {
			typedef std::shared_ptr<T> type;
		};

		template<typename... A>
		typename result<A...>::type operator()(A &&... a) const {
			return std::make_shared<T>(std::forward<A>(a)...);
		}
	};

	template<typename T> using ptr_ = boost::phoenix::function<make_shared_f<T> >;
}

#endif //KNOWROB_PARSERS_COMMON_H
