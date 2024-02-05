/*
 * Copyright (c) 2022, Daniel Beßler
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include <vector>
#include <boost/algorithm/string.hpp>

#include "knowrob/semweb/PrefixRegistry.h"
#include "knowrob/terms/Constant.h"
#include "knowrob/formulas/Predicate.h"

using namespace knowrob;

PredicatePtr StringTerm::operator()(TermPtr arg1, TermPtr arg2) const
{
	return std::make_shared<Predicate>(value_, std::vector<TermPtr>{arg1, arg2});
}

void StringTerm::write(std::ostream& os) const
{
    // print IRI's in short form
    // TODO: rather have something similar to Prolog's portray for pretty printing
    if(value_.rfind("http", 0) == 0) {
        std::vector<std::string> urlAndFragment;
        boost::split(urlAndFragment, value_, boost::is_any_of("#"));

        if(urlAndFragment.size() == 2 && !urlAndFragment[1].empty()) {
            auto alias = semweb::PrefixRegistry::get().uriToAlias(urlAndFragment[0]);
            if(alias) {
                write1(os, alias.value());
                os << ':';
                write1(os, urlAndFragment[1]);
                return;
            }
        }
    }

    write1(os, value_);
}

void StringTerm::write1(std::ostream& os, const std::string &str)
{
    if(std::islower(str[0])) {
        // avoid single quotes
        os << str;
    }
    else {
        os << '\'' << str << '\'';
    }
}
