//
// Created by daniel on 28.07.23.
//

#include "knowrob/queries/Query.h"
#include "knowrob/queries/QueryEngine.h"

using namespace knowrob;

int Query::defaultFlags()
{ return (int)QueryFlag::QUERY_FLAG_ALL_SOLUTIONS; }

namespace std {
	std::ostream& operator<<(std::ostream& os, const knowrob::Query& q) //NOLINT
	{
		return q.print(os);
	}
}
