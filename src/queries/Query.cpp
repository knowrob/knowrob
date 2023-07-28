//
// Created by daniel on 28.07.23.
//

#include "knowrob/queries/Query.h"

namespace std {
	std::ostream& operator<<(std::ostream& os, const knowrob::Query& q) //NOLINT
	{
		return q.print(os);
	}
}
