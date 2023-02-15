/*
 * Copyright (c) 2022, Daniel Beßler
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include <sstream>
#include <knowrob/queries/QueryError.h>

using namespace knowrob;

QueryError::QueryError(
	const Query &erroneousQuery,
	const Term &errorTerm)
: std::runtime_error(formatErrorString(erroneousQuery, errorTerm))
{
}

std::string QueryError::formatErrorString(
	const Query &erroneousQuery,
	const Term &errorTerm)
{
	std::stringstream buffer;
	buffer << errorTerm;
	return buffer.str();
}
