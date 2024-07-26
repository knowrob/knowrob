/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_PY_CONVERTER_H
#define KNOWROB_PY_CONVERTER_H

#include <boost/python.hpp>
#include "memory"

// This header contains converter classes for the mapping of C++ classes to Python
// including converters for shared_ptr, string_view, vector, and optional.
// Widely based on code pieces found on the Internet.

#include "converter/shared_ptr.h"
#include "converter/string_view.h"
#include "converter/vector.h"
#include "converter/optional.h"
#include "converter/pair.h"

#endif //KNOWROB_PY_CONVERTER_H
