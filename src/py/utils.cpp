/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include "knowrob/py/utils.h"
#include "knowrob/URI.h"

namespace knowrob::py {
	std::string resolveModulePath(std::string_view modulePath) {
		// TODO: also support modulePath that use "." as delimiter as this is the
		//  common way to address python modules. only std::filesystem::path is used at the moment.
		return URI::resolve(modulePath);
	}
}
