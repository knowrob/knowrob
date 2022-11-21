/*
 * Copyright (c) 2022, Daniel Beßler
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef __KNOWROB_DATA_FILE_H__
#define __KNOWROB_DATA_FILE_H__

#include <string>

#include "knowrob/db/IDataSource.h"

namespace knowrob {
    /**
     * A data file of some format that can be opened to read the data using an apropiate parser.
     */
    class IDataFile : public IDataSource {
    public:
        IDataFile(const std::string &fileURL);
        ~IDataFile();

        // TODO: provide an interface to open the file here?
        //   - some parsers may only support to load files from local filesystem
        //     IDataFile could download remote files and provide a local path for parsing
        //     them.

    protected:
    };
}

#endif //__KNOWROB_DATA_FILE_H__
