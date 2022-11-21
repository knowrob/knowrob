/*
 * Copyright (c) 2022, Daniel Beßler
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef __KNOWROB_DATA_SOURCE_H__
#define __KNOWROB_DATA_SOURCE_H__

namespace knowrob {
    /**
     * A source of data used to draw answers to queries.
     */
    class IDataSource {
    public:
        virtual ~IDataSource(){}

    protected:
    };
}

#endif //__KNOWROB_DATA_SOURCE_H__
