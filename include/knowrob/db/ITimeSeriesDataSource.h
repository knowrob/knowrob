/*
 * Copyright (c) 2022, Daniel Beßler
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef __KNOWROB_TIME_SERIES_DATA_SOURCE_H__
#define __KNOWROB_TIME_SERIES_DATA_SOURCE_H__

#include "knowrob/db/IDataSource.h"

namespace knowrob {
    /**
     * A collection of time-series data.
     */
    class ITimeSeriesDataSource : public IDataSource {
    public:
        virtual ~ITimeSeriesDataSource(){}

    protected:
    };
}

#endif //__KNOWROB_TIME_SERIES_DATA_SOURCE_H__
