//
// Created by daniel on 31.08.23.
//

#ifndef KNOWROB_EDB_STAGE_H
#define KNOWROB_EDB_STAGE_H

#include <memory>
#include "knowrob/semweb/KnowledgeGraph.h"
#include "QueryStage.h"

namespace knowrob {

    class EDBStage : public QueryStage {
    public:
        EDBStage(KnowledgeGraphPtr edb,
                 const RDFLiteralPtr &literal,
                 int queryFlags);

    protected:
        KnowledgeGraphPtr edb_;

        AnswerBufferPtr submitQuery(const RDFLiteralPtr &literal) override;
    };

} // knowrob

#endif //KNOWROB_EDB_STAGE_H
