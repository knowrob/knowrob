//
// Created by daniel on 21.03.23.
//

#ifndef KNOWROB_MODAL_LITERAL_H
#define KNOWROB_MODAL_LITERAL_H

#include <memory>
#include "knowrob/terms/NestedModality.h"
#include "knowrob/terms/Predicate.h"

namespace knowrob {
    /**
     * Either an atomic formula or the negation of an atomic formula.
     * The literal is labeled by a sequence of modal operators.
     */
    class ModalLiteral {
    public:
        ModalLiteral(const ModalitySequencePtr modalities, const PredicatePtr &predicate, bool isNegated)
        : modalities_(modalities), predicate_(predicate), isNegated_(isNegated) {}

        const ModalitySequencePtr& modalities() const { return modalities_; }

        const PredicatePtr& predicate() const { return predicate_; }

        bool isNegated() const { return isNegated_; }

    protected:
        const ModalitySequencePtr modalities_;
        const PredicatePtr predicate_;
        bool isNegated_;
    };

} // knowrob

#endif //KNOWROB_MODAL_LITERAL_H
