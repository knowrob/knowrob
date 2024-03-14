/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_EPISTEMIC_MODALITY_H
#define KNOWROB_EPISTEMIC_MODALITY_H

#include "knowrob/Logger.h"
#include "Modality.h"
#include "knowrob/triples/Perspective.h"

#include <utility>

namespace knowrob {
	enum class EpistemicOperator : uint8_t {
		KNOWLEDGE = 1,
		BELIEF = 2
	};

	/**
	 * Epistemic is concerned with knowledge and belief.
	 */
	class EpistemicModality : public Modality {
	public:
		EpistemicModality();

		explicit EpistemicModality(const std::string_view &agent);

		const std::optional<PerspectivePtr> &agent() const;

		ModalityType modalityType() const override;

		const char *necessity_symbol() const override;

		const char *possibility_symbol() const override;

	protected:
		const std::optional<PerspectivePtr> agent_;
	};

} // knowrob

#endif //KNOWROB_EPISTEMIC_MODALITY_H
