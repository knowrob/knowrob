/*
 * Copyright (c) 2022, Daniel Beßler
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_BOTTOM_H_
#define KNOWROB_BOTTOM_H_

#include <ostream>
#include <memory>
#include "Predicate.h"

namespace knowrob {
	/**
	 * A predicate with a fixed truth value being `false`.
	 */
	class BottomTerm : public Predicate {
	public:
		static const std::shared_ptr<BottomTerm>& get();

		// Override Term
		void write(std::ostream& os) const override;

	private:
		BottomTerm();
	};
}

#endif //KNOWROB_BOTTOM_H_
