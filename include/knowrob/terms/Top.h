/*
 * Copyright (c) 2022, Daniel Beßler
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_TOP_H_
#define KNOWROB_TOP_H_

#include <ostream>
#include <memory>
#include "Predicate.h"

namespace knowrob {
	/**
	 * A predicate with a fixed truth value being `true`.
	 */
	class TopTerm : public Predicate {
	public:
		static const std::shared_ptr<TopTerm>& get();

		// Override Term
		void write(std::ostream& os) const override;

	private:
		TopTerm();
	};
}

#endif //KNOWROB_TOP_H_
