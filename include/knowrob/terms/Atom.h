/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_ATOM_H
#define KNOWROB_ATOM_H

#include <unordered_map>
#include <optional>
#include <map>
#include "Atomic.h"

namespace knowrob {
	/**
	 * The type of an atom.
	 */
	enum class AtomType {
		/** a regular atom */
		REGULAR,
		/** an IRI node */
		IRI
	};

	/**
	 * An atom is an atomic term that represents a constant.
	 */
	class Atom : public Atomic {
	public:
		/**
		 * @param stringForm the string form of the atom
		 * @return a shared pointer to an atom
		 */
		static std::shared_ptr<knowrob::Atom> Tabled(std::string_view stringForm);

		/**
		 * Constructs an atom from a string.
		 * @param stringForm the string form of the atom
		 */
		explicit Atom(std::string_view stringForm);

		/**
		 * @param other another atom
		 * @return true if both atoms are equal
		 */
		bool isSameAtom(const Atom &other) const;

		/**
		 * @return the type of this atom.
		 */
		virtual AtomType atomType() const { return AtomType::REGULAR; }

		// override Atomic
		AtomicType atomicType() const final { return AtomicType::ATOM; }

		// override Atomic
		std::string_view stringForm() const final { return stringForm_; }

	protected:
		using AtomTable = std::map<std::string, std::optional<std::weak_ptr<Atom>>, std::less<>>;
		static AtomTable table_;

		std::string_view stringForm_;

		// override Term
		void write(std::ostream &os) const override;
	};

	using AtomPtr = std::shared_ptr<Atom>;

} // knowrob

#endif //KNOWROB_ATOM_H
