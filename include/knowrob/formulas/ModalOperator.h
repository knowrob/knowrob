/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_MODAL_OPERATOR_H
#define KNOWROB_MODAL_OPERATOR_H

#include <list>
#include <map>
#include <optional>
#include "knowrob/terms/Term.h"
#include "knowrob/triples/Perspective.h"
#include "knowrob/TimeInterval.h"

namespace knowrob {
	/**
	 * The type of a modal operator.
	 */
	enum class ModalType {
		KNOWLEDGE = 1,
		BELIEF,
        ALWAYS,
        SOMETIMES
	};

	/**
	 * An operator of a modal language, e.g. "B" is often used for "belief" and "K" for "knowledge".
	 * Additional parameters can be added to the operator, e.g. "B[confidence=0.8]".
	 */
	class ModalOperator {
	public:
		static constexpr const char* KEY_PERSPECTIVE = "agent";
		static constexpr const char* KEY_CONFIDENCE = "confidence";
		static constexpr const char* KEY_BEGIN = "begin";
		static constexpr const char* KEY_END = "end";

		/**
		 * @param modalType the type of this operator.
		 */
		explicit ModalOperator(ModalType modalType);

		/**
		 * @param other another operator.
		 * @return true if this and other are the same operators.
		 */
		bool operator==(const ModalOperator &other) const;

		/**
		 * @return true if this operator is a necessity operator.
		 */
		bool isModalNecessity() const;

		/**
		 * @return true if this operator is a possibility operator.
		 */
		bool isModalPossibility() const;

		/**
		 * @return the type of this operator.
		 */
		auto modalType() const { return modalType_; }

		/**
		 * @return the symbol of this modal operator.
		 */
		const char *symbol() const;

		/**
		 * @param key the key of the parameter.
		 * @param value the value of the parameter.
		 */
		void setParameter(std::string_view key, const TermPtr &value);

		/**
		 * @param key the key of the parameter.
		 * @return the value of the parameter if any or else null-opt.
		 */
		std::optional<TermPtr> parameter(std::string_view key) const;

		/**
		 * @return the perspective parameter of this operator if any.
		 */
		std::optional<PerspectivePtr> perspective() const;

		/**
		 * @param agent the perspective parameter of this operator.
		 */
		void setPerspective(const std::string_view &agent);

		/**
		 * @return the confidence parameter of this operator if any.
		 */
        std::optional<double> confidence() const;

        /**
		 * @param confidence the confidence parameter of this operator.
		 */
        void setConfidence(double confidence);

        /**
         * @return the time interval begin parameter of this operator if any.
		 */
        std::optional<double> begin() const;

        /**
         * @param begin the time interval begin parameter of this operator.
		 */
        void setBegin(double begin);

        /**
		 * @return the time interval end parameter of this operator if any.
         */
        std::optional<double> end() const;

        /**
		 * @param end the time interval end parameter of this operator.
		 */
		void setEnd(double end);

		/**
		 * @return the time interval parameter of this operator if any.
		 */
		void setTimeInterval(const TimeInterval &timeInterval);

		/**
		 * @param os the output stream.
		 */
		void write(std::ostream &os) const;

	protected:
		const ModalType modalType_;
		std::map<std::string, TermPtr> parameters_;
	};

	using ModalOperatorPtr = std::shared_ptr<const ModalOperator>;

	/**
	 * An iteration over modalities.
	 * Each iteration corresponds to applying the accessibility relation once.
	 */
	class ModalIteration {
	public:
		ModalIteration() = default;

		/**
		 * @param other another iteration.
		 * @return true if this and other are the same iterations.
		 */
		bool operator==(const ModalIteration &other) const;

		/**
		 * @param modalOperator add an operator to this iteration.
		 */
		void operator+=(const ModalOperatorPtr &modalOperator);

		ModalIteration operator+(const ModalOperatorPtr &modalOperator) const;

		/**
		 * @return number of operators in this sequence.
		 */
		auto numOperators() const { return modalitySequence_.size(); }

		/**
		 * @return begin iterator of modal operators.
		 */
		auto begin() const { return modalitySequence_.begin(); }

		/**
		 * @return end iterator of modal operators.
		 */
		auto end() const { return modalitySequence_.end(); }

		/**
		 * @return an empty iteration.
		 */
		static const ModalIteration &emptyIteration();

	protected:
		std::list<ModalOperatorPtr> modalitySequence_;
	};

} // knowrob

#endif //KNOWROB_MODAL_OPERATOR_H
