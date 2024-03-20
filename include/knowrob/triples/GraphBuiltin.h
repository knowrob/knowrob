/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_GRAPH_BUILTIN_H
#define KNOWROB_GRAPH_BUILTIN_H

#include <utility>

#include "memory"
#include "knowrob/triples/GraphTerm.h"
#include "knowrob/terms/Variable.h"
#include "knowrob/terms/Atomic.h"
#include "knowrob/terms/Function.h"

namespace knowrob {
	/**
	 * The type of a builtin.
	 */
	enum class GraphBuiltinType {
		Bind, // Bind a value to a variable in the query pipeline.
		Min, // Calculate the minimum of two terms.
		Max, // Calculate the maximum of two terms.
		Less, // Check if the first term is less than the second term.
		LessOrEqual, // Check if the first term is less than or equal to the second term.
		Greater, // Check if the first term is greater than the second term.
		GreaterOrEqual, // Check if the first term is greater than or equal to the second term.
		Equal // Check if the first term is equal to the second term.
	};

	namespace graph::builtins {
		/**
		 * The functor name of the "bind" builtin.
		 */
		const AtomPtr bindFunctor = Atom::Tabled("bind");

		/**
		 * The functor name of the "min" builtin.
		 */
		const AtomPtr minFunctor = Atom::Tabled("min");

		/**
		 * The functor name of the "max" builtin.
		 */
		const AtomPtr maxFunctor = Atom::Tabled("max");

		/**
		 * The functor name of the "less" builtin.
		 */
		const AtomPtr lessFunctor = Atom::Tabled("less");

		/**
		 * The functor name of the "lessOrEqual" builtin.
		 */
		const AtomPtr lessOrEqualFunctor = Atom::Tabled("lessOrEqual");

		/**
		 * The functor name of the "greater" builtin.
		 */
		const AtomPtr greaterFunctor = Atom::Tabled("greater");

		/**
		 * The functor name of the "greaterOrEqual" builtin.
		 */
		const AtomPtr greaterOrEqualFunctor = Atom::Tabled("greaterOrEqual");

		/**
		 * The functor name of the "equal" builtin.
		 */
		const AtomPtr equalFunctor = Atom::Tabled("equal");
	}

	/**
	 * A builtin term that is part of a graph query.
	 */
	class GraphBuiltin : public GraphTerm, public Function {
	public:
		using BuiltinPtr = std::shared_ptr<GraphBuiltin>;

		/**
		 * Constructs a builtin term from a builtin type, a functor, a list of arguments, and an optional variable to bind.
		 * @param builtinType the builtin type.
		 * @param functor the functor.
		 * @param arguments the arguments.
		 * @param bindVar the variable to bind or null if no binding is used by the builtin.
		 */
		GraphBuiltin(GraphBuiltinType builtinType,
					 const AtomPtr &functor,
					 const std::vector<TermPtr> &arguments,
					 VariablePtr bindVar = nullptr)
				: GraphTerm(GraphTermType::Builtin),
				  Function(functor, arguments),
				  builtinType_(builtinType),
				  bindVar_(std::move(bindVar)),
				  isOptional_(true) {}

		/**
		 * @return the builtin type.
		 */
		auto builtinType() const { return builtinType_; }

		/**
		 * @return the variable to bind or null if no binding is used by the builtin.
		 */
		auto bindVar() const { return bindVar_; }

		/**
		 * Bind a value to a variable in the query pipeline.
		 * @param var the variable to bind.
		 * @param val the value to bind to the variable.
		 * @return a new Builtin that binds the value to the variable.
		 */
		static BuiltinPtr bind(const VariablePtr &var, const TermPtr &val) {
			return std::make_shared<GraphBuiltin>(
					GraphBuiltinType::Bind,
					graph::builtins::bindFunctor,
					std::vector<TermPtr>{val},
					var);
		}

		/**
		 * Create a new Builtin that calculates the minimum of two terms.
		 * @param var the variable to bind the result to.
		 * @param a the first term.
		 * @param b the second term.
		 * @return a new Builtin that calculates the minimum of two terms.
		 */
		static BuiltinPtr min(const VariablePtr &var, const TermPtr &a, const TermPtr &b) {
			return std::make_shared<GraphBuiltin>(
					GraphBuiltinType::Min,
					graph::builtins::minFunctor,
					std::vector<TermPtr>{a, b},
					var);
		}

		/**
		 * Create a new Builtin that calculates the maximum of two terms.
		 * @param var the variable to bind the result to.
		 * @param a the first term.
		 * @param b the second term.
		 * @return a new Builtin that calculates the maximum of two terms.
		 */
		static BuiltinPtr max(const VariablePtr &var, const TermPtr &a, const TermPtr &b) {
			return std::make_shared<GraphBuiltin>(
					GraphBuiltinType::Max,
					graph::builtins::maxFunctor,
					std::vector<TermPtr>{a, b},
					var);
		}

		/**
		 * Create a new Builtin that checks if the first term is less than the second term.
		 * @param a the first term.
		 * @param b the second term.
		 * @return a new Builtin that checks if the first term is less than the second term.
		 */
		static BuiltinPtr less(const TermPtr &a, const TermPtr &b) {
			return std::make_shared<GraphBuiltin>(
					GraphBuiltinType::Less,
					graph::builtins::lessFunctor,
					std::vector<TermPtr>{a, b});
		}

		/**
		 * Create a new Builtin that checks if the first term is less than or equal to the second term.
		 * @param a the first term.
		 * @param b the second term.
		 * @return a new Builtin that checks if the first term is less than or equal to the second term.
		 */
		static BuiltinPtr lessOrEqual(const TermPtr &a, const TermPtr &b) {
			return std::make_shared<GraphBuiltin>(
					GraphBuiltinType::LessOrEqual,
					graph::builtins::lessOrEqualFunctor,
					std::vector<TermPtr>{a, b});
		}

		/**
		 * Create a new Builtin that checks if the first term is greater than the second term.
		 * @param a the first term.
		 * @param b the second term.
		 * @return a new Builtin that checks if the first term is greater than the second term.
		 */
		static BuiltinPtr greater(const TermPtr &a, const TermPtr &b) {
			return std::make_shared<GraphBuiltin>(
					GraphBuiltinType::Greater,
					graph::builtins::greaterFunctor,
					std::vector<TermPtr>{a, b});
		}

		/**
		 * Create a new Builtin that checks if the first term is greater than or equal to the second term.
		 * @param a the first term.
		 * @param b the second term.
		 * @return a new Builtin that checks if the first term is greater than or equal to the second term.
		 */
		static BuiltinPtr greaterOrEqual(const TermPtr &a, const TermPtr &b) {
			return std::make_shared<GraphBuiltin>(
					GraphBuiltinType::GreaterOrEqual,
					graph::builtins::greaterOrEqualFunctor,
					std::vector<TermPtr>{a, b});
		}

		/**
		 * Create a new Builtin that checks if the first term is equal to the second term.
		 * @param a the first term.
		 * @param b the second term.
		 * @return a new Builtin that checks if the first term is equal to the second term.
		 */
		static BuiltinPtr equal(const TermPtr &a, const TermPtr &b) {
			return std::make_shared<GraphBuiltin>(
					GraphBuiltinType::Equal,
					graph::builtins::equalFunctor,
					std::vector<TermPtr>{a, b});
		}

		/**
		 * @return true if the builtin is optional, false otherwise.
		 */
		bool isOptional() const { return isOptional_; }

		/**
		 * Set the optional flag of the builtin.
		 * @param isOptional the optional flag.
		 */
		void setOptional(bool isOptional) { isOptional_ = isOptional; }

		// Overwritten from GraphTerm
		void write(std::ostream &os) const override { Function::write(os); }

	protected:
		GraphBuiltinType builtinType_;
		VariablePtr bindVar_;
		bool isOptional_;
	};
} // knowrob

#endif //KNOWROB_GRAPH_BUILTIN_H
