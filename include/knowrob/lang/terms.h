/*
 * Copyright (c) 2022, Daniel Beßler
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef __KNOWROB_TERMS_H__
#define __KNOWROB_TERMS_H__

namespace knowrob {
	class Term {
	public:
		static boost::shared_ptr<Term> read(const std::string &termString);
		
		Term(){}
		
		/*std::list<Variable*> getVariables() {
			std::list<Variable*> l;
			getVariables(l);
			return l;
		}*/
	
	protected:
		//virtual void getVariables(std::list<Variable*> &list) = 0;
	};
	
	class Variable : public Term {
	public:
		Variable(const std::string &name) : name_(name) {}
		
		const std::string& name() { return name_; }
	protected:
		std::string name_;
		
		//virtual void getVariables(std::list<Variable*> &list) { list.add(this); }
	};
	
	template <typename T>
	class AtomicTerm : public Term {
	public:
		AtomicTerm(const T &value) : value_(value) {}
		
		const T& value() { return value_; }
	protected:
		T value_;
	};

	/**
	 * A predicate in the knowledge base defined by its functor and arity.
	 */
	class PredicateIndicator {
	public:
		PredicateIndicator(const std::string &functor, unsigned int arity)
		: functor_(functor), arity_(arity) {};
		
		/** Get the functor of this predicate.
		 *
		 * @return functor name
		 */
		const std::string& functor() const { return functor_; }
		
		/** Get the arity of this predicate.
		 *
		 * @return arity of predicate
		 */
		unsigned int arity() const { return arity_; }
	private:
		std::string functor_;
		unsigned int arity_;
	};
	
	class Predicate : public Term {
	public:
		Predicate(const std::string &functor, const std::vector<Term> &arguments)
		: inidcator_(functor, arguments.size()), arguments_(arguments)
		{}
		
		const PredicateIndicator& inidcator() { return inidcator_; }
		
		const std::list<boost::shared_ptr<Term>>& arguments() { return arguments_; }
	
	protected:
		PredicateIndicator inidcator_;
		std::vector<Term> arguments_;
	};
	
	/*
	class Triple : public Predicate {
	};
	
	class FuzzyPredicate : public Predicate {
	};
	
	class TemporalizedPredicate : public Predicate {
	};
	*/
}

#endif //__KNOWROB_TERMS_H__
