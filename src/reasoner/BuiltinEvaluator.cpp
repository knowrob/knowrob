/*
 * Copyright (c) 2023, Daniel Beßler
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include <boost/algorithm/string/predicate.hpp>
#include "knowrob/Logger.h"
#include "knowrob/reasoner/BuiltinEvaluator.h"
#include "knowrob/formulas/Predicate.h"
#include "knowrob/queries/QueryError.h"

using namespace knowrob;

#define BUILTIN_FUNCTION(func) [this]( \
	const QueryInstancePtr &queryInstance, \
	const std::vector<TermPtr> &args) { return func(queryInstance, args); }

BuiltinEvaluator::BuiltinEvaluator()
{
	// define some builtins
	builtins_[PredicateIndicator("atom_concat",3)] = BUILTIN_FUNCTION(atom_concat3);
}

const std::shared_ptr<BuiltinEvaluator>& BuiltinEvaluator::get()
{
	static auto singleton = std::shared_ptr<BuiltinEvaluator>(new BuiltinEvaluator());
	return singleton;
}

bool BuiltinEvaluator::loadConfiguration(const ReasonerConfiguration &cfg)
{
	return true;
}

unsigned long BuiltinEvaluator::getCapabilities() const
{
	return ReasonerCapability::CAPABILITY_NONE;
}

bool BuiltinEvaluator::isBuiltinSupported(const std::shared_ptr<PredicateIndicator> &indicator)
{
	return builtins_.find(*indicator) != builtins_.end();
}

std::shared_ptr<PredicateDescription> BuiltinEvaluator::getPredicateDescription(
		const std::shared_ptr<PredicateIndicator> &indicator)
{
	return {};
}

void BuiltinEvaluator::startQuery(uint32_t queryID,
								  const std::shared_ptr<const Query> &uninstantiatedQuery)
{}

void BuiltinEvaluator::finishQuery(uint32_t queryID,
								   const std::shared_ptr<AnswerStream::Channel> &outputStream,
								   bool isImmediateStopRequested)
{}

void BuiltinEvaluator::runQueryInstance(uint32_t queryID, const QueryInstancePtr &queryInstance)
{
	auto builtinQuery = queryInstance->create();
	auto *builtinPredicate = ((Predicate*)builtinQuery->formula().get());

	auto it = builtins_.find(*builtinPredicate->indicator());
	if(it == builtins_.end()) {
		KB_WARN("BuiltinEvaluator has no builtin {}/{} defined.",
				builtinPredicate->indicator()->functor(),
				builtinPredicate->indicator()->arity());
	}
	else {
		KB_DEBUG("BuiltinEvaluator has new query {}.", *builtinPredicate);
		it->second(queryInstance, builtinPredicate->arguments());
	}
}

void BuiltinEvaluator::pushSubstitution1(
		const QueryInstancePtr &queryInstance,
		Variable &var, const TermPtr& value)
{
	auto result = std::make_shared<Answer>();
	result->substitute(var,value);
	queryInstance->pushSolution(result);
}

void BuiltinEvaluator::atom_concat3(const QueryInstancePtr &queryInstance, const std::vector<TermPtr> &args)
{
	auto numVars =
			((int)(args[0]->type()==TermType::VARIABLE)) +
			((int)(args[1]->type()==TermType::VARIABLE)) +
			((int)(args[2]->type()==TermType::VARIABLE));
	auto numAtoms =
			((int)(args[0]->type()==TermType::STRING)) +
			((int)(args[1]->type()==TermType::STRING)) +
			((int)(args[2]->type()==TermType::STRING));
	if(numVars>1)
		throw QueryError("instantiation error");
	if(numAtoms<2)
		throw QueryError("type error");

	if(args[2]->type()==TermType::VARIABLE) {
		auto var3 = (Variable*)args[2].get();
		auto atom1 = ((StringTerm*)args[0].get());
		auto atom2 = ((StringTerm*)args[1].get());
		auto atom3 = std::make_shared<StringTerm>(atom1->value() + atom2->value());
		// return var3=atom3
		pushSubstitution1(queryInstance, *var3, atom3);
	}
	else if(numVars==0) {
		auto atom1 = ((StringTerm*)args[0].get());
		auto atom2 = ((StringTerm*)args[1].get());
		auto atom3 = ((StringTerm*)args[2].get());
		if(atom1->value() + atom2->value() == atom3->value()) {
			// return true
			queryInstance->pushSolution(std::make_shared<Answer>());
		}
	}
	else if(args[0]->type()==TermType::VARIABLE) {
		auto var1 = (Variable*)args[0].get();
		auto atom2 = ((StringTerm*)args[1].get());
		auto atom3 = ((StringTerm*)args[2].get());
		if(boost::algorithm::ends_with(atom3->value(), atom2->value())) {
			auto atom1 = std::make_shared<StringTerm>(
					atom3->value().substr(0, atom3->value().length()-atom2->value().length()));
			// return var1=atom1
			pushSubstitution1(queryInstance, *var1, atom1);
		}
	}
	else { // arg2->type()==TermType::VARIABLE
		auto var2 = (Variable*)args[1].get();
		auto atom1 = ((StringTerm*)args[0].get());
		auto atom3 = ((StringTerm*)args[2].get());
		if(boost::algorithm::starts_with(atom3->value(), atom1->value())) {
			auto atom2 = std::make_shared<StringTerm>(
					atom3->value().substr(atom1->value().length()));
			// return var2=atom2
			pushSubstitution1(queryInstance, *var2, atom2);
		}
	}
	// send EOS indicating that no more solutions will be generated.
	queryInstance->pushEOS();
}
