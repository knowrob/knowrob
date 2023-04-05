//
// Created by daniel on 03.04.23.
//

#ifndef KNOWROB_MONGO_GRAPH_HIERARCHY_H
#define KNOWROB_MONGO_GRAPH_HIERARCHY_H

#include <mongoc.h>
#include <string>

static inline bson_t* mngTripleTransitive(const std::string &tripleCollectionName,
                                          const std::string &oneCollectionName)
{
    /*
    (	has_value(S0,Ctx)
            ->	( Start=S_val, To='s', From='o', StartValue='$start.s' )
            ;	( Start=V_val, To='o', From='s', StartValue='$start.o' )
    )
    */
    // mng_query_value(P,Query_p)
    return BCON_NEW (
            "pipeline", "[",
            // recursive lookup
            "{", "$graphLookup", "{",
                "from", BCON_UTF8(tripleCollectionName.c_str()),
                "startWith", BCON_UTF8(Start),
                "connectToField", BCON_UTF8(To),
                "connectFromField", BCON_UTF8(From),
                "as", BCON_UTF8("t_paths"),
                "depthField", BCON_UTF8("depth")),
                "restrictSearchWithMatch", "{",
                    "p*", Query_p,
                    // TODO: include graph
                    // TODO: include scope
                "}",
            "}", "}",
            // $graphLookup does not ensure order, so we need to order by recursion depth in a separate step
            "{", "$lookup", "{",
                "from", BCON_UTF8(oneCollectionName.c_str()),
                "as", BCON_UTF8("t_sorted"),
                "let", "{", "t_paths", BCON_UTF8("$t_paths"), "}",
                "pipeline", "[",
                    "{", "$set",  "{", "t_paths", BCON_UTF8("$$t_paths"), "}", "}",
                    "{", "$unwind", BCON_UTF8("$t_paths"), "}",
                    "{", "$replaceRoot", "{", "newRoot", BCON_UTF8("$t_paths"), "}", "}",
                    "{", "$sort", "{", "depth", BCON_INT32(1), "}", "}",
                "]",
            "}", "}",
            "{", "$set", "{", "next", BCON_UTF8("$t_sorted"), "}", "}",
            "{", "$set", "{", "start", "{", "$arrayElemAt", "[", BCON_UTF8("$next"), BCON_INT32(0), "]", "}", "}", "}",
            "{", "$unset", BCON_UTF8("t_paths"), "}",
            "{", "$unset", BCON_UTF8("t_sorted"), "}",
            // TODO: add additional triple in next if P is a reflexive property
            // reflexivity(StartValue, Ctx, Step)
            // iterate over results
            "{", "$unwind", BCON_UTF8("$next"), "}",
            // TODO: match values with expression given in query.
            //       seems only conditionally if To id "s" ?!?
            // ( To=='s', has_value(V0,Ctx), Step=['$match', ['next.o', V_val]] )
            "{", "$match", "{", BCON_UTF8("next.o"), V_val, "}", "}",
            "]");
}

void mngTriple()
{
    /**
lookup_triple(triple(S,P,V), Ctx, Step) :-
	\+ memberchk(transitive, Ctx),
	memberchk(collection(Coll), Ctx),
	memberchk(step_vars(StepVars), Ctx),
	% TODO: revise below
	mng_triple_doc(triple(S,P,V), QueryDoc, Ctx),
	(	memberchk(['s',_],QueryDoc)
	->	StartValue='$start.s'
	;	StartValue='$start.o'
	),
	%
	(	taxonomical_property(P)
	->	( Key_p='$p',  Key_o='$o*' )
	;	( Key_p='$p*', Key_o='$o' )
	),
	findall(MatchQuery,
		% first match what is grounded in compile context
		(	MatchQuery=QueryDoc
		% next match variables grounded in call context
		;	(	member([Arg,FieldValue],[[S,'$s'],[P,Key_p],[V,Key_o]]),
				triple_arg_var(Arg, ArgVar),
				mongolog:var_key(ArgVar, Ctx, ArgKey),
				atom_concat('$$',ArgKey,ArgValue),
				atom_concat(ArgValue,'.type',ArgType),
				triple_arg_value(Arg, ArgValue, FieldValue, Ctx, ArgExpr),
				MatchQuery=['$expr', ['$or', array([
					% pass through if var is not grounded
					['$eq', array([string(ArgType), string('var')])],
					ArgExpr % else perform a match
				])]]
			)
		;	mongolog_scope_match(Ctx, MatchQuery)
		;	graph_match(Ctx, MatchQuery)
		),
		MatchQueries
	),
	%
	findall(InnerStep,
		% match input triples with query pattern
		(	(	MatchQueries=[FirstMatch]
			->	InnerStep=['$match', FirstMatch]
			;	InnerStep=['$match', ['$and', array(MatchQueries)]]
			)
		% limit results if requested
		;	(	member(limit(Limit),Ctx),
				InnerStep=['$limit',int(Limit)]
			)
		),
		InnerPipeline
	),
	% pass input document values to lookup
	mongolog:lookup_let_doc(StepVars, LetDoc),
	% lookup matching documents and store in 'next' field
    (	Step=['$lookup', [
			['from',string(Coll)],
			['as',string('next')],
			['let',LetDoc],
			['pipeline', array(InnerPipeline)]
		]]
	% add additional results if P is a reflexive property
	;	(	memberchk(reflexive,Ctx),
			(	Step=['$unwind',string('$next')]
			;	Step=['$set', ['start', string('$next')]]
			;	Step=['$set', ['next', array([string('$next')])]]
			;	reflexivity(StartValue, Ctx, Step)
			)
		)
	% at this point 'next' field holds an array of matching documents
	% that is unwinded here.
	;	Step=['$unwind',string('$next')]
	).
     */
}

void mngAssertTriple()
{
    /**
%%
% assert(triple(S,P,O)) uses $lookup to find matching triples
% with overlapping scope which are toggled to be removed in next stage.
% then the union of their scopes is computed and used for output document.
%
compile_assert(triple(S,P,O), Ctx, Pipeline) :-
	% add additional options to the compile context
	extend_context(triple(S,P,O), P1, Ctx, Ctx0),
	option(collection(Collection), Ctx0),
	option(query_scope(Scope), Ctx0),
	triple_graph(Ctx0, Graph),
	mongolog_time_scope(Scope, SinceTyped, UntilTyped),
	% throw instantiation_error if one of the arguments was not referred to before
	mongolog:all_ground([S,O], Ctx),
	% resolve arguments
	mongolog:var_key_or_val(S, Ctx, S_query),
	mongolog:var_key_or_val(O, Ctx, V_query),
	% special handling for RDFS semantic
	% FIXME: with below code P can not be inferred in query
	(	taxonomical_property(P1)
	->	( Pstar=array([string(P1)]), Ostar=string('$parents') )
	;	( Pstar=string('$parents'),  Ostar=array([V_query]) )
	),
	% build triple docuemnt
	TripleDoc=[
		['s', S_query], ['p', string(P1)], ['o', V_query],
		['p*', Pstar], ['o*', Ostar],
		['graph', string(Graph)],
		['scope', string('$v_scope')]
	],
	% configure the operation performed on scopes.
	% the default is to compute the union of scopes.
	(	option(intersect_scope, Ctx)
	->	(SinceOp='$max', UntilOp='$min')
	;	(SinceOp='$min', UntilOp='$max')
	),
	% compute steps of the aggregate pipeline
	% TODO: if just one document, update instead of delete
	findall(Step,
		% assign v_scope field.
		(	Step=['$set', ['v_scope', [['time',[
					['since', SinceTyped],
					['until', UntilTyped]
			]]]]]
		% lookup documents that overlap with triple into 'next' field,
		% and toggle their delete flag to true
		;	delete_overlapping(triple(S,P,O), SinceTyped, UntilTyped, Ctx0, Step)
		% lookup parent documents into the 'parents' field
		;	lookup_parents(triple(S,P1,O), Ctx0, Step)
		% update v_scope.time.since
		;	reduce_num_array(string('$next'), SinceOp,
				'scope.time.since', 'v_scope.time.since', Step)
		% get max until of scopes in $next, update v_scope.time.until
		;	reduce_num_array(string('$next'), UntilOp,
				'scope.time.until', 'v_scope.time.until', Step)
		% add triples to triples array that have been queued to be removed
		;	mongolog:add_assertions(string('$next'), Collection, Step)
		% add merged triple document to triples array
		;	mongolog:add_assertion(TripleDoc, Collection, Step)
		;	(	once(must_propagate_assert(P)),
				propagate_assert(S, Ctx0, Step)
			)
		),
		Pipeline
	).
     */
}

#endif //KNOWROB_MONGO_GRAPH_HIERARCHY_H
