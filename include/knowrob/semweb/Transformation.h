/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_SEMWEB_TRANSFORMATION_H
#define KNOWROB_SEMWEB_TRANSFORMATION_H

namespace knowrob::semweb {

	// TODO: usecase (1): an ontology is loaded entirely into a local raptor world.
	//                    the transformation should be applied to the whole ontology,
	//                    and changed should be reflected in the raptor world.
	// TODO: usecase (2): a query is issued and the query should be transformed by the inverse
	//                    of the transformation before being sent to the reasoner, e.g. a sparql query engine.
	// TODO: usecase (2.1): the result of a reasoner is transformed by the transformation before being returned
	//                    to the system.
	class Transformation {
	public:
		Transformation() = default;
	};

}


#endif //KNOWROB_SEMWEB_TRANSFORMATION_H
