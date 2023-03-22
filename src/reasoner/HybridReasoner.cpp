//
// Created by daniel on 12.03.23.
//

#include "knowrob/reasoner/HybridReasoner.h"

using namespace knowrob;

/*
- get or create an initial incomplete world graph
- get reasoning attention
- run graph completion with reasoner ensemble + attention
- run graph matching in parallel and yield results into modal pipeline
*/

HybridReasoner::HybridReasoner()
{
}
