//
// Created by daniel on 07.04.23.
//

#include <queue>
#include <set>
#include "knowrob/semweb/Class.h"

using namespace knowrob::semweb;

Class::Class(std::string_view iri)
: Resource(iri) {}

void Class::addDirectParent(const std::shared_ptr<Class> &directParent)
{ directParents_.push_back(directParent); }

void Class::forallParents(const ClassVisitor &visitor,
                          bool includeSelf,
                          bool skipDuplicates)
{
    std::queue<Class*> queue_;
    std::set<std::string_view> visited_;

    // push initial elements to the queue
    if(includeSelf) queue_.push(this);
    else for(auto &x : directParents_) queue_.push(x.get());

    // visit each parent
    while(!queue_.empty()) {
        auto front = queue_.front();
        queue_.pop();
        // visit popped property
        visitor(*front);
        // remember visited nodes
        if(skipDuplicates) visited_.insert(front->iri());
        // push parents of visited property on the queue
        for(auto &directParent : front->directParents_) {
            if(skipDuplicates && visited_.count(directParent->iri())>0) continue;
            queue_.push(directParent.get());
        }
    }
}
