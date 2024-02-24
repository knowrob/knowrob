/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include <queue>
#include <set>
#include "knowrob/semweb/Property.h"

using namespace knowrob::semweb;

Property::Property(std::string_view iri)
: Resource(iri), flags_(0) {}

Property::Property(const IRIAtomPtr &iri)
: Resource(iri), flags_(0) {}

void Property::addDirectParent(const std::shared_ptr<Property> &directParent)
{ directParents_.push_back(directParent); }

void Property::setInverse(const std::shared_ptr<Property> &inverse)
{ inverse_ = inverse; }

bool Property::hasFlag(PropertyFlag flag) const
{ return flags_ & flag; }

void Property::setFlag(PropertyFlag flag)
{ flags_ |= flag; }

void Property::forallParents(const PropertyVisitor &visitor,
                             bool includeSelf,
                             bool skipDuplicates)
{
    std::queue<Property*> queue_;
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
