//
// Created by daniel on 07.04.23.
//

#include <queue>
#include <set>
#include "knowrob/semweb/Class.h"
#include "knowrob/Logger.h"

using namespace knowrob::semweb;

Class::Class(std::string_view iri)
		: Resource(iri) {}

Class::Class(const IRIAtomPtr &iri)
		: Resource(iri) {}

bool Class::Comparator::operator()(const std::shared_ptr<Class> &lhs, const std::shared_ptr<Class> &rhs) const {
	return lhs->iri() < rhs->iri();
}

void Class::addDirectParent(const std::shared_ptr<Class> &directParent) {
	directParents_.insert(directParent);
}

bool Class::isDirectSubClassOf(const std::shared_ptr<Class> &directParent) {
	return directParents_.count(directParent) > 0;
}

bool Class::isSubClassOf(const std::shared_ptr<Class> &parent, bool includeSelf) {
	std::queue<Class *> queue_;
	std::set<std::string_view> visited_;

	if (includeSelf && this == parent.get()) return true;
	queue_.push(this);

	// visit each parent
	while (!queue_.empty()) {
		auto front = queue_.front();
		queue_.pop();

		// visit popped property
		if (front->directParents_.count(parent) > 0) return true;
		// remember visited nodes
		visited_.insert(front->iri());
		// push parents of visited property on the queue
		for (auto &directParent: front->directParents_) {
			if (visited_.count(directParent->iri()) > 0) continue;
			queue_.push(directParent.get());
		}
	}

	return false;
}

void Class::forallParents(const ClassVisitor &visitor,
						  bool includeSelf,
						  bool skipDuplicates) {
	std::queue<Class *> queue_;
	std::set<std::string_view> visited_;

	// push initial elements to the queue
	if (includeSelf) queue_.push(this);
	else for (auto &x: directParents_) queue_.push(x.get());

	// visit each parent
	while (!queue_.empty()) {
		auto front = queue_.front();
		queue_.pop();
		// visit popped property
		visitor(*front);
		// remember visited nodes
		if (skipDuplicates) visited_.insert(front->iri());
		// push parents of visited property on the queue
		for (auto &directParent: front->directParents_) {
			if (skipDuplicates && visited_.count(directParent->iri()) > 0) continue;
			queue_.push(directParent.get());
		}
	}
}
