/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_SEMWEB_PROPERTY_H
#define KNOWROB_SEMWEB_PROPERTY_H

#include <memory>
#include <list>
#include <functional>
#include "Resource.h"
#include "knowrob/terms/IRIAtom.h"
#include "Class.h"

namespace knowrob::semweb {
	/**
	 * Defines property type and semantics.
	 */
	enum PropertyFlag {
		DATATYPE_PROPERTY = 1 << 0,
		ANNOTATION_PROPERTY = 1 << 1,
		OBJECT_PROPERTY = 1 << 2,
		TRANSITIVE_PROPERTY = 1 << 3,
		REFLEXIVE_PROPERTY = 1 << 4,
		SYMMETRIC_PROPERTY = 1 << 5
	};

	// forward declaration
	class Property;

	// called for each parent in the property hierarchy
	using PropertyVisitor = std::function<void(Property &)>;

	/**
	 * A property used in knowledge graphs.
	 */
	class Property : public Resource {
	public:
		explicit Property(std::string_view iri);

		explicit Property(const IRIAtomPtr &iri);

		/**
		 * @param directParent a direct super property.
		 */
		void addDirectParent(const std::shared_ptr<Property> &directParent);

		/**
		 * @param directParent a direct super property.
		 */
		void removeDirectParent(const std::shared_ptr<Property> &directParent);

		/**
		 * @return all direct super properties of this property.
		 */
		const auto &directParents() const { return directParents_; }

		/**
		 * Define the inverse property of this property.
		 * @param inverse a property.
		 */
		void setInverse(const std::shared_ptr<Property> &inverse);

		/**
		 * @return the inverse of this property or a null pointer reference.
		 */
		const auto &inverse() const { return inverse_; }

		/**
		 * @param flag a property flag.
		 * @return true if this property has the flag.
		 */
		bool hasFlag(PropertyFlag flag) const;

		/**
		 * Define a flag of this property.
		 * @param flag a property flag.
		 */
		void setFlag(PropertyFlag flag);

		/**
		 * @return true if this property is a datatype property.
		 */
		bool isDatatypeProperty() const { return hasFlag(DATATYPE_PROPERTY); }

		/**
		 * @return true if this property is an annotation property.
		 */
		bool isAnnotationProperty() const { return hasFlag(ANNOTATION_PROPERTY); }

		/**
		 * @return true if this property is an object property.
		 */
		bool isObjectProperty() const { return hasFlag(OBJECT_PROPERTY); }

		/**
		 * @return true if this property is a transitive property.
		 */
		bool isTransitiveProperty() const { return hasFlag(TRANSITIVE_PROPERTY); }

		/**
		 * @return true if this property is a reflexive property.
		 */
		bool isReflexiveProperty() const { return hasFlag(REFLEXIVE_PROPERTY); }

		/**
		 * @return true if this property is a symmetric property.
		 */
		bool isSymmetricProperty() const { return hasFlag(SYMMETRIC_PROPERTY); }

		/**
		 * Visit all parents of this property.
		 * @param visitor a function that is called for each parent.
		 * @param includeSelf true if the property itself should be included.
		 * @param skipDuplicates true if duplicates should be skipped.
		 */
		void forallParents(const PropertyVisitor &visitor, bool includeSelf = true, bool skipDuplicates = true);

		/**
		 * @return the reification concept of this property.
		 */
		auto reification() const { return reification_; }

		/**
		 * Map an IRI of a property to the IRI of the concept that reifies the property.
		 * @param iri an IRI of a property.
		 * @return the reification concept of the property.
		 */
		static knowrob::IRIAtomPtr reifiedIRI(std::string_view iri);

		/**
		 * Map an IRI of a reification concept to the IRI of the property.
		 * @param iri an IRI of a reification concept.
		 * @return the property IRI.
		 */
		static knowrob::IRIAtomPtr unReifiedIRI(std::string_view iri);

	protected:
		struct Comparator {
			bool operator()(const std::shared_ptr<Property> &lhs, const std::shared_ptr<Property> &rhs) const;
		};

		std::shared_ptr<Property> inverse_;
		std::set<std::shared_ptr<Property>, Comparator> directParents_;
		std::shared_ptr<Class> reification_;
		int flags_;
	};

	using PropertyPtr = std::shared_ptr<Property>;

} // knowrob

#endif //KNOWROB_SEMWEB_PROPERTY_H
