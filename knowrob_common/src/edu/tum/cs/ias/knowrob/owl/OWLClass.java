package edu.tum.cs.ias.knowrob.owl;

import java.util.Vector;

public class OWLClass extends OWLThing {

	
	/**
	 * Vector of subclasses
	 */
	Vector<OWLClass> subclasses;
	
	/**
	 * Reference to the parent / superclass
	 */
	Vector<OWLClass> parents;

	
	/**
	 * Constructor. Set the IRI and optionally a label. If none is given, 
	 * it is initialized with the IRI's short name.
	 * 
	 * @param iri Identifier of this thing.
	 * @param label Optional natural-language label.
	 */
	protected OWLClass(String iri, String label) {
		
		super(iri, label);
		this.parents = new Vector<OWLClass>();
		this.subclasses = new Vector<OWLClass>();
	}


	/**
	 * Copy constructor: create OWLClass from more generic {@link OWLThing}
	 * 
	 * @param ind {@link OWLClass} to be copied into this {@link OWLThing}
	 */
	protected OWLClass(OWLThing ind) {
		this(ind.getIRI(), ind.getLabel());
	}
	
	
	/**
	 * OWLClass factory. Return existing instance, if available, and create new
	 * OWLClass instance if necessary. Avoids duplicate instances with the same 
	 * IRI.
	 * 
	 * @param iri Identifier of this thing.
	 * @param label Optional natural-language label.
	 * @return Instance of an {@link OWLClass} with the specified IRI
	 */
	public static OWLClass getOWLClass(String iri, String label) {

		OWLClass res = new OWLClass(OWLThing.getOWLThing(iri, label));
		identifiers.put(iri, res);
		return res;
	}
	
	/**
	 * OWLClass factory. Return existing instance, if available, and create new
	 * OWLClass instance if necessary. Avoids duplicate instances with the same 
	 * IRI.
	 * 
	 * @param iri Identifier of this thing.
	 * @return Instance of an {@link OWLClass} with the specified IRI
	 */
	public static OWLClass getOWLClass(String iri) {
		return getOWLClass(iri, null); 
	}
	
	
	
	
	/**
	 * Get all subclasses defined for this OWL class.
	 * 
	 * @return Vector of subclasses
	 */
	public Vector<OWLClass> getSubclasses() {
		return subclasses;
	}
	
	
	/**
	 * Add a subclass definition. Complementary to addParentClass.
	 * 
	 * @param sub Subclass
	 */
	public void addSubclass(OWLClass sub) {

		subclasses.add(sub);
		sub.addParentClass(this);
	}
	
	
	/**
	 * Remove a subclass from this class
	 * 
	 * @param sub Subclass to be removed
	 */
	public void removeSubclass(OWLClass sub) {
		subclasses.remove(sub);
	}
	
	
	/**
	 * Set the parent classes of this class
	 * 
	 * @param p Parent class
	 */
	public void setParentClasses(Vector<OWLClass> p) {
		this.parents.clear();
		this.parents.addAll(p);
	}
	
	/**
	 * Add a superclass definition. Complementary to addSubclass.
	 * 
	 * @param p Superclass
	 */
	public void addParentClass(OWLClass p) {
		p.addSubclass(this);
	}
	
	
	/**
	 * Get the parent classes of this class
	 * @return
	 */
	public Vector<OWLClass> getParents() {
		return parents;
	}

}
