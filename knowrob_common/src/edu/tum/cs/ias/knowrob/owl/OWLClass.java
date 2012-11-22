package edu.tum.cs.ias.knowrob.owl;

import java.io.UnsupportedEncodingException;
import java.net.URLDecoder;
import java.util.Collections;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedHashMap;
import java.util.Map;
import java.util.Vector;

import edu.tum.cs.ias.knowrob.prolog.PrologInterface;
import edu.tum.cs.ias.knowrob.prolog.PrologQueryUtils;


public class OWLClass extends OWLThing {


	/**
	 * Vector of subclasses
	 */
	protected Vector<OWLClass> subclasses;

	/**
	 * Reference to the parent / superclass
	 */
	protected Vector<OWLClass> superclasses;


	/**
	 * Map of existential restrictions (someValuesFrom)
	 */
	protected Map<String, Vector<String>> some_values_from;

	/**
	 * Map of universal restrictions (allValuesFrom)
	 */
	protected Map<String, Vector<String>> all_values_from;

	/**
	 * Map of value restrictions (hasValue)
	 */
	protected Map<String, Vector<String>> has_value;




	/**
	 * Constructor. Set the IRI and optionally a label. If none is given, 
	 * it is initialized with the IRI's short name.
	 * 
	 * @param iri Identifier of this thing.
	 * @param label Optional natural-language label.
	 */
	protected OWLClass(String iri, String label) {

		super(iri, label);
		this.superclasses = new Vector<OWLClass>();
		this.subclasses = new Vector<OWLClass>();

		some_values_from = Collections.synchronizedMap(new LinkedHashMap<String, Vector<String>>());
		all_values_from = Collections.synchronizedMap(new LinkedHashMap<String, Vector<String>>());
		has_value = Collections.synchronizedMap(new LinkedHashMap<String, Vector<String>>());
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

		// return exact match if available
		if(identifiers.containsKey(iri) && identifiers.get(iri) instanceof OWLClass) {
			return (OWLClass) identifiers.get(iri);			
		}

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
	 * Set the superclasses of this class
	 * 
	 * @param p Parent class
	 */
	public void setSuperClasses(Vector<OWLClass> p) {
		this.superclasses.clear();
		this.superclasses.addAll(p);
	}

	/**
	 * Add a superclass definition. Complementary to addSubclass.
	 * 
	 * @param p Superclass
	 */
	public void addSuperClass(OWLClass p) {
		
		if(!superclasses.contains(p))
			superclasses.add(p);
	}


	/**
	 * Get the superclasses of this class
	 * @return
	 */
	public Vector<OWLClass> getSuperClasses() {
		return superclasses;
	}




	/**
	 * Set the someValuesFrom restrictions
	 * 
	 * @param someValuesFrom Map with the defined restrictions
	 */
	public void setSomeValuesFrom(Map<String, Vector<String>> someValuesFrom) {
		this.some_values_from.putAll(someValuesFrom);
	}


	/**
	 * Add a someValuesFrom  definition
	 * 
	 * @param prop Property to be used in restriction
	 * @param classdef Class definition for restriction
	 */
	public void addSomeValuesFrom(String prop, String classdef) {

		if(!some_values_from.containsKey(prop)) {
			some_values_from.put(prop, new Vector<String>());
		}
		some_values_from.get(prop).add(classdef);
	}


	/**
	 * Get the someValuesFrom definitions of this class
	 * @return someValuesFrom definitions
	 */
	public Map<String, Vector<String>> getSomeValuesFrom() {
		return some_values_from;
	}




	/**
	 * Set the allValuesFrom restrictions
	 * 
	 * @param allValuesFrom Map with the defined restrictions
	 */
	public void setAllValuesFrom(Map<String, Vector<String>> allValuesFrom) {
		this.all_values_from.putAll(allValuesFrom);
	}


	/**
	 * Add a allValuesFrom  definition
	 * 
	 * @param prop Property to be used in restriction
	 * @param classdef Class definition for restriction
	 */
	public void addAllValuesFrom(String prop, String classdef) {

		if(!all_values_from.containsKey(prop)) {
			all_values_from.put(prop, new Vector<String>());
		}
		all_values_from.get(prop).add(classdef);
	}


	/**
	 * Get the allValuesFrom definitions of this class
	 * 
	 * @return allValuesFrom definitions
	 */
	public Map<String, Vector<String>> getAllValuesFrom() {
		return all_values_from;
	}




	/**
	 * Set the hasValue restrictions
	 * 
	 * @param hasValue Map with the defined restrictions
	 */
	public void setHasValue(Map<String, Vector<String>> hasValue) {
		this.has_value.putAll(hasValue);
	}


	/**
	 * Add a hasValue  definition
	 * 
	 * @param prop Property to be used in restriction
	 * @param value Individuals or values to be used for the restriction
	 */
	public void addHasValue(String prop, String value) {

		if(!has_value.containsKey(prop)) {
			has_value.put(prop, new Vector<String>());
		}
		has_value.get(prop).add(value);
	}


	/**
	 * Get the hasValue definitions of this class
	 * @return hasValue definitions
	 */
	public Map<String, Vector<String>> getHasValue() {
		return has_value;
	}



	/**
	 * Recursively read all properties of an OWL class into its internal data structures.
	 */
	public void readFromProlog() {

		if(isReadFromProlog())
			return;
		
		// Read the action's label if an rdfs:label is set (assuming IRI has been set during initialization) 
		try {
			HashMap<String, Vector<String>> qLabel = PrologInterface.executeQuery("rdf_has('"+iri+"',rdfs:label,L),util:strip_literal_type(L,Label)");

			if(qLabel.get("Label")!=null && qLabel.get("Label").size()>0) {
				this.label = OWLThing.removeSingleQuotes(qLabel.get("Label").get(0));
			}			
		} catch (Exception e) { } // fail silently if no label is set


		// Read superclasses 
		HashMap<String, Vector<String>> qSuper = PrologInterface.executeQuery("owl_direct_subclass_of('" + iri + "', Super)");

		if(qSuper != null) {

			for(String sup : qSuper.get("Super")) {

				if (sup.contains("__Description"))
					continue;

				OWLClass superclass = OWLClass.getOWLClass(OWLThing.removeSingleQuotes(sup));
				
				if(!superclasses.contains(superclass))
					superclasses.add(superclass);
			}
		}

		// Recursively read subclasses 
		HashMap<String, Vector<String>> subclasses = PrologInterface.executeQuery(
				"owl_direct_subclass_of(Sub, '" + iri + "')");

		if(subclasses!=null && subclasses.get("Sub") != null) {

			this.subclasses.clear();

			for(String sub_iri : subclasses.get("Sub")) {

				OWLClass sub = OWLClass.getOWLClass(OWLThing.removeSingleQuotes(sub_iri));
				sub.readFromProlog();

				this.addSubclass(sub);
				sub.addSuperClass(this);
			}
		}

		// Read class properties
		try {

			HashMap<String, Vector<String>> qProp = 
				PrologInterface.executeQuery("((class_properties_some('"+iri+"', Prop, V), Type='some'); " +
						"(class_properties_all('"+iri+"', Prop, V), Type='all'); " +
						"(class_properties_value('"+iri+"', Prop, V), Type='value')), " +
				"util:strip_literal_type(V,Val)");

			if(qProp != null) {

				Vector<String> prop = qProp.get("Prop");
				Vector<String> val  = qProp.get("Val");
				Vector<String> type = qProp.get("Type");


				// Make sure each property is added only once 
				// (properties may be present two or more times in the result set)

				HashSet<String> alreadyAdded = new HashSet<String>();
				if(prop != null && val != null)

					for(int i=0;i<prop.size() && i<val.size();i++) {

						if (alreadyAdded.contains(prop.get(i)+val.get(i)))
							continue;

						alreadyAdded.add(prop.get(i)+val.get(i));
						String p = OWLThing.removeSingleQuotes(prop.get(i));
						String v = OWLThing.removeSingleQuotes(val.get(i));

						// HACK: remove faulty local URLs generated by Prolog export
						if(v.startsWith("file:")) {
							String[] vs = v.split("/");
							v = vs[vs.length-1];
							
							try {
								v = URLDecoder.decode(URLDecoder.decode(v, "UTF-8"), "UTF-8");
							} catch (UnsupportedEncodingException e) {
								e.printStackTrace();
							}
						}
						
						if(type.get(i).contains("some")) {
							this.addSomeValuesFrom(p, v);

						} else if(type.get(i).contains("all")) {
							this.addAllValuesFrom(p, v);

						} else if(type.get(i).contains("value")) {
							this.addHasValue(p, v);
						}
					}
			}

		} catch (Exception e) {
			e.printStackTrace();
		}

		// set flag that this class has been read
		this.setReadFromProlog(true);
	}


	/**
	 * Write all properties of this class to Prolog.
	 */
	public void writeToProlog() {


		// check whether classes need to be written to avoid infinite loops
		if(!this.needsSaveToProlog()) {
			return;
		}
		
		
		// set flag that this class has been written 
		// (in the beginning of this method to avoid problems with infinite 
		// recursion due to recursive relations)
		this.setSaveToProlog(false);
		
		
		// write label
		PrologInterface.executeQuery("rdf_assert('" + iri + "', rdfs:label, literal(type('http://www.w3.org/2001/XMLSchema#string','"+label+"')))"); 

		
		// Read class properties and check for removed ones
		HashMap<String, Vector<String>> qProp = 
			PrologInterface.executeQuery("((class_properties_some('"+iri+"', Prop, V), Type='some'); " +
					"(class_properties_all('"+iri+"', Prop, V), Type='all'); " +
					"(class_properties_value('"+iri+"', Prop, V), Type='value')), " +
			"util:strip_literal_type(V,Val)");

		if(qProp != null) {

			Vector<String> prop = qProp.get("Prop");
			Vector<String> val  = qProp.get("Val");
			Vector<String> type = qProp.get("Type");

			if(prop != null && val != null) {
				
				for(int i=0;i<prop.size() && i<val.size();i++) {

					String p = OWLThing.removeSingleQuotes(prop.get(i));
					String v = OWLThing.removeSingleQuotes(val.get(i));

					if(type.get(i).contains("some")) {

						if((!some_values_from.containsKey(p)) || (!some_values_from.get(p).contains(v))) { // TODO: too greedy!, removes all subActions

							// remove p,v pair
							PrologInterface.executeQuery("findall(R, (rdfs_subclass_of('"+iri+"', R), " +
									"rdf_has(R, 'http://www.w3.org/2002/07/owl#onProperty','"+p+"'), " +
									"rdf_has(R, 'http://www.w3.org/2002/07/owl#someValuesFrom', '"+v+"')), Rs), " +
									"member(Restr, Rs), rdf_retractall(Restr, _, _), " +
									"rdf_retractall(_, 'http://www.w3.org/2000/01/rdf-schema#subClassOf', Restr)");
						}

					} else if(type.get(i).contains("all")) {

						if((!all_values_from.containsKey(p)) || (!all_values_from.get(p).contains(v))) {

							// remove p,v pair
							PrologInterface.executeQuery("findall(R, (rdfs_subclass_of('"+iri+"', R), " +
									"rdf_has(R, 'http://www.w3.org/2002/07/owl#onProperty','"+p+"'), " +
									"rdf_has(R, 'http://www.w3.org/2002/07/owl#allValuesFrom', '"+v+"')), Rs), " +
									"member(Restr, Rs), rdf_retractall(Restr, _, _), " +
									"rdf_retractall(_, 'http://www.w3.org/2000/01/rdf-schema#subClassOf', Restr)");
						}

					} else if(type.get(i).contains("value")) {

						if((!has_value.containsKey(p)) || (!has_value.get(p).contains(v))) {

							// remove p,v pair
							PrologInterface.executeQuery("findall(R, (rdfs_subclass_of('"+iri+"', R), " + 
									"rdf_has(R, 'http://www.w3.org/2002/07/owl#onProperty','"+p+"'), " +
									"( rdf_has(R, 'http://www.w3.org/2002/07/owl#hasValue', literal(type(_,'"+v+"'))); " +
									"  rdf_has(R, 'http://www.w3.org/2002/07/owl#hasValue', '"+v+"'))), Rs), " +
									"member(Restr, Rs), rdf_retractall(Restr, _, _), " +
									"rdf_retractall(_, 'http://www.w3.org/2000/01/rdf-schema#subClassOf', Restr)");
						}
					}
				}
			}
		}
		
		
		
		// write class properties by creating appropriate restrictions
		for(String prop : has_value.keySet()) {
			for(String value : has_value.get(prop)) {
				PrologQueryUtils.createRestriction(iri,prop, value, "http://www.w3.org/2002/07/owl#hasValue", "knowrob_java");
			}
		}
		
		for(String prop : some_values_from.keySet()) {
			for(String value : some_values_from.get(prop)) {
				PrologQueryUtils.createRestriction(iri, prop, value, "http://www.w3.org/2002/07/owl#someValuesFrom", "knowrob_java");
			}
		}
		
		for(String prop : all_values_from.keySet()) {
			for(String value : all_values_from.get(prop)) {
				PrologQueryUtils.createRestriction(iri, prop, value, "http://www.w3.org/2002/07/owl#allValuesFrom", "knowrob_java");
			}
		}
		
		// write superclass statement
		for(OWLClass sup : superclasses) {
			PrologQueryUtils.assertSubClassOf(iri, sup.getIRI());
		}
		
		// write subclasses (by triggering their output method)
		for(OWLClass sub : subclasses) {
			sub.writeToProlog();
		}
		
	}
}
