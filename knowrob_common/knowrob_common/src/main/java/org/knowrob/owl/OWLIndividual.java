/*
 * Copyright (c) 2012 Moritz Tenorth
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Technische Universiteit Eindhoven nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
*/
package org.knowrob.owl;

import java.util.Collection;
import java.util.Collections;
import java.util.HashMap;
import java.util.Map;
import java.util.Vector;

import org.knowrob.prolog.PrologInterface;
import org.knowrob.prolog.PrologQueryUtils;


public class OWLIndividual extends OWLThing {
	

	/**
	 * List of strings representing the object types (OWL classes)
	 */
	protected Vector<OWLClass> types;

	/**
	 * Map of data properties that are defined for this individual to their respective values.
	 */
	protected Map<String, Vector<String>> data_props;
	
	/**
	 * Map of object properties that are defined for this individual to their respective values.
	 */
	protected Map<String, Vector<String>> obj_props;
	


	/**
	 * Constructor. Set the IRI and optionally a label. If none is given, 
	 * it is initialized with the IRI's short name.
	 * 
	 * @param iri Identifier of this thing.
	 * @param label Optional natural-language label.
	 */
	protected OWLIndividual(String iri, String label) {
		
		super(iri, label);
		this.types = new Vector<OWLClass>();
		data_props = Collections.synchronizedMap(new HashMap<String, Vector<String>>());
		obj_props = Collections.synchronizedMap(new HashMap<String, Vector<String>>());
	}


	/**
	 * Copy constructor: create OWLIndividual from more generic {@link OWLThing}
	 * 
	 * @param ind {@link OWLIndividual} to be copied into this {@link OWLThing}
	 */
	protected OWLIndividual(OWLThing ind) {
		this(ind.getIRI(), ind.getLabel());
	}
	
	
	/**
	 * OWLIndividual factory. Return existing instance, if available, and create new
	 * OWLIndividual instance if necessary. Avoids duplicate instances with the same 
	 * IRI.
	 * 
	 * @param iri Identifier of this thing.
	 * @param label Optional natural-language label.
	 * @return Instance of an {@link OWLIndividual} with the specified IRI
	 */
	public static OWLIndividual getOWLIndividual(String iri, String label) {

		// return exact match if available
		if(identifiers.containsKey(iri) && identifiers.get(iri) instanceof OWLIndividual) {
			return (OWLIndividual) identifiers.get(iri);			
		}
		
		// create OWLIndividual from higher-level objects if the existing object for this IRI has a more abstract type
		OWLIndividual res = new OWLIndividual(OWLThing.getOWLThing(iri, label));
		identifiers.put(iri, res);
		return res;
	}
	
	/**
	 * OWLIndividual factory. Return existing instance, if available, and create new
	 * OWLIndividual instance if necessary. Avoids duplicate instances with the same 
	 * IRI.
	 * 
	 * @param iri Identifier of this thing.
	 * @return Instance of an {@link OWLIndividual} with the specified IRI
	 */
	public static OWLIndividual getOWLIndividual(String iri) {
		return getOWLIndividual(iri, null); 
	}
	
	
	
	/**
	 * OWLIndividual factory. Create new individual for class cl, creating a new 
	 * unique identifier. Adds cl to the types of the created individual.
	 * 
	 * @param cl OWL class of this thing.
	 * @return Instance of an {@link OWLIndividual} of the specified class
	 */
	public static OWLIndividual getOWLIndividualOfClass(String cl) {
		
		OWLIndividual res = getOWLIndividual(getUniqueID(cl), null);
		res.addType(OWLClass.getOWLClass(cl));
		
		return res;
	}


	/**
	 * Get 
	 * @return
	 */
	public Vector<OWLClass> getTypes() {
		return types;
	}

	/**
	 * 
	 * @param types
	 */
	public void setTypes(Vector<OWLClass> types) {
		this.types.clear();
		this.types.addAll(types);
	}

	/**
	 * 
	 * @param type
	 */
	public void addType(OWLClass type) {
		types.add(type);
	}

	/**
	 * 
	 * @param type
	 */
	public void addTypes(Collection<OWLClass> t) {
		types.addAll(t);
	}

	/**
	 * 
	 * @param type
	 * @return
	 */
	public boolean hasType(OWLClass type) {
		
		if(types.contains(type))
			return true;
		else
			return false;
	}
	
	public boolean hasType(String type_iri) {
		
		for(OWLClass c : types) {
			if(c.getIRI().equals(type_iri) || c.getShortName().equals(type_iri))
				return true;
		}
		return false;
	}

	/**
	 * Get the data properties defined for this individual.
	 * 
	 * @return Map from property identifiers to a Vector with their values.
	 */
	public Map<String, Vector<String>> getDataProperties() {
		return data_props;
	}
	
	/**
	 * Get the object properties defined for this individual.
	 * 
	 * @return Map from property identifiers to a Vector with their values.
	 */
	public Map<String, Vector<String>> getObjProperties() {
		return obj_props;
	}
	
	/**
	 * Set the internal data properties map to the given one (after resetting it first).
	 * 
	 * @param properties New map of data properties to be used.
	 */
	public void setDataProperties(Map<String, Vector<String>> properties) {
		this.data_props.clear();
		this.data_props.putAll(properties);
	}
	
	/**
	 * Set the internal object properties map to the given one (after resetting it first).
	 * 
	 * @param properties New map of object properties to be used.
	 */
	public void setObjProperties(Map<String, Vector<String>> properties) {
		this.obj_props.clear();
		this.obj_props.putAll(properties);
	}

	
	/**
	 * Check whether a data property is defined for this individual.
	 * 
	 * @param property Identifier of the property to be checked.
	 * @return true if this property exists
	 */
	public boolean hasDataProperty(String property) {
		if(data_props.containsKey(property))
			return true;
		else return false;
	}
	
	/**
	 * Check whether an object property is defined for this individual.
	 * 
	 * @param property Identifier of the property to be checked.
	 * @return true if this property exists
	 */
	public boolean hasObjProperty(String property) {
		if(obj_props.containsKey(property))
			return true;
		else return false;
	}

	/**
	 * Get a {@link Vector} of all values defined for a data property.
	 *  
	 * @param property Identifier of the property to be retrieved.
	 * @return Vector with the defined values.
	 */
	public Vector<String> getDataPropValues(String property) {
		return data_props.get(property);
	}
	
	/**
	 * Get a {@link Vector} of all values defined for an object property.
	 *  
	 * @param property Identifier of the property to be retrieved.
	 * @return Vector with the defined values.
	 */
	public Vector<String> getObjPropValues(String property) {
		return obj_props.get(property);
	}
	
	/**
	 * Add a data property-value pair to the properties map.
	 * 
	 * @param property
	 * @param value
	 */
	public void addDataPropValue(String property, String value) {
		
		if(!data_props.containsKey(property))
			data_props.put(property, new Vector<String>());
		
		data_props.get(property).add(value);
	}
	
	/**
	 * Add an object property-value pair to the properties map.
	 * 
	 * @param property
	 * @param value
	 */
	public void addObjPropValue(String property, String value) {
		
		if(!obj_props.containsKey(property))
			obj_props.put(property, new Vector<String>());
		
		obj_props.get(property).add(value);
	}
	
	

	public void writeToProlog() {
		
		// check whether instance need to be written to avoid infinite loops
		if(!this.needsSaveToProlog()) {
			return;
		}

		// set flag that this class has been written 
		// (in the beginning of this method to avoid problems with infinite 
		// recursion due to recursive relations)
		this.setSaveToProlog(false);


		// check for deleted types
		for(String t : PrologQueryUtils.readTypesOfInstance(iri)) {
			
			OWLClass cl = OWLClass.getOWLClass(OWLThing.removeSingleQuotes(t));
			if(!types.contains(cl)) {
				PrologInterface.executeQuery("rdf_retractall('" + iri + "', rdf:type, '" + cl.getIRI() + "')");
			}
		}

		// set instance types
		for(OWLClass t : types) {
			if(PrologInterface.executeQuery("rdf_has('" + iri + "', rdf:type, '" + t.getIRI() + "')") == null)
				PrologInterface.executeQuery("rdf_assert('" + iri + "', rdf:type, '" + t.getIRI() + "')");
		}
		
		
		// write object properties
		for(String p : obj_props.keySet()) {
			
			PrologInterface.executeQuery("rdf_retractall('" + iri + "', '"+p+"', _)");
		
			for(String v : obj_props.get(p)) {
				PrologQueryUtils.assertObjectPropertyForInst(iri, p, v);
			}
		}
		
		// write data properties
		for(String p : data_props.keySet()) {

			PrologInterface.executeQuery("rdf_retractall('" + iri + "', '"+p+"', _)");

			for(String v : data_props.get(p)) {				
				String type = "http://www.w3.org/2001/XMLSchema#string";
				try {
					Float.valueOf(v);
					type = "http://www.w3.org/2001/XMLSchema#float";
				} catch(NumberFormatException e) {}
				PrologQueryUtils.assertDataPropertyForInst(iri, p, v, type);
			}
		}
	}
}
