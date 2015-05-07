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

import java.util.Vector;

import javax.vecmath.Matrix4d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import org.knowrob.prolog.PrologInterface;
import org.knowrob.prolog.PrologQueryUtils;


public class ObjectInstance extends OWLIndividual {

	
	/**
	 * Dimensions of the bounding box of the object
	 */
	protected Vector3d dimensions;
	
	/**
	 * 4x4 pose matrix describing the position and orientation
	 */
	protected Matrix4d pose_matrix;
	
	/**
	 * Set of direct child objects (physical decomposition)
	 */
	protected Vector<ObjectInstance> physicalParts;
	
	

	/**
	 * Constructor. Set the IRI and optionally a label. If none is given, 
	 * it is initialized with the IRI's short name.
	 * 
	 * @param iri Identifier of this thing.
	 */
	protected ObjectInstance(String iri, String label) {
		
		super(iri, label);
		
		this.dimensions = new Vector3d(); 
		this.pose_matrix = new Matrix4d();
		this.pose_matrix.setIdentity();
		
		this.physicalParts = new Vector<ObjectInstance>();
	}


	/**
	 * Copy constructor: create ObjectInstance from more generic {@link OWLIndividual}
	 * 
	 * @param ind {@link OWLIndividual} to be copied into this {@link ObjectInstance}
	 */
	protected ObjectInstance(OWLIndividual ind) {
		
		this(ind.getIRI(), ind.getLabel());
		this.types.addAll(ind.getTypes());
		this.data_props.putAll(ind.getDataProperties());
		this.obj_props.putAll(ind.getObjProperties());
	}
	
	
	/**
	 * ObjectInstance factory. Return existing instance, if available, and create new
	 * ObjectInstance instance if necessary. Avoids duplicate instances with the same 
	 * IRI.
	 * 
	 * @param iri Identifier of this thing.
	 * @param label Optional natural-language label.
	 * @return Instance of an {@link ObjectInstance} with the specified IRI
	 */
	public static ObjectInstance getObjectInstance(String iri, String label) {

		// return exact match if available
		if(identifiers.containsKey(iri) && identifiers.get(iri) instanceof ObjectInstance) {
			return (ObjectInstance) identifiers.get(iri);			
		}
		
		// create ObjectInstance from higher-level objects if the existing object for this IRI has a more abstract type
		ObjectInstance res = new ObjectInstance(OWLIndividual.getOWLIndividual(iri, label));
		identifiers.put(iri, res);
		return res;
	}
	
	/**
	 * ObjectInstance factory. Return existing instance, if available, and create new
	 * ObjectInstance instance if necessary. Avoids duplicate instances with the same 
	 * IRI.
	 * 
	 * @param iri Identifier of this thing.
	 * @return Instance of an {@link ObjectInstance} with the specified IRI
	 */
	public static ObjectInstance getObjectInstance(String iri) {
		return getObjectInstance(iri, null); 
	}
	
	
	

	/**
	 * 
	 * @return
	 */
	public Vector<ObjectInstance> getPhysicalParts() {
		return physicalParts;
	}

	/**
	 * 
	 * @param physicalParts
	 */
	public void setPhysicalParts(Vector<ObjectInstance> physicalParts) {
		this.physicalParts = physicalParts;
	}


	/**
	 * 
	 * @param physicalParts
	 */
	public void addPhysicalPart(ObjectInstance part) {
		this.physicalParts.add(part);
	}


	/**
	 * 
	 * @param physicalParts
	 */
	public void removePhysicalPart(ObjectInstance part) {
		this.physicalParts.remove(part);
	}


	/**
	 * 
	 * @param physicalParts
	 */
	public boolean hasPhysicalPart(ObjectInstance part) {
		if(this.physicalParts.contains(part))
			return true;
		else return false;
	}
	
	
	

	/**
	 * Get the dimensions of the 3D bounding box
	 * 
	 * @return Vector with the object dimensions
	 */
	public Vector3d getDimensions() {
		return dimensions;
	}


	/**
	 * Set the dimensions of the 3D bounding box
	 * 
	 * @param dimensions Vector with the object dimensions
	 */
	public void setDimensions(Vector3d dimensions) {
		this.dimensions = dimensions;
	}

	
	/**
	 * Get the 4x4 pose matrix
	 * 
	 * @return Pose matrix
	 */
	public Matrix4d getPoseMatrix() {
		return pose_matrix;
	}


	/**
	 * Set the 4x4 pose matrix
	 * 
	 * @param poseMatrix Pose matrix
	 */
	public void setPoseMatrix(Matrix4d poseMatrix) {
		pose_matrix = poseMatrix;
	}
	
	
	/**
	 * Get only the translation component of the pose (3D position)
	 * 
	 * @return Position vector
	 */
	public Vector3d getPosition() {
		return new Vector3d(pose_matrix.m03, 
							pose_matrix.m13, 
							pose_matrix.m23);
	}


	/**
	 * Set only the translation component of the pose
	 * 
	 * @param position Position vector
	 */
	public void setPosition(Vector3d position) {
		pose_matrix.setElement(0, 3, position.x);
		pose_matrix.setElement(1, 3, position.y);
		pose_matrix.setElement(2, 3, position.z);
	}

	
	/**
	 * Interface for getting the orientation as quaternion.
	 * 
	 * @return Object orientation as quaternion
	 */
	public Quat4d getPoseQuaternion() {
		Quat4d res = new Quat4d();
		res.set(this.pose_matrix);
		return res;
	}
	

	/**
	 * Interface for setting the pose as vector and quaternion.
	 * 
	 * @param translation Position vector
	 * @param orientation Orientation quaternion
	 * @param scale Scale value
	 */
	public void setPoseQuaternion(Vector3d translation, Quat4d orientation, double scale) {
		pose_matrix = new Matrix4d(orientation, translation, scale);
	}

	
	
	public Vector<Float> getPoseAsVector() {
		Vector<Float> r = new Vector<Float>();
		
		for(int i=0;i<4;i++)
			for(int j=0;j<4;j++)
				r.add((float) pose_matrix.getElement(i, j));
		
		return r;
	}
			
			
	
	public void readFromProlog() {
		
		// read dimensions, pose, children
	}
	
	public void readFromPrologWithChildren() {
		
		readFromProlog();
		
		for(ObjectInstance inst : physicalParts)
			inst.readFromProlog();
	}
	
	public void writeToProlog() {

		// write general instance-related stuff
		super.writeToProlog();
		
		
		// write object pose (new pose automatically overrides old one)
		PrologQueryUtils.setObjPose(iri, "SemanticMapPerception",  getPoseAsVector());
		
		
		// write object dimensions
		PrologInterface.executeQuery("rdf_retractall('" + iri + "', 'http://knowrob.org/kb/knowrob.owl#depthOfObject', _)");
		PrologInterface.executeQuery("rdf_retractall('" + iri + "', 'http://knowrob.org/kb/knowrob.owl#widthOfObject', _)");
		PrologInterface.executeQuery("rdf_retractall('" + iri + "', 'http://knowrob.org/kb/knowrob.owl#heightOfObject', _)");
		PrologQueryUtils.assertDataPropertyForInst(iri, "http://knowrob.org/kb/knowrob.owl#depthOfObject",  dimensions.x+"", "http://www.w3.org/2001/XMLSchema#float");
		PrologQueryUtils.assertDataPropertyForInst(iri, "http://knowrob.org/kb/knowrob.owl#widthOfObject",  dimensions.y+"", "http://www.w3.org/2001/XMLSchema#float");
		PrologQueryUtils.assertDataPropertyForInst(iri, "http://knowrob.org/kb/knowrob.owl#heightOfObject", dimensions.z+"", "http://www.w3.org/2001/XMLSchema#float");
		
		
		// write child objects
		PrologInterface.executeQuery("rdf_retractall('" + iri + "', 'http://knowrob.org/kb/knowrob.owl#properPhysicalParts', _)");
		
		for(ObjectInstance child : physicalParts) {
			child.writeToProlog();
			PrologQueryUtils.assertObjectPropertyForInst(iri, "http://knowrob.org/kb/knowrob.owl#properPhysicalParts",  child.getIRI());
		}
	}
}

