package edu.tum.cs.uima;

import java.util.ArrayList;

/**
 * Dummy for UIMA Framework
 * 
 * @author Stefan Profanter
 * 
 */
public abstract class JCas {
	/**
	 * List of annotations for this CAS
	 */
	protected ArrayList<Annotation>	annotations	= new ArrayList<Annotation>();

	/**
	 * Dummy for UIMA framework
	 * 
	 * @return list of annotations
	 */
	public ArrayList<Annotation> getAnnotations() {
		return annotations;
	}

	/**
	 * Dummy for UIMA framework
	 * 
	 * @param annotations
	 *            value to set
	 */
	public void setAnnotations(ArrayList<Annotation> annotations) {
		this.annotations = annotations;
	}
}
