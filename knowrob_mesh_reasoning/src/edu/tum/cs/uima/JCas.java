package edu.tum.cs.uima;

import java.io.Serializable;
import java.util.LinkedList;

/**
 * Dummy for UIMA Framework
 * 
 * @author Stefan Profanter
 * 
 */
public abstract class JCas implements Serializable {
	/**
	 * List of annotations for this CAS
	 */
	protected LinkedList<Annotation>	annotations	= new LinkedList<Annotation>();

	/**
	 * Dummy for UIMA framework
	 * 
	 * @return list of annotations
	 */
	public LinkedList<Annotation> getAnnotations() {
		return annotations;
	}

	/**
	 * Dummy for UIMA framework
	 * 
	 * @param annotations
	 *            value to set
	 */
	public void setAnnotations(LinkedList<Annotation> annotations) {
		this.annotations = annotations;
	}
}
