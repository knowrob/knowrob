package edu.tum.cs.uima;

/**
 * Dummy for UIMA Framework
 * 
 * @author Stefan Profanter
 * 
 */
public abstract class AnalysisEngine {

	/**
	 * Dummy for UIMA framework
	 * 
	 * @param aJCas
	 *            CAS Object to analyze
	 */
	public abstract void process(JCas aJCas);

}
