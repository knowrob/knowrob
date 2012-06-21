package edu.tum.cs.vis.model.uima.analyzer;

import edu.tum.cs.uima.AnalysisEngine;
import edu.tum.cs.uima.JCas;
import edu.tum.cs.vis.model.uima.cas.MeshCas;

/**
 * Base class for all mesh AnalyzeEngines.
 * 
 * @author Stefan Profanter
 * 
 */
public abstract class MeshAnalyzer extends AnalysisEngine {

	@Override
	public void process(JCas aJCas) {

		if (!(aJCas instanceof MeshCas)) {
			System.err.println("[WARN]: " + this.getClass() + " can only handle MeshCas, not: "
					+ aJCas.getClass());
			return;
		}
		MeshCas cas = (MeshCas) aJCas;

		process(cas);
	}

	/**
	 * Processes the given MeshCas
	 * 
	 * @param cas
	 *            CAS to store found annotations.
	 */
	public abstract void process(MeshCas cas);

}
