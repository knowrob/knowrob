package edu.tum.cs.vis.model.uima.analyzer;

import org.apache.log4j.Logger;

import edu.tum.cs.uima.AnalysisEngine;
import edu.tum.cs.uima.JCas;
import edu.tum.cs.util.PrintUtil;
import edu.tum.cs.vis.model.uima.cas.MeshCas;

/**
 * Base class for all mesh AnalyzeEngines.
 * 
 * @author Stefan Profanter
 * 
 */
public abstract class MeshAnalyzer extends AnalysisEngine {

	private float	progress			= -1;

	private long	processStartTime	= 0;

	private long	processDuration		= 0;

	public abstract Logger getLogger();

	public abstract String getName();

	public String getNameAndProgress() {
		String progr;
		if (getProgress() < 0)
			progr = "waiting";
		else if (getProgress() >= 100)
			progr = "ended     (" + PrintUtil.prettyMillis(processDuration) + ")";
		else {
			progr = String.format("%3.2f%%", getProgress());
			long millis = System.currentTimeMillis() - processStartTime;
			progr += "     (" + PrintUtil.prettyMillis(millis) + ")";
		}
		return getName() + " - " + progr;
	}

	/**
	 * @return the progress
	 */
	public float getProgress() {
		updateProgress();
		return progress;
	}

	@Override
	public void process(JCas aJCas) {

		if (!(aJCas instanceof MeshCas)) {
			getLogger()
					.warn(this.getClass() + " can only handle MeshCas, not: " + aJCas.getClass());
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
	public void process(MeshCas cas) {
		progress = 0;
		updateProgress();
		getLogger().debug("Started");
		processStartTime = System.currentTimeMillis();
		processStart(cas);
		processDuration = System.currentTimeMillis() - processStartTime;

		getLogger().debug("Ended. Took: " + PrintUtil.prettyMillis(processDuration));
	}

	/**
	 * Processes the given MeshCas
	 * 
	 * @param cas
	 *            CAS to store found annotations.
	 */
	public abstract void processStart(MeshCas cas);

	/**
	 * @param progress
	 *            the progress to set
	 */
	public void setProgress(float progress) {
		this.progress = progress;
	}

	public abstract void updateProgress();

}
