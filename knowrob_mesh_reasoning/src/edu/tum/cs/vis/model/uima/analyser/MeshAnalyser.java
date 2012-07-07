/*******************************************************************************
 * Copyright (c) 2012 Stefan Profanter. All rights reserved. This program and the accompanying
 * materials are made available under the terms of the GNU Public License v3.0 which accompanies
 * this distribution, and is available at http://www.gnu.org/licenses/gpl.html
 * 
 * Contributors: Stefan Profanter - initial API and implementation, Year: 2012
 ******************************************************************************/
package edu.tum.cs.vis.model.uima.analyser;

import org.apache.log4j.Logger;

import edu.tum.cs.uima.AnalysisEngine;
import edu.tum.cs.uima.JCas;
import edu.tum.cs.util.PrintUtil;
import edu.tum.cs.vis.model.uima.cas.MeshCas;

/**
 * Base class for all mesh AnalyseEngines.
 * 
 * @author Stefan Profanter
 * 
 */
public abstract class MeshAnalyser extends AnalysisEngine {

	/**
	 * Current progress in percent (from 0 to 100). If it is -1, AnalysisEngine hasn't started yet
	 */
	private float	progress			= -1;

	/**
	 * Value of System.currentTimeMillis() when analysing started
	 */
	private long	processStartTime	= 0;

	/**
	 * Duration in milliseconds of last process call.
	 */
	private long	processDuration		= 0;

	/**
	 * Returns the log4j logger for this class
	 * 
	 * @return log4j logger
	 */
	public abstract Logger getLogger();

	/**
	 * Short name for the analyser. Eg. "FlatSurface". Used for list on MeshReasoningViewControl.
	 * 
	 * @return the name of the analyser.
	 */
	public abstract String getName();

	/**
	 * Returns a concatenation of <code>getName</code> and the current progress status (waiting,
	 * current duration, or ended with duration)
	 * 
	 * @return String in format: getName() - [waiting|hh:mm:ss.xxx|ended hh:mm:ss.xxx]
	 */
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

		updateProgress();
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

	/**
	 * Called in <code>process</code> and <code>getProcess</code> to force class to update the
	 * process status/progress.
	 */
	public abstract void updateProgress();

}
