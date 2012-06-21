/*******************************************************************************
 * Copyright (c) 2012 Stefan Profanter. All rights reserved. This program and the accompanying
 * materials are made available under the terms of the GNU Public License v3.0 which accompanies
 * this distribution, and is available at http://www.gnu.org/licenses/gpl.html
 * 
 * Contributors: Stefan Profanter - initial API and implementation, Year: 2012
 ******************************************************************************/
package edu.tum.cs.vis.model.uima.analyzer;

import java.util.ArrayList;

import org.apache.log4j.Logger;

import edu.tum.cs.uima.Annotation;
import edu.tum.cs.vis.model.uima.annotation.FlatSurfaceAnnotation;
import edu.tum.cs.vis.model.uima.cas.MeshCas;

/**
 * Analyzer for convex and concav surfaces
 * 
 * @author Stefan Profanter
 * 
 */
public class RoundSurfaceAnalyzer extends MeshAnalyzer {

	/**
	 * Log4J Logger
	 */
	private static Logger	logger	= Logger.getLogger(RoundSurfaceAnalyzer.class);

	/* (non-Javadoc)
	 * @see edu.tum.cs.vis.model.uima.analyzer.MeshAnalyzer#getLogger()
	 */
	@Override
	public Logger getLogger() {
		return logger;
	}

	/* (non-Javadoc)
	 * @see edu.tum.cs.vis.model.uima.analyzer.MeshAnalyzer#getName()
	 */
	@Override
	public String getName() {
		return "RoundSurface";
	}

	/* (non-Javadoc)
	 * @see edu.tum.cs.vis.model.uima.analyzer.MeshAnalyzer#processStart(edu.tum.cs.vis.model.uima.cas.MeshCas)
	 */
	@Override
	public void processStart(MeshCas cas) {
		ArrayList<FlatSurfaceAnnotation> annotations = new ArrayList<FlatSurfaceAnnotation>();

		for (Annotation a : cas.getAnnotations()) {
			if (a instanceof FlatSurfaceAnnotation) {
				FlatSurfaceAnnotation fsa = (FlatSurfaceAnnotation) a;
				annotations.add(fsa);
			}
		}

	}

	/* (non-Javadoc)
	 * @see edu.tum.cs.vis.model.uima.analyzer.MeshAnalyzer#updateProgress()
	 */
	@Override
	public void updateProgress() {
		// TODO Auto-generated method stub

	}

}
