/*******************************************************************************
 * Copyright (c) 2012 Stefan Profanter. All rights reserved. This program and the accompanying
 * materials are made available under the terms of the GNU Public License v3.0 which accompanies
 * this distribution, and is available at http://www.gnu.org/licenses/gpl.html
 * 
 * Contributors: Marco Attene - initial API and implementation, Year: 2006, Stefan Profanter -
 * ported C code to Java, Year 212
 ******************************************************************************/
package edu.tum.cs.vis.model.uima.analyzer;

import java.util.ArrayList;
import java.util.List;

import org.apache.log4j.Logger;

import edu.tum.cs.vis.model.uima.cas.MeshCas;
import edu.tum.cs.vis.model.util.Group;
import edu.tum.cs.vis.model.util.Mesh;
import edu.tum.cs.vis.model.util.Triangle;

public class FittingPrimitivesAnalyzer extends MeshAnalyzer {

	/**
	 * Log4J Logger
	 */
	private static Logger	logger	= Logger.getLogger(FittingPrimitivesAnalyzer.class);

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
		return "FittingPrimitives";
	}

	/**
	 * Process a group which contains a mesh.
	 * 
	 * @param g
	 *            group to process
	 */
	private void processGroup(final Group g, List<Triangle> polygons) {

		processMesh(g.getMesh(), polygons);

		for (Group gr : g.getChildren()) {
			processGroup(gr, polygons);
		}
	}

	/**
	 * Process a mesh which contains polygons and find the neighbors for each triangle.
	 * 
	 * @param m
	 *            Mesh to process
	 */
	void processMesh(final Mesh m, List<Triangle> polygons) {
		if (m.getTriangles().size() == 0)
			return;

		polygons.addAll(m.getTriangles());

	}

	/* (non-Javadoc)
	 * @see edu.tum.cs.vis.model.uima.analyzer.MeshAnalyzer#processStart(edu.tum.cs.vis.model.uima.cas.MeshCas)
	 */
	@Override
	public void processStart(MeshCas cas) {
		ArrayList<Triangle> allPolygons = new ArrayList<Triangle>();
		processGroup(cas.getGroup(), allPolygons);

	}

	/* (non-Javadoc)
	 * @see edu.tum.cs.vis.model.uima.analyzer.MeshAnalyzer#updateProgress()
	 */
	@Override
	public void updateProgress() {
		// TODO Auto-generated method stub

	}

}
