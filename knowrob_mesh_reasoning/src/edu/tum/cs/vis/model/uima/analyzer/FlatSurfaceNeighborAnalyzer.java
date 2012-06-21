/*******************************************************************************
 * Copyright (c) 2012 Stefan Profanter. All rights reserved. This program and the accompanying
 * materials are made available under the terms of the GNU Public License v3.0 which accompanies
 * this distribution, and is available at http://www.gnu.org/licenses/gpl.html
 * 
 * Contributors: Stefan Profanter - initial API and implementation, Year: 2012
 ******************************************************************************/
package edu.tum.cs.vis.model.uima.analyzer;

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;
import java.util.concurrent.Callable;
import java.util.concurrent.atomic.AtomicInteger;

import org.apache.log4j.Logger;

import edu.tum.cs.uima.Annotation;
import edu.tum.cs.vis.model.uima.annotation.FlatSurfaceAnnotation;
import edu.tum.cs.vis.model.uima.cas.MeshCas;

/**
 * Sets the neighbors of a flat surface annotation
 * 
 * @author Stefan Profanter
 * 
 */
public class FlatSurfaceNeighborAnalyzer extends MeshAnalyzer {

	/**
	 * Log4J Logger
	 */
	private static Logger				logger				= Logger.getLogger(FlatSurfaceNeighborAnalyzer.class);

	/**
	 * First a list of all threads is created which will be executed afterwards with a thread pool
	 */
	private final List<Callable<Void>>	threads				= new LinkedList<Callable<Void>>();

	/**
	 * Number of currently processed polygons. Used for progress status.
	 */
	private final AtomicInteger			polygonsElaborated	= new AtomicInteger(0);
	ArrayList<FlatSurfaceAnnotation>	annotations			= new ArrayList<FlatSurfaceAnnotation>();

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
		return "FlatSurfaceNeighbor";
	}

	/**
	 * Called from the worker threads to update current progress cnt will be added to
	 * trianglesElaborated
	 * 
	 * @param cnt
	 *            number of elaborated polygons.
	 */
	void polygonsElaborated(int cnt) {
		polygonsElaborated.addAndGet(cnt);
	}

	/* (non-Javadoc)
	 * @see edu.tum.cs.vis.model.uima.analyzer.MeshAnalyzer#processStart(edu.tum.cs.vis.model.uima.cas.MeshCas)
	 */
	@Override
	public void processStart(MeshCas cas) {

		annotations.clear();
		polygonsElaborated.set(0);

		for (Annotation a : cas.getAnnotations()) {
			if (a instanceof FlatSurfaceAnnotation) {
				FlatSurfaceAnnotation fsa = (FlatSurfaceAnnotation) a;
				annotations.add(fsa);
			}
		}

		int startIdx = 0;
		int interval = 100;

		do {
			threads.add(new FlatSurfaceNeighborAnalyzerThread(startIdx, Math.min(startIdx
					+ interval, annotations.size()), annotations, this, cas));
			startIdx += interval;
		} while (startIdx < annotations.size());

		executeInPool(threads);

		for (FlatSurfaceAnnotation fsa : annotations)
			fsa.getNeighborPolygons().clear();
		annotations.clear();

	}

	/* (non-Javadoc)
	 * @see edu.tum.cs.vis.model.uima.analyzer.MeshAnalyzer#updateProgress()
	 */
	@Override
	public void updateProgress() {
		if (annotations != null && annotations.size() > 0)
			setProgress(polygonsElaborated.get() / (float) annotations.size() * 100.0f);

	}

}

/**
 * Worker thread for elaborating a part of the polygon list in a thread pool.
 * 
 * @author Stefan Profanter
 * 
 */
class FlatSurfaceNeighborAnalyzerThread implements Callable<Void> {

	/**
	 * Index of first polygon in list to elaborate
	 */
	final int									start;
	/**
	 * All polygons in the list from start to < end will be elaborated
	 */
	final int									end;

	/**
	 * List of all polygons. <code>start</code> and <code>end</code> are the indices which indicate
	 * the range to elaborate.
	 */
	final ArrayList<FlatSurfaceAnnotation>		annotations;

	/**
	 * The parent analyzer. Used to update progress.
	 */
	private final FlatSurfaceNeighborAnalyzer	analyzer;

	/**
	 * the cas which is being analyzed
	 */
	private final MeshCas						cas;

	/**
	 * Default constructor.
	 * 
	 * @param start
	 *            Start index in polygons. Where to start elaboration.
	 * @param end
	 *            End index in polygons. Where to end elaboration.
	 * @param annotations
	 *            List of all flat surface annotations.
	 * @param analyzer
	 *            parent analyzer used to update progress.
	 * @param cas
	 *            the cas which is being analyzed
	 */
	public FlatSurfaceNeighborAnalyzerThread(int start, int end,
			ArrayList<FlatSurfaceAnnotation> annotations, FlatSurfaceNeighborAnalyzer analyzer,
			MeshCas cas) {
		this.start = start;
		this.end = end;
		this.annotations = annotations;
		this.analyzer = analyzer;
		this.cas = cas;
	}

	@Override
	public Void call() throws Exception {

		for (int i = start; i < end; i++) {
			FlatSurfaceAnnotation tr = annotations.get(i);
			for (int j = i + 1; j < annotations.size(); j++) {
				FlatSurfaceAnnotation n = annotations.get(j);
				n.addNeighbor(tr);
			}
		}
		analyzer.polygonsElaborated(end - start);
		return null;
	}

}
