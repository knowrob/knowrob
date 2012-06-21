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

import edu.tum.cs.vis.model.uima.cas.MeshCas;
import edu.tum.cs.vis.model.util.Group;
import edu.tum.cs.vis.model.util.Mesh;
import edu.tum.cs.vis.model.util.Polygon;

/**
 * Analyzer for a mesh which sets direct neighbors of a triangle.
 * 
 * The neighbor information is used in other Analyzer for better performance.
 * 
 * @author Stefan Profanter
 * 
 */
public class NeighborAnalyzer extends MeshAnalyzer {

	/**
	 * Log4j logger
	 */
	private static Logger				logger				= Logger.getLogger(NeighborAnalyzer.class);

	/**
	 * First a list of all threads is created which will be executed afterwards with a thread pool
	 */
	private final List<Callable<Void>>	threads				= new LinkedList<Callable<Void>>();

	/**
	 * Number of currently processed polygons. Used for progress status.
	 */
	private final AtomicInteger			polygonsElaborated	= new AtomicInteger(0);

	/**
	 * When calling <code>process</code> all polygons of the group and its children are collected in
	 * this list to process them afterwards.
	 */
	private ArrayList<Polygon>			allPolygons;

	@Override
	public Logger getLogger() {
		return logger;
	}

	@Override
	public String getName() {
		return "Neighbor";
	}

	/**
	 * Called from the worker threads to update current progress cnt will be added to
	 * polygonsElaborated
	 * 
	 * @param cnt
	 *            number of elaborated polygons.
	 */
	void polygonsElaborated(int cnt) {
		polygonsElaborated.addAndGet(cnt);
	}

	/**
	 * Process a group which contains a mesh.
	 * 
	 * @param g
	 *            group to process
	 */
	private void processGroup(final Group g) {

		processMesh(g.getMesh());

		for (Group gr : g.getChildren()) {
			processGroup(gr);
		}
	}

	/**
	 * Process a mesh which contains polygons and find the neighbors for each triangle.
	 * 
	 * @param m
	 *            Mesh to process
	 */
	void processMesh(final Mesh m) {
		if (m.getPolygons().size() == 0)
			return;

		allPolygons.addAll(m.getPolygons());

	}

	@Override
	public void processStart(MeshCas cas) {
		allPolygons = new ArrayList<Polygon>();
		processGroup(cas.getGroup());

		logger.debug("Number of Polygons: " + allPolygons.size());

		int startIdx = 0;
		int interval = 100;

		do {
			threads.add(new NeighborAnalyzerThread(startIdx, Math.min(startIdx + interval,
					allPolygons.size()), allPolygons, this));
			startIdx += interval;
		} while (startIdx < allPolygons.size());

		executeInPool(threads);

	}

	@Override
	public void updateProgress() {
		if (allPolygons != null)
			setProgress((float) polygonsElaborated.get() / (float) allPolygons.size() * 100.0f);
	}
}

/**
 * Worker thread for elaborating a part of the polygon list in a thread pool.
 * 
 * @author Stefan Profanter
 * 
 */
class NeighborAnalyzerThread implements Callable<Void> {

	/**
	 * Index of first polygon in list to elaborate
	 */
	final int						start;
	/**
	 * All polygons in the list from start to < end will be elaborated
	 */
	final int						end;

	/**
	 * List of all polygons. <code>start</code> and <code>end</code> are the indices which indicate
	 * the range to elaborate.
	 */
	final ArrayList<Polygon>		polygons;

	/**
	 * The parent analyzer. Used to update progress.
	 */
	private final NeighborAnalyzer	analyzer;

	/**
	 * Default constructor.
	 * 
	 * @param start
	 *            Start index in polygons. Where to start elaboration.
	 * @param end
	 *            End index in polygons. Where to end elaboration.
	 * @param polygons
	 *            List of all polygons.
	 * @param analyzer
	 *            parent analyzer used to update progress.
	 */
	public NeighborAnalyzerThread(int start, int end, ArrayList<Polygon> polygons,
			NeighborAnalyzer analyzer) {
		this.start = start;
		this.end = end;
		this.polygons = polygons;
		this.analyzer = analyzer;
	}

	@Override
	public Void call() throws Exception {
		for (int i = start; i < end; i++) {
			Polygon tr = polygons.get(i);
			for (int j = i + 1; j < polygons.size(); j++) {
				Polygon n = polygons.get(j);
				n.addNeighbor(tr);
			}
		}
		analyzer.polygonsElaborated(end - start);
		return null;
	}

}
