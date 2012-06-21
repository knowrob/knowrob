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
import edu.tum.cs.vis.model.util.Triangle;

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
	 * Number of currently processed triangles. Used for progress status.
	 */
	private final AtomicInteger			trianglesElaborated	= new AtomicInteger(0);

	/**
	 * When calling <code>process</code> all triangles of the group and its children are collected
	 * in this list to process them afterwards.
	 */
	private ArrayList<Triangle>			allTriangles;

	@Override
	public Logger getLogger() {
		return logger;
	}

	@Override
	public String getName() {
		return "Neighbor";
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
	 * Process a mesh which contains triangles and find the neighbors for each triangle.
	 * 
	 * @param m
	 *            Mesh to process
	 */
	void processMesh(final Mesh m) {
		if (m.getTriangles().size() == 0)
			return;

		allTriangles.addAll(m.getTriangles());

	}

	@Override
	public void processStart(MeshCas cas) {

		trianglesElaborated.set(0);
		allTriangles = new ArrayList<Triangle>();
		processGroup(cas.getGroup());

		logger.debug("Number of triangles: " + allTriangles.size());

		int startIdx = 0;
		int interval = 100;

		do {
			threads.add(new NeighborAnalyzerThread(startIdx, Math.min(startIdx + interval,
					allTriangles.size()), allTriangles, this));
			startIdx += interval;
		} while (startIdx < allTriangles.size());

		executeInPool(threads);
	}

	/**
	 * Called from the worker threads to update current progress cnt will be added to
	 * trianglesElaborated
	 * 
	 * @param cnt
	 *            number of elaborated triangles.
	 */
	void trianglesElaborated(int cnt) {
		trianglesElaborated.addAndGet(cnt);
	}

	@Override
	public void updateProgress() {
		if (allTriangles != null)
			setProgress((float) trianglesElaborated.get() / (float) allTriangles.size() * 100.0f);
	}
}

/**
 * Worker thread for elaborating a part of the triangle list in a thread pool.
 * 
 * @author Stefan Profanter
 * 
 */
class NeighborAnalyzerThread implements Callable<Void> {

	/**
	 * Index of first triangle in list to elaborate
	 */
	final int						start;
	/**
	 * All triangle in the list from start to < end will be elaborated
	 */
	final int						end;

	/**
	 * List of all triangles. <code>start</code> and <code>end</code> are the indices which indicate
	 * the range to elaborate.
	 */
	final ArrayList<Triangle>		triangles;

	/**
	 * The parent analyzer. Used to update progress.
	 */
	private final NeighborAnalyzer	analyzer;

	public static int				elab	= 0;

	/**
	 * Default constructor.
	 * 
	 * @param start
	 *            Start index in triangles. Where to start elaboration.
	 * @param end
	 *            End index in triangles. Where to end elaboration.
	 * @param triangles
	 *            List of all triangles.
	 * @param analyzer
	 *            parent analyzer used to update progress.
	 */
	public NeighborAnalyzerThread(int start, int end, ArrayList<Triangle> triangles,
			NeighborAnalyzer analyzer) {
		this.start = start;
		this.end = end;
		this.triangles = triangles;
		this.analyzer = analyzer;
	}

	@Override
	public Void call() throws Exception {
		for (int i = start; i < end; i++) {
			Triangle tr = triangles.get(i);
			for (int j = i + 1; j < triangles.size(); j++) {
				Triangle n = triangles.get(j);
				n.addNeighbor(tr);
			}
		}
		analyzer.trianglesElaborated(end - start);
		elab += (end - start);
		return null;
	}

}
