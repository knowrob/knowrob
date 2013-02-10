/*******************************************************************************
 * Copyright (c) 2012 Stefan Profanter. All rights reserved. This program and the accompanying
 * materials are made available under the terms of the GNU Public License v3.0 which accompanies
 * this distribution, and is available at http://www.gnu.org/licenses/gpl.html
 * 
 * Contributors: Stefan Profanter - initial API and implementation, Year: 2012
 ******************************************************************************/
package edu.tum.cs.vis.model.uima.analyser;

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;
import java.util.concurrent.Callable;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

import org.apache.log4j.Logger;

import edu.tum.cs.ias.knowrob.utils.ThreadPool;
import edu.tum.cs.vis.model.uima.cas.MeshCas;
import edu.tum.cs.vis.model.util.Triangle;

/**
 * Analyzer for a mesh which sets direct neighbors of a triangle.
 * 
 * The neighbor information is used in other analyzers for better performance.
 * 
 * @author Stefan Profanter
 * 
 */
public class NeighborAnalyser extends MeshAnalyser {

	/**
	 * Log4j logger
	 */
	private static Logger	logger				= Logger.getLogger(NeighborAnalyser.class);

	/**
	 * Number of currently processed triangles. Used for progress status.
	 */
	final AtomicInteger		trianglesElaborated	= new AtomicInteger(0);

	/**
	 * When calling <code>process</code> all triangles of the group and its children are collected
	 * in this list to process them afterwards.
	 */
	ArrayList<Triangle>		allTriangles;

	@Override
	public Logger getLogger() {
		return logger;
	}

	@Override
	public String getName() {
		return "Neighbor";
	}

	@Override
	public void processStart(MeshCas cas) {

		trianglesElaborated.set(0);
		allTriangles = cas.getModel().getTriangles();

		final int interval = 100;

		final Lock lock = new ReentrantLock();

		List<Callable<Void>> threads = new LinkedList<Callable<Void>>();

		for (int start = 0; start < allTriangles.size(); start += interval) {
			final int st = start;
			threads.add(new Callable<Void>() {

				@Override
				public Void call() throws Exception {
					int end = Math.min(st + interval, allTriangles.size());
					for (int i = st; i < end; i++) {
						Triangle tr = allTriangles.get(i);
						for (int j = i + 1; j < allTriangles.size(); j++) {
							Triangle n = allTriangles.get(j);
							// check and add triangle as neighbor
							n.addNeighbor(tr, lock);
						}
					}
					trianglesElaborated.addAndGet(end - st);
					return null;
				}

			});
		};

		ThreadPool.executeInPool(threads);
		updateProgress();
	}

	@Override
	public void updateProgress() {
		if (allTriangles != null)
			setProgress((float) trianglesElaborated.get() / (float) allTriangles.size() * 100.0f);
	}
}
