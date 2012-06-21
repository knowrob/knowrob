/*******************************************************************************
 * Copyright (c) 2012 Stefan Profanter. All rights reserved. This program and the accompanying
 * materials are made available under the terms of the GNU Public License v3.0 which accompanies
 * this distribution, and is available at http://www.gnu.org/licenses/gpl.html
 * 
 * Contributors: Stefan Profanter - initial API and implementation, Year: 2012
 ******************************************************************************/
package edu.tum.cs.vis.model.uima.analyzer;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.List;
import java.util.concurrent.Callable;
import java.util.concurrent.atomic.AtomicInteger;

import org.apache.log4j.Logger;

import edu.tum.cs.vis.model.uima.annotation.DrawableAnnotation;
import edu.tum.cs.vis.model.uima.annotation.FlatSurfaceAnnotation;
import edu.tum.cs.vis.model.uima.cas.MeshCas;
import edu.tum.cs.vis.model.util.Group;
import edu.tum.cs.vis.model.util.Mesh;
import edu.tum.cs.vis.model.util.Triangle;

/**
 * Takes a MeshCas and searches all flat surfaces by comparing the surface normals. Flat means the
 * surface normals are equal (or exactly the opposite) or within a given tolerance.
 * 
 * @author Stefan Profanter
 * 
 */
public class FlatSurfaceAnalyzer extends MeshAnalyzer {

	/**
	 * Log4J Logger
	 */
	private static Logger				logger		= Logger.getLogger(FlatSurfaceAnalyzer.class);

	/**
	 * First a list of all threads is created which will be executed afterwards with a thread pool
	 */
	private final List<Callable<Void>>	threads		= new LinkedList<Callable<Void>>();

	/**
	 * Allowed tolerance between surface normals considered as equal. Tolerance is in radiant.
	 */
	static final double					TOLERANCE	= 0.01 * Math.PI / 180;

	/**
	 * Do a BFS on the neighbors of the <code>start</code> triangle and find all neighbors with
	 * nearly the same surface normal.
	 * 
	 * @param start
	 *            Triangle with reference normal
	 * @param alreadyInAnnotation
	 *            List of polynom which are already in an annotaion. So they form already a flat
	 *            surface and don't need to be checked again.
	 * @param cas
	 *            Cas to add a found annotation
	 */
	static void triangleBFS(Triangle start, MeshCas cas) {

		FlatSurfaceAnnotation annotation;
		synchronized (cas.getAnnotations()) {
			if (cas.findAnnotation(FlatSurfaceAnnotation.class, start) != null)
				return; // Triangle is already in an annotation

			// else create a new annotation and add it to CAS
			annotation = new FlatSurfaceAnnotation();
			synchronized (annotation) {
				annotation.getMesh().setTextureBasePath(
						cas.getModel().getGroup().getMesh().getTextureBasePath());
				annotation.getMesh().getTriangles().add(start);
			}

			cas.addAnnotation(annotation);
		}

		// List of already visited triangles for BFS
		LinkedList<Triangle> visited = new LinkedList<Triangle>();
		visited.add(start);

		// FIFO queue for triangles to visit for BFS
		LinkedList<Triangle> queue = new LinkedList<Triangle>();
		if (start.getNeighbors() != null) {
			// Add all neighbor triangles to the queue
			queue.addAll(start.getNeighbors());
		}

		while (!queue.isEmpty()) {
			Triangle currNeighborTriangle = queue.pop();
			visited.add(currNeighborTriangle);

			double radiant = 0;// currNeighbor.getDihedralAngle();

			// First check if surface normal is exactly the same direction
			boolean isEqual = (radiant < FlatSurfaceAnalyzer.TOLERANCE)
					|| (Math.PI - radiant < FlatSurfaceAnalyzer.TOLERANCE);

			if (isEqual) {
				synchronized (cas.getAnnotations()) {
					DrawableAnnotation ma = cas.findAnnotation(FlatSurfaceAnnotation.class,
							currNeighborTriangle);
					if (ma == annotation)
						continue;
					if (ma != null) {
						// Triangle is already in another annotation, so combine the two annotations
						// into one
						FlatSurfaceAnnotation fsa = (FlatSurfaceAnnotation) ma;
						synchronized (cas.getAnnotations()) {
							synchronized (ma) {

								cas.getAnnotations().remove(annotation);
								synchronized (fsa.getMesh().getTriangles()) {
									// Copy all triangles and neighbor triangles from current
									// annotation to found annotation
									fsa.getMesh().getTriangles()
											.addAll(annotation.getMesh().getTriangles());
									fsa.getNeighborTriangles().addAll(
											annotation.getNeighborTriangles());

									// Remove items from queue which are already in found annotation
									for (Iterator<Triangle> it = queue.iterator(); it.hasNext();) {
										ArrayList<Triangle> triangles = ((FlatSurfaceAnnotation) ma)
												.getMesh().getTriangles();
										Triangle next = it.next();
										/*if (triangles.contains(next.getTriangle1())
												|| triangles.contains(next.getTriangle2()))
											it.remove();*/
									}
								}
								synchronized (annotation) {
									annotation.setMesh(fsa.getMesh());
								}
								annotation = fsa;
							}
						}
					} else {
						synchronized (annotation.getMesh().getTriangles()) {
							annotation.getMesh().getTriangles().add(currNeighborTriangle);
						}
					}
				}

				// Add all neighbors of current triangle to queue
				for (Triangle a : currNeighborTriangle.getNeighbors()) {
					synchronized (annotation.getMesh()) {
						synchronized (annotation.getMesh().getTriangles()) {

							if (annotation.getMesh().getTriangles().contains(a)
									|| visited.contains(a))
								continue;
						}
					}
					queue.add(a);
				}
			} else {
				annotation.addNeighborTriangles(currNeighborTriangle);
			}
		}
		annotation.setFeatures();
	}

	/**
	 * First a list of all triangles in the groups is created and afterwards the analyzing starts
	 */
	private ArrayList<Triangle>	allTriangles;

	/**
	 * Number of triangles already elaborated/processed. Used for indicating current process
	 */
	final AtomicInteger			trianglesElaborated	= new AtomicInteger(0);

	@Override
	public Logger getLogger() {
		return logger;
	}

	@Override
	public String getName() {
		return "FlatSurface";
	}

	/**
	 * Process the mesh of the group <code>g</code> with <code>processMesh</code> and all child
	 * groups.
	 * 
	 * @param g
	 *            group to process
	 * @param cas
	 *            CAS to add a new annotation if flat surface is found.
	 */
	private void processGroup(Group g, MeshCas cas) {
		processMesh(g.getMesh(), cas);
		for (Group gr : g.getChildren()) {
			processGroup(gr, cas);
		}
	}

	/**
	 * Process all triangles in the given mesh <code>m</code> with <code>triangleBFS</code>
	 * 
	 * @param m
	 *            mesh to process
	 * @param cas
	 *            CAS to add a new annotation if flat surface is found.
	 */
	private void processMesh(Mesh m, final MeshCas cas) {

		if (m.getTriangles().size() == 0)
			return;

		allTriangles.addAll(m.getTriangles());
	}

	@Override
	public void processStart(MeshCas cas) {

		allTriangles = new ArrayList<Triangle>();

		processGroup(cas.getModel().getGroup(), cas);

		int startIdx = 0;
		int interval = 100;

		do {
			threads.add(new FlatSurfaceAnalyzerThread(startIdx, Math.min(startIdx + interval,
					allTriangles.size()), allTriangles, this, cas));
			startIdx += interval;
		} while (startIdx < allTriangles.size());

		// executeInPool(threads);

		for (Callable<Void> c : threads)
			try {
				c.call();
			} catch (Exception e) {
				e.printStackTrace();
			}
		updateProgress();

		// trianglesElaborated.set(allTriangles.size());
		allTriangles = null;

		logger.debug("Number of FlatSurfaceAnnotations: " + cas.getAnnotations().size());

	}

	@Override
	public void updateProgress() {
		if (allTriangles != null && allTriangles.size() > 0) {
			setProgress((float) trianglesElaborated.get() / (float) allTriangles.size() * 100.0f);
		}
	}
}

/**
 * Worker thread for elaborating a part of the triangle list in a thread pool.
 * 
 * @author Stefan Profanter
 * 
 */
class FlatSurfaceAnalyzerThread implements Callable<Void> {

	/**
	 * Index of first triangle in list to elaborate
	 */
	final int							start;
	/**
	 * All triangles in the list from start to < end will be elaborated
	 */
	final int							end;

	/**
	 * List of all triangles. <code>start</code> and <code>end</code> are the indices which indicate
	 * the range to elaborate.
	 */
	final ArrayList<Triangle>			triangles;

	/**
	 * The parent analyzer. Used to update progress.
	 */
	private final FlatSurfaceAnalyzer	analyzer;

	/**
	 * the cas which is being analyzed
	 */
	private final MeshCas				cas;

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
	 * @param cas
	 *            the cas which is being analyzed
	 */
	public FlatSurfaceAnalyzerThread(int start, int end, ArrayList<Triangle> triangles,
			FlatSurfaceAnalyzer analyzer, MeshCas cas) {
		this.start = start;
		this.end = end;
		this.triangles = triangles;
		this.analyzer = analyzer;
		this.cas = cas;
	}

	@Override
	public Void call() throws Exception {
		for (int i = start; i < end; i++) {
			Triangle tr = triangles.get(i);

			FlatSurfaceAnalyzer.triangleBFS(tr, cas);

		}

		analyzer.trianglesElaborated.addAndGet(end - start);

		return null;
	}

}