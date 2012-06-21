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

import edu.tum.cs.vis.model.uima.annotation.FlatSurfaceAnnotation;
import edu.tum.cs.vis.model.uima.annotation.MeshAnnotation;
import edu.tum.cs.vis.model.uima.cas.MeshCas;
import edu.tum.cs.vis.model.util.Group;
import edu.tum.cs.vis.model.util.Mesh;
import edu.tum.cs.vis.model.util.Triangle;
import edu.tum.cs.vis.model.util.TriangleNeighbor;

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
	 * Do a BFS on the neighbors of the <code>start</code> polygon and find all neighbors with
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
	static void polygonBFS(Triangle start, MeshCas cas) {

		FlatSurfaceAnnotation annotation;
		synchronized (cas.getAnnotations()) {
			if (cas.findAnnotation(FlatSurfaceAnnotation.class, start) != null)
				return; // Triangle is already in an annotation

			// else create a new annotation and add it to CAS
			annotation = new FlatSurfaceAnnotation();
			synchronized (annotation) {
				annotation.getMesh().setTextureBasePath(
						cas.getGroup().getMesh().getTextureBasePath());
				annotation.getMesh().getTriangles().add(start);
			}

			cas.addAnnotation(annotation);
		}

		// List of already visited polygons for BFS
		LinkedList<Triangle> visited = new LinkedList<Triangle>();
		visited.add(start);

		// FIFO queue for polygons to visit for BFS
		LinkedList<TriangleNeighbor> queue = new LinkedList<TriangleNeighbor>();
		if (start.getNeighbors() != null) {
			// Add all neighbor polygons to the queue
			queue.addAll(start.getNeighbors());
		}

		while (!queue.isEmpty()) {
			TriangleNeighbor currNeighbor = queue.pop();
			Triangle neighbor;
			if (visited.contains(currNeighbor.getPolygon1()))
				neighbor = currNeighbor.getPolygon2();

			else
				neighbor = currNeighbor.getPolygon1();
			visited.add(neighbor);

			double radiant = currNeighbor.getDihedralAngle();

			// First check if surface normal is exactly the same direction
			boolean isEqual = (radiant < FlatSurfaceAnalyzer.TOLERANCE)
					|| (Math.PI - radiant < FlatSurfaceAnalyzer.TOLERANCE);

			if (isEqual) {
				synchronized (cas.getAnnotations()) {
					MeshAnnotation ma = cas.findAnnotation(FlatSurfaceAnnotation.class, neighbor);
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
									// Copy all polygons and neighbor polygons from current
									// annotation to found annotation
									fsa.getMesh().getTriangles()
											.addAll(annotation.getMesh().getTriangles());
									fsa.getNeighborPolygons().addAll(
											annotation.getNeighborPolygons());

									// Remove items from queue which are already in found annotation
									for (Iterator<TriangleNeighbor> it = queue.iterator(); it
											.hasNext();) {
										ArrayList<Triangle> triangles = ((FlatSurfaceAnnotation) ma)
												.getMesh().getTriangles();
										TriangleNeighbor next = it.next();
										if (triangles.contains(next.getPolygon1())
												|| triangles.contains(next.getPolygon2()))
											it.remove();
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
							annotation.getMesh().getTriangles().add(neighbor);
						}
					}
				}

				// Add all neighbors of current polygon to queue
				for (TriangleNeighbor a : neighbor.getNeighbors()) {
					Triangle newTriangle = a.getNeighbor(neighbor);
					synchronized (annotation.getMesh()) {
						synchronized (annotation.getMesh().getTriangles()) {

							if (annotation.getMesh().getTriangles().contains(newTriangle)
									|| visited.contains(newTriangle))
								continue;
						}
					}
					queue.add(a);
				}
			} else {
				annotation.addNeighborPolygon(neighbor);
			}
		}
		annotation.setFeatures();
	}

	/**
	 * First a list of all polygons in the groups is created and afterwards the analyzing starts
	 */
	private ArrayList<Triangle>	allPolygons;

	/**
	 * Number of polygons already elaborated/processed. Used for indicating current process
	 */
	final AtomicInteger			polygonsElaborated	= new AtomicInteger(0);

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
	 * Process all polygons in the given mesh <code>m</code> with <code>polygonBFS</code>
	 * 
	 * @param m
	 *            mesh to process
	 * @param cas
	 *            CAS to add a new annotation if flat surface is found.
	 */
	private void processMesh(Mesh m, final MeshCas cas) {

		if (m.getTriangles().size() == 0)
			return;

		allPolygons.addAll(m.getTriangles());
	}

	@Override
	public void processStart(MeshCas cas) {

		allPolygons = new ArrayList<Triangle>();

		processGroup(cas.getGroup(), cas);

		int startIdx = 0;
		int interval = 100;

		do {
			threads.add(new FlatSurfaceAnalyzerThread(startIdx, Math.min(startIdx + interval,
					allPolygons.size()), allPolygons, this, cas));
			startIdx += interval;
		} while (startIdx < allPolygons.size());

		// executeInPool(threads);

		for (Callable<Void> c : threads)
			try {
				c.call();
			} catch (Exception e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
		updateProgress();

		/*for (Triangle p : allPolygons) {
			polygonBFS(p, cas);
			trianglesElaborated.incrementAndGet();
		}*/

		// trianglesElaborated.set(allPolygons.size());
		allPolygons = null;

		logger.debug("Number of FlatSurfaceAnnotations: " + cas.getAnnotations().size());

	}

	@Override
	public void updateProgress() {
		if (allPolygons != null && allPolygons.size() > 0) {
			setProgress((float) polygonsElaborated.get() / (float) allPolygons.size() * 100.0f);
		}
	}
}

/**
 * Worker thread for elaborating a part of the polygon list in a thread pool.
 * 
 * @author Stefan Profanter
 * 
 */
class FlatSurfaceAnalyzerThread implements Callable<Void> {

	/**
	 * Index of first polygon in list to elaborate
	 */
	final int							start;
	/**
	 * All polygons in the list from start to < end will be elaborated
	 */
	final int							end;

	/**
	 * List of all polygons. <code>start</code> and <code>end</code> are the indices which indicate
	 * the range to elaborate.
	 */
	final ArrayList<Triangle>			polygons;

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
	 *            Start index in polygons. Where to start elaboration.
	 * @param end
	 *            End index in polygons. Where to end elaboration.
	 * @param polygons
	 *            List of all polygons.
	 * @param analyzer
	 *            parent analyzer used to update progress.
	 * @param cas
	 *            the cas which is being analyzed
	 */
	public FlatSurfaceAnalyzerThread(int start, int end, ArrayList<Triangle> polygons,
			FlatSurfaceAnalyzer analyzer, MeshCas cas) {
		this.start = start;
		this.end = end;
		this.polygons = polygons;
		this.analyzer = analyzer;
		this.cas = cas;
	}

	@Override
	public Void call() throws Exception {
		for (int i = start; i < end; i++) {
			Triangle tr = polygons.get(i);

			FlatSurfaceAnalyzer.polygonBFS(tr, cas);

		}

		analyzer.polygonsElaborated.addAndGet(end - start);

		return null;
	}

}