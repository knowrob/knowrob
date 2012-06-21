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

import javax.vecmath.Vector3d;

import org.apache.log4j.Logger;

import edu.tum.cs.vis.model.uima.annotation.FlatSurfaceAnnotation;
import edu.tum.cs.vis.model.uima.annotation.MeshAnnotation;
import edu.tum.cs.vis.model.uima.cas.MeshCas;
import edu.tum.cs.vis.model.util.Group;
import edu.tum.cs.vis.model.util.Mesh;
import edu.tum.cs.vis.model.util.Polygon;

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
	private static Logger				logger				= Logger.getLogger(FlatSurfaceAnalyzer.class);

	/**
	 * First a list of all threads is created which will be executed afterwards with a thread pool
	 */
	private final List<Callable<Void>>	threads				= new LinkedList<Callable<Void>>();

	/**
	 * Allowed tolerance between surface normals considered as equal. Tolerance is in radiant.
	 */
	static final double					TOLERANCE			= 1.0 * Math.PI / 180;

	/**
	 * List of all polygons which were added already in an annotation. Used during process.
	 */
	final LinkedList<Polygon>			alreadyInAnnotation	= new LinkedList<Polygon>();

	/**
	 * First a list of all polygons in the groups is created and afterwards the analyzing starts
	 */
	private ArrayList<Polygon>			allPolygons;

	/**
	 * Number of polygons already elaborated/processed. Used for indicating current process
	 */
	final AtomicInteger					polygonsElaborated	= new AtomicInteger(0);

	@Override
	public Logger getLogger() {
		return logger;
	}

	@Override
	public String getName() {
		return "FlatSurface";
	}

	/**
	 * Do a BFS on the neighbors of the <code>start</code> polygon and find all neighbors with
	 * nearly the same surface normal.
	 * 
	 * @param start
	 *            Polygon with reference normal
	 * @param alreadyInAnnotation
	 *            List of polynom which are already in a annotaion. So they form already a flat
	 *            surface and don't need to be checked again.
	 * @param cas
	 *            Cas to add a found annotation
	 */
	void polygonBFS(Polygon start, MeshCas cas) {
		FlatSurfaceAnnotation annotation = new FlatSurfaceAnnotation();
		synchronized (alreadyInAnnotation) {
			if (alreadyInAnnotation.contains(start))
				return;

			synchronized (cas.getAnnotations()) {
				cas.addAnnotation(annotation);
				alreadyInAnnotation.add(start);
			}
		}

		LinkedList<Polygon> visited = new LinkedList<Polygon>();

		annotation.getMesh().setTextureBasePath(cas.getGroup().getMesh().getTextureBasePath());
		synchronized (annotation.getMesh().getPolygons()) {
			annotation.getMesh().getPolygons().add(start);
		}

		visited.add(start);

		LinkedList<Polygon> queue = new LinkedList<Polygon>();
		if (start.getNeighbors() != null)
			queue.addAll(start.getNeighbors());

		Vector3d inv = (Vector3d) start.getNormalVector().clone();
		inv.scale(-1);

		while (!queue.isEmpty()) {
			Polygon p = queue.pop();
			visited.add(p);

			// First check if sufrace normal is exactly the same direction
			boolean isEqual = (p.getNormalVector().equals(start.getNormalVector()) || p
					.getNormalVector().equals(inv));

			// if not, use angle between normals
			if (!isEqual) {
				/*
				 * Calculate angle (range: 0 - PI) between the two surface normals. Because due to floating point arithmetic there may be
				 * small errors which can be compensated by checking the angle
				 */
				double radiant = Math.acos(start.getNormalVector().dot(p.getNormalVector()));
				isEqual = (radiant < FlatSurfaceAnalyzer.TOLERANCE)
						|| (Math.PI - radiant < FlatSurfaceAnalyzer.TOLERANCE);
			}

			synchronized (alreadyInAnnotation) {

				if (isEqual) {
					if (alreadyInAnnotation.contains(p)
							&& !annotation.getMesh().getPolygons().contains(p)) {
						// Polygon has already been added by another thread into another annotation.
						// So combine the two annotations into one.
						synchronized (cas.getAnnotations()) {
							MeshAnnotation ma = cas.findAnnotation(FlatSurfaceAnnotation.class, p);
							if (ma == null) {
								// should never happen
								logger.error("Polygon seems to be already in annotation, but no annotation found for it!");
							} else {
								cas.getAnnotations().remove(annotation);

								synchronized (((FlatSurfaceAnnotation) ma).getMesh().getPolygons()) {
									((FlatSurfaceAnnotation) ma).getMesh().getPolygons()
											.addAll(annotation.getMesh().getPolygons());
								}
								annotation = (FlatSurfaceAnnotation) ma;
							}
						}
					} else {
						synchronized (annotation.getMesh().getPolygons()) {
							annotation.getMesh().getPolygons().add(p);
							alreadyInAnnotation.add(p);
						}
					}
					for (Polygon a : p.getNeighbors()) {
						if (annotation.getMesh().getPolygons().contains(a) || visited.contains(a)
								|| queue.contains(a))
							continue;
						queue.add(a);
					}
				}
			}

		}
		annotation.setFeatures();
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

		if (m.getPolygons().size() == 0)
			return;

		allPolygons.addAll(m.getPolygons());
	}

	@Override
	public void processStart(MeshCas cas) {
		FlatSurfaceAnnotation fsa = new FlatSurfaceAnnotation();
		fsa.setMesh(cas.getGroup().getMesh().getInitializedChildMesh());

		allPolygons = new ArrayList<Polygon>();

		processGroup(cas.getGroup(), cas);

		alreadyInAnnotation.clear();

		int startIdx = 0;
		int interval = 100;

		do {
			threads.add(new FlatSurfaceAnalyzerThread(startIdx, Math.min(startIdx + interval,
					allPolygons.size()), allPolygons, this, cas));
			startIdx += interval;
		} while (startIdx < allPolygons.size());

		executeInPool(threads);

		/*for (Polygon p : allPolygons) {
			polygonBFS(p, cas);
			polygonsElaborated.incrementAndGet();
		}*/

		logger.debug("Number of FlatSurfaceAnnotations: " + cas.getAnnotations().size());

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
	final ArrayList<Polygon>			polygons;

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
	public FlatSurfaceAnalyzerThread(int start, int end, ArrayList<Polygon> polygons,
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
			Polygon tr = polygons.get(i);

			analyzer.polygonBFS(tr, cas);

		}

		analyzer.polygonsElaborated.addAndGet(end - start);
		return null;
	}

}