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

	class PolygonNormalVectorCombination {
		public Polygon	p;
		public Vector3d	referenceVector;

		/**
		 * @param p2
		 * @param normalVector
		 */
		public PolygonNormalVectorCombination(Polygon p2, Vector3d normalVector) {
			p = p2;
			referenceVector = normalVector;
		}
	}

	/**
	 * Log4J Logger
	 */
	private static Logger							logger				= Logger.getLogger(FlatSurfaceAnalyzer.class);

	/**
	 * First a list of all threads is created which will be executed afterwards with a thread pool
	 */
	private final List<Callable<Void>>				threads				= new LinkedList<Callable<Void>>();

	/**
	 * Allowed tolerance between surface normals considered as equal. Tolerance is in radiant.
	 */
	static final double								TOLERANCE			= 10 * Math.PI / 180;

	/**
	 * List of all polygons which were added already in an annotation. Used during process.
	 */
	final LinkedList<Polygon>						alreadyInAnnotation	= new LinkedList<Polygon>();

	/**
	 * First a list of all polygons in the groups is created and afterwards the analyzing starts
	 */
	private ArrayList<Polygon>						allPolygons;

	/**
	 * Number of polygons already elaborated/processed. Used for indicating current process
	 */
	final AtomicInteger								polygonsElaborated	= new AtomicInteger(0);

	public static ArrayList<FlatSurfaceAnnotation>	testArr				= new ArrayList<FlatSurfaceAnnotation>();

	private static int								test				= 0;

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

		LinkedList<PolygonNormalVectorCombination> queue = new LinkedList<PolygonNormalVectorCombination>();
		if (start.getNeighbors() != null) {
			for (Polygon p : start.getNeighbors())
				queue.add(new PolygonNormalVectorCombination(p, start.getNormalVector()));
		}

		while (!queue.isEmpty()) {
			PolygonNormalVectorCombination pnv = queue.pop();
			visited.add(pnv.p);

			Vector3d inv = (Vector3d) pnv.referenceVector.clone();
			inv.scale(-1);

			// First check if surface normal is exactly the same direction
			boolean isEqual = (pnv.p.getNormalVector().equals(pnv.referenceVector) || pnv.p
					.getNormalVector().equals(inv));

			// if not, use angle between normals
			if (!isEqual) {
				/*
				 * Calculate angle (range: 0 - PI) between the two surface normals. Because due to floating point arithmetic there may be
				 * small errors which can be compensated by checking the angle
				 */
				double radiant = Math.acos(pnv.referenceVector.dot(pnv.p.getNormalVector()));
				isEqual = (radiant < FlatSurfaceAnalyzer.TOLERANCE)
						|| (Math.PI - radiant < FlatSurfaceAnalyzer.TOLERANCE);
			}

			synchronized (alreadyInAnnotation) {

				if (isEqual) {
					if (alreadyInAnnotation.contains(pnv.p)
							&& !annotation.getMesh().getPolygons().contains(pnv.p)) {
						// Polygon has already been added by another thread into another annotation.
						// So combine the two annotations into one.
						synchronized (cas.getAnnotations()) {
							MeshAnnotation ma = cas.findAnnotation(FlatSurfaceAnnotation.class,
									pnv.p);

							if (ma == null) {
								// should never happen
								logger.error("Polygon seems to be already in annotation, but no annotation found for it!");
							} else {
								cas.getAnnotations().remove(annotation);

								synchronized (((FlatSurfaceAnnotation) ma).getMesh().getPolygons()) {
									// Copy all polygons and neighbor polygons from current
									// annotation to found annotation
									((FlatSurfaceAnnotation) ma).getMesh().getPolygons()
											.addAll(annotation.getMesh().getPolygons());
									((FlatSurfaceAnnotation) ma).getNeighborPolygons().addAll(
											annotation.getNeighborPolygons());

									// Remove items from queue which
									for (Iterator<PolygonNormalVectorCombination> it = queue
											.iterator(); it.hasNext();) {
										if (((FlatSurfaceAnnotation) ma).getMesh().getPolygons()
												.contains(it.next().p))
											it.remove();
									}
								}
								annotation = (FlatSurfaceAnnotation) ma;
							}
						}
					} else {
						synchronized (annotation.getMesh().getPolygons()) {
							annotation.getMesh().getPolygons().add(pnv.p);
							alreadyInAnnotation.add(pnv.p);
						}
					}
					for (Polygon a : pnv.p.getNeighbors()) {
						if (annotation.getMesh().getPolygons().contains(a) || visited.contains(a))
							continue;
						boolean found = false;
						for (PolygonNormalVectorCombination pq : queue)
							if (pq.p == a) {
								found = true;
								break;
							}
						if (found)
							continue;
						queue.add(new PolygonNormalVectorCombination(a, pnv.p.getNormalVector()));
					}
				} else {
					annotation.addNeighborPolygon(pnv.p);
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

		// executeInPool(threads);

		for (Callable<Void> c : threads)
			try {
				c.call();
			} catch (Exception e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
		updateProgress();

		/*for (Polygon p : allPolygons) {
			polygonBFS(p, cas);
			polygonsElaborated.incrementAndGet();
		}*/

		// polygonsElaborated.set(allPolygons.size());
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
	final ArrayList<Polygon>			polygons;

	/**
	 * The parent analyzer. Used to update progress.
	 */
	private final FlatSurfaceAnalyzer	analyzer;

	/**
	 * the cas which is being analyzed
	 */
	private final MeshCas				cas;

	public static int					cnt	= 0;

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