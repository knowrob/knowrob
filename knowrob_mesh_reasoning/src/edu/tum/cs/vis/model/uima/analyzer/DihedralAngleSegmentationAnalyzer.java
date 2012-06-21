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

import edu.tum.cs.vis.model.uima.annotation.DihedralAngleSegmentationAnnotation;
import edu.tum.cs.vis.model.uima.annotation.MeshAnnotation;
import edu.tum.cs.vis.model.uima.cas.MeshCas;
import edu.tum.cs.vis.model.util.Group;
import edu.tum.cs.vis.model.util.Mesh;
import edu.tum.cs.vis.model.util.Triangle;
import edu.tum.cs.vis.model.util.TriangleNeighbor;

public class DihedralAngleSegmentationAnalyzer extends MeshAnalyzer {
	/**
	 * Log4J Logger
	 */
	private static Logger	logger		= Logger.getLogger(DihedralAngleSegmentationAnalyzer.class);

	/**
	 * Allowed tolerance between dihedral angles considered as equal. Tolerance is in radiant.
	 */
	static final double		TOLERANCE	= 2.00 * Math.PI / 180;

	/**
	 * Maximum angle for which two triangles are considered as in the same segment.
	 */
	static final double		MAX_ANGLE	= 45.0 * Math.PI / 180;

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
		DihedralAngleSegmentationAnnotation annotation;
		synchronized (cas.getAnnotations()) {
			if (cas.findAnnotation(DihedralAngleSegmentationAnnotation.class, start) != null)
				return; // Triangle is already in an annotation

			// else create a new annotation and add it to CAS
			annotation = new DihedralAngleSegmentationAnnotation();
			synchronized (annotation) {
				annotation.getMesh().setTextureBasePath(
						cas.getGroup().getMesh().getTextureBasePath());
				annotation.getMesh().getTriangles().add(start);
			}

			cas.addAnnotation(annotation);
		}

		if (start.getNeighbors() == null || start.getNeighbors().size() == 0)
			return;

		// List of already visited triangles for BFS
		LinkedList<Triangle> visited = new LinkedList<Triangle>();
		visited.add(start);

		// FIFO queue for triangles to visit for BFS
		LinkedList<TriangleNeighbor> queue = new LinkedList<TriangleNeighbor>();
		// Add all neighbor triangles to the queue
		queue.addAll(start.getNeighbors());

		double currentDihedral = Double.MAX_VALUE;

		// Get the smallest dihedral angle between start and the neighbors
		for (TriangleNeighbor n : start.getNeighbors()) {
			currentDihedral = Math.min(currentDihedral, n.getDihedralAngle());
		}

		if (currentDihedral > MAX_ANGLE)
			return;

		while (!queue.isEmpty()) {
			TriangleNeighbor currNeighbor = queue.pop();
			Triangle neighbor;
			if (visited.contains(currNeighbor.getTriangle1()))
				neighbor = currNeighbor.getTriangle2();
			else
				neighbor = currNeighbor.getTriangle1();
			visited.add(neighbor);

			double radiant = currNeighbor.getDihedralAngle();

			if (Math.abs(radiant - currentDihedral) <= TOLERANCE) {
				synchronized (cas.getAnnotations()) {
					MeshAnnotation ma = cas.findAnnotation(
							DihedralAngleSegmentationAnnotation.class, neighbor);
					if (ma == annotation)
						continue;
					if (ma != null) {
						// Triangle is already in another annotation, so combine the two annotations
						// into one
						DihedralAngleSegmentationAnnotation fsa = (DihedralAngleSegmentationAnnotation) ma;
						synchronized (cas.getAnnotations()) {
							synchronized (ma) {

								cas.getAnnotations().remove(annotation);
								synchronized (fsa.getMesh().getTriangles()) {
									// Copy all triangles and neighbor triangles from current
									// annotation to found annotation
									fsa.getMesh().getTriangles()
											.addAll(annotation.getMesh().getTriangles());

									// Remove items from queue which are already in found annotation
									for (Iterator<TriangleNeighbor> it = queue.iterator(); it
											.hasNext();) {
										ArrayList<Triangle> triangles = ((DihedralAngleSegmentationAnnotation) ma)
												.getMesh().getTriangles();
										TriangleNeighbor next = it.next();
										if (triangles.contains(next.getTriangle1())
												|| triangles.contains(next.getTriangle2()))
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

				// Add all neighbors of current triangle to queue
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
			}
		}
		// annotation.setFeatures();
	}

	/**
	 * First a list of all threads is created which will be executed afterwards with a thread pool
	 */
	private final List<Callable<Void>>	threads				= new LinkedList<Callable<Void>>();

	/**
	 * First a list of all triangles in the groups is created and afterwards the analyzing starts
	 */
	private ArrayList<Triangle>			allTriangles;

	/**
	 * Number of triangles already elaborated/processed. Used for indicating current process
	 */
	final AtomicInteger					trianglesElaborated	= new AtomicInteger(0);

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
		return "DihedralAngleSeg.";
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
	private void processGroup(ArrayList<Triangle> allTriangles, Group g, MeshCas cas) {
		processMesh(allTriangles, g.getMesh(), cas);
		for (Group gr : g.getChildren()) {
			processGroup(allTriangles, gr, cas);
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
	private void processMesh(ArrayList<Triangle> allTriangles, Mesh m, final MeshCas cas) {

		if (m.getTriangles().size() == 0)
			return;

		allTriangles.addAll(m.getTriangles());
	}

	/* (non-Javadoc)
	 * @see edu.tum.cs.vis.model.uima.analyzer.MeshAnalyzer#processStart(edu.tum.cs.vis.model.uima.cas.MeshCas)
	 */
	@Override
	public void processStart(MeshCas cas) {
		allTriangles = new ArrayList<Triangle>();

		processGroup(allTriangles, cas.getGroup(), cas);

		for (int i = 0; i < allTriangles.size(); i++) {
			triangleBFS(allTriangles.get(i), cas);
			trianglesElaborated.set(i);
		}
		trianglesElaborated.set(allTriangles.size());

		logger.debug("Number of DihedralAngleSegmentations: " + cas.getAnnotations().size());

	}

	/* (non-Javadoc)
	 * @see edu.tum.cs.vis.model.uima.analyzer.MeshAnalyzer#updateProgress()
	 */
	@Override
	public void updateProgress() {
		if (allTriangles != null && allTriangles.size() > 0) {
			setProgress((float) trianglesElaborated.get() / (float) allTriangles.size() * 100.0f);
		}
	}

}
