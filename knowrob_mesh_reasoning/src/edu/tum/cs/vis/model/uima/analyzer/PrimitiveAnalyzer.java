/*******************************************************************************
 * Copyright (c) 2012 Stefan Profanter. All rights reserved. This program and the accompanying
 * materials are made available under the terms of the GNU Public License v3.0 which accompanies
 * this distribution, and is available at http://www.gnu.org/licenses/gpl.html
 * 
 * Contributors: Stefan Profanter - initial API and implementation, Year: 2012
 ******************************************************************************/
package edu.tum.cs.vis.model.uima.analyzer;

import java.util.ArrayList;
import java.util.concurrent.atomic.AtomicInteger;

import javax.vecmath.Vector3d;

import org.apache.log4j.Logger;

import edu.tum.cs.vis.model.uima.annotation.CurvatureAnnotation;
import edu.tum.cs.vis.model.uima.annotation.DrawableAnnotation;
import edu.tum.cs.vis.model.uima.annotation.PrimitiveAnnotation;
import edu.tum.cs.vis.model.uima.annotation.primitive.ConeAnnotation;
import edu.tum.cs.vis.model.uima.annotation.primitive.PlaneAnnotation;
import edu.tum.cs.vis.model.uima.annotation.primitive.PrimitiveType;
import edu.tum.cs.vis.model.uima.annotation.primitive.SphereAnnotation;
import edu.tum.cs.vis.model.uima.cas.MeshCas;
import edu.tum.cs.vis.model.util.Triangle;
import edu.tum.cs.vis.model.view.MeshReasoningView;

/**
 * @author Stefan Profanter
 * 
 */
public class PrimitiveAnalyzer extends MeshAnalyzer {

	/**
	 * Log4J Logger
	 */
	private static Logger	logger					= Logger.getLogger(PrimitiveAnalyzer.class);

	private static float	EPSILON_SAME_CURVATURE	= 1f;
	private static float	EPSILON_IS_PLANE		= 1.5f;

	/**
	 * @param triangle
	 * @param cas
	 */
	private static void analyzeTriangle(Triangle start, MeshCas cas) {

		PrimitiveAnnotation annotation = null;
		PrimitiveType type;

		synchronized (cas.getAnnotations()) {
			if (cas.findAnnotation(PrimitiveAnnotation.class, start) != null)
				return; // Triangle is already in an annotation

			// else create a new annotation and add it to CAS

			type = getPrimitiveType(start);

			if (type == PrimitiveType.SPHERE)
				annotation = new SphereAnnotation();
			else if (type == PrimitiveType.PLANE)
				annotation = new PlaneAnnotation();
			else if (type == PrimitiveType.CONE)
				annotation = new ConeAnnotation();
			else {
				logger.error("Unknown type!");
				return;
			}
			synchronized (annotation) {
				annotation.getMesh().setTextureBasePath(
						cas.getModel().getGroup().getMesh().getTextureBasePath());
				annotation.getMesh().getTriangles().add(start);
			}

			cas.addAnnotation(annotation);
		}

		// List of already visited triangles for BFS
		/*	LinkedList<Triangle> visited = new LinkedList<Triangle>();
			visited.add(start);

			// FIFO queue for triangles to visit for BFS
			LinkedList<TriangleNeighbor> queue = new LinkedList<TriangleNeighbor>();
			if (start.getNeighbors() != null) {
				// Add all neighbor triangles to the queue
				queue.addAll(start.getNeighbors());
			}

			while (!queue.isEmpty()) {
				TriangleNeighbor currNeighbor = queue.pop();
				Triangle currNeighborTriangle;
				if (visited.contains(currNeighbor.getTriangle1()))
					currNeighborTriangle = currNeighbor.getTriangle2();

				else
					currNeighborTriangle = currNeighbor.getTriangle1();
				visited.add(currNeighborTriangle);

				double radiant = currNeighbor.getDihedralAngle();

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
										for (Iterator<TriangleNeighbor> it = queue.iterator(); it
												.hasNext();) {
											ArrayList<Triangle> triangles = ((FlatSurfaceAnnotation) ma)
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
								annotation.getMesh().getTriangles().add(currNeighborTriangle);
							}
						}
					}

					// Add all neighbors of current triangle to queue
					for (TriangleNeighbor a : currNeighborTriangle.getNeighbors()) {
						Triangle newTriangle = a.getNeighbor(currNeighborTriangle);
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
					annotation.addNeighborTriangles(currNeighborTriangle);
				}
			}
			annotation.setFeatures();*/
	}

	private static PrimitiveType getPrimitiveType(Triangle t) {
		CurvatureAnnotation annotation = null;
		for (DrawableAnnotation a : t.getAnnotations())
			if (a instanceof CurvatureAnnotation)
				annotation = (CurvatureAnnotation) a;
		if (annotation == null)
			return null;

		Vector3d max = annotation.getkMax();
		Vector3d min = annotation.getkMin();
		double kMax = max.length();
		double kMin = min.length();

		if (Math.abs(kMax - kMin) < EPSILON_SAME_CURVATURE) {
			// it is either a plane or sphere
			if (Math.abs((kMax + kMin) / 2) < EPSILON_IS_PLANE)
				return PrimitiveType.PLANE;

			return PrimitiveType.SPHERE;
		}

		// It is a cylinder or cone
		// A Cylinder is a special form of a cone.
		return PrimitiveType.CONE;
	}

	/**
	 * First a list of all triangles in the groups is created and afterwards the analyzing starts
	 */
	private ArrayList<Triangle>	allTriangles;

	/**
	 * Number of triangles already elaborated/processed. Used for indicating current process
	 */
	final AtomicInteger			trianglesElaborated	= new AtomicInteger(0);

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
		return "Primitive";
	}

	/* (non-Javadoc)
	 * @see edu.tum.cs.vis.model.uima.analyzer.MeshAnalyzer#processStart(edu.tum.cs.vis.model.uima.cas.MeshCas)
	 */
	@Override
	public void processStart(MeshCas cas) {
		allTriangles = cas.getModel().getTriangles();

		for (int i = 0; i < allTriangles.size(); i++) {
			MeshReasoningView.test = i;
			analyzeTriangle(allTriangles.get(i), cas);
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
