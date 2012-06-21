/*******************************************************************************
 * Copyright (c) 2012 Stefan Profanter. All rights reserved. This program and the accompanying
 * materials are made available under the terms of the GNU Public License v3.0 which accompanies
 * this distribution, and is available at http://www.gnu.org/licenses/gpl.html
 * 
 * Contributors: Stefan Profanter - initial API and implementation, Year: 2012
 ******************************************************************************/
package edu.tum.cs.vis.model.uima.analyzer;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.List;
import java.util.concurrent.Callable;
import java.util.concurrent.atomic.AtomicInteger;

import javax.vecmath.Vector3f;

import org.apache.log4j.Logger;

import edu.tum.cs.uima.Annotation;
import edu.tum.cs.vis.model.uima.annotation.MeshAnnotation;
import edu.tum.cs.vis.model.uima.annotation.PrimitiveAnnotation;
import edu.tum.cs.vis.model.uima.annotation.primitive.ConeAnnotation;
import edu.tum.cs.vis.model.uima.annotation.primitive.PlaneAnnotation;
import edu.tum.cs.vis.model.uima.annotation.primitive.PrimitiveType;
import edu.tum.cs.vis.model.uima.annotation.primitive.SphereAnnotation;
import edu.tum.cs.vis.model.uima.cas.MeshCas;
import edu.tum.cs.vis.model.util.Curvature;
import edu.tum.cs.vis.model.util.ThreadPool;
import edu.tum.cs.vis.model.util.Triangle;
import edu.tum.cs.vis.model.util.Vertex;

/**
 * @author Stefan Profanter
 * 
 */
public class PrimitiveAnalyzer extends MeshAnalyzer {

	/**
	 * Log4J Logger
	 */
	private static Logger	logger			= Logger.getLogger(PrimitiveAnalyzer.class);

	private static double	PLANE_TOLERANCE	= 2f * Math.PI / 180f;

	static void analyzeVertex(Vertex v) {

		Curvature c = v.getCurvature();
		PrimitiveType t = getPrimitiveType(v);
		c.setPrimitiveType(getPrimitiveType(v));
	}

	private static PrimitiveType getPrimitiveType(Vertex v) {

		Curvature c = v.getCurvature();
		if (c.getSaturation() < 0.55)
			return PrimitiveType.PLANE;

		float hue = c.getHue();

		if (hue < 35 * Math.PI / 180 || hue > 230 * Math.PI / 180)
			return PrimitiveType.SPHERE_CONVEX;
		else if (hue >= 35 * Math.PI / 180 && hue < 75 * Math.PI / 180)
			return PrimitiveType.CONE_CONVEX;
		else if (hue >= 75 * Math.PI / 180 && hue < 150 * Math.PI / 180)
			return PrimitiveType.SPHERE_CONCAV;
		else
			// if (hue >= 150*Math.PI/180)
			return PrimitiveType.CONE_CONCAV;
	}

	private static PrimitiveType getTrianglePrimitiveType(Triangle triangle) {
		return getTrianglePrimitiveType(triangle, true);
	}

	private static PrimitiveType getTrianglePrimitiveType(Triangle triangle, boolean checkNeighbors) {
		int planeCnt = 0;
		int sphereConvexCnt = 0;
		int sphereConcavCnt = 0;
		int coneConvexCnt = 0;
		int coneConcavCnt = 0;
		for (Vertex v : triangle.getPosition())
			if (v.getCurvature().getPrimitiveType() == PrimitiveType.PLANE)
				planeCnt++;
			else if (v.getCurvature().getPrimitiveType() == PrimitiveType.SPHERE_CONVEX)
				sphereConvexCnt++;
			else if (v.getCurvature().getPrimitiveType() == PrimitiveType.SPHERE_CONCAV)
				sphereConcavCnt++;
			else if (v.getCurvature().getPrimitiveType() == PrimitiveType.CONE_CONVEX)
				coneConvexCnt++;
			else if (v.getCurvature().getPrimitiveType() == PrimitiveType.CONE_CONCAV)
				coneConcavCnt++;

		if (checkNeighbors) {
			for (Triangle t : triangle.getNeighbors()) {
				PrimitiveType type = getTrianglePrimitiveType(t, false);

				if (type == PrimitiveType.PLANE)
					planeCnt++;
				else if (type == PrimitiveType.SPHERE_CONVEX)
					sphereConvexCnt++;
				else if (type == PrimitiveType.SPHERE_CONCAV)
					sphereConcavCnt++;
				else if (type == PrimitiveType.CONE_CONVEX)
					coneConvexCnt++;
				else if (type == PrimitiveType.CONE_CONCAV)
					coneConcavCnt++;
			}
		}

		int max = Math.max(
				planeCnt,
				Math.max(sphereConvexCnt,
						Math.max(sphereConcavCnt, Math.max(coneConvexCnt, coneConcavCnt))));

		if (max == planeCnt) {
			return PrimitiveType.PLANE;
		} else if (max == sphereConvexCnt) {
			return PrimitiveType.SPHERE_CONVEX;
		} else if (max == sphereConcavCnt) {
			return PrimitiveType.SPHERE_CONCAV;
		} else if (max == coneConvexCnt) {
			return PrimitiveType.CONE_CONVEX;
		} else
			return PrimitiveType.CONE_CONCAV;
	}

	ArrayList<Vertex>	allVertices;

	ArrayList<Triangle>	allTriangles;

	/**
	 * Number of triangles already elaborated/processed. Used for indicating current process
	 */
	final AtomicInteger	itemsElaborated	= new AtomicInteger(0);

	protected void analyzeTriangle(MeshCas cas, Triangle triangle,
			HashSet<Triangle> alreadyInAnnotation) {
		if (alreadyInAnnotation.contains(triangle))
			return;

		PrimitiveAnnotation annotation;
		PrimitiveType type = getTrianglePrimitiveType(triangle);
		if (type == PrimitiveType.PLANE)
			annotation = new PlaneAnnotation();
		else if (type == PrimitiveType.SPHERE_CONCAV || type == PrimitiveType.SPHERE_CONVEX)
			annotation = new SphereAnnotation(type == PrimitiveType.SPHERE_CONCAV);
		else
			annotation = new ConeAnnotation(type == PrimitiveType.CONE_CONCAV);

		annotation.getMesh().getTriangles().add(triangle);
		alreadyInAnnotation.add(triangle);

		synchronized (cas.getAnnotations()) {
			cas.addAnnotation(annotation);
		}

		// List of already visited triangles for BFS
		HashSet<Triangle> visited = new HashSet<Triangle>();
		visited.add(triangle);

		// FIFO queue for triangles to visit for BFS
		LinkedList<Triangle> queue = new LinkedList<Triangle>();
		if (triangle.getNeighbors() != null) {
			// Add all neighbor triangles to the queue
			queue.addAll(triangle.getNeighbors());
		}

		while (!queue.isEmpty()) {
			Triangle currNeighbor = queue.pop();
			visited.add(currNeighbor);
			if (alreadyInAnnotation.contains(currNeighbor))
				continue;

			// First check if surface normal is exactly the same direction
			boolean isEqual = (type == getTrianglePrimitiveType(currNeighbor));

			if (isEqual && type == PrimitiveType.PLANE) {
				isEqual = planeAngleWithinTolerance(triangle.getNormalVector(),
						currNeighbor.getNormalVector());
			}

			if (isEqual) {
				synchronized (annotation.getMesh().getTriangles()) {
					annotation.getMesh().getTriangles().add(currNeighbor);
				}
				alreadyInAnnotation.add(currNeighbor);

				// Add all neighbors of current triangle to queue
				for (Triangle a : currNeighbor.getNeighbors()) {
					synchronized (annotation.getMesh()) {
						synchronized (annotation.getMesh().getTriangles()) {

							if (visited.contains(a)
									|| annotation.getMesh().getTriangles().contains(a))
								continue;
						}
					}
					queue.add(a);
				}
			}
		}

	}

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

	private boolean isSamePlane(PrimitiveAnnotation a1, PrimitiveAnnotation a2) {
		if (!(a1 instanceof PlaneAnnotation && a2 instanceof PlaneAnnotation))
			return true;
		return planeAngleWithinTolerance(((PlaneAnnotation) a1).getPlaneNormal(),
				((PlaneAnnotation) a2).getPlaneNormal());
	}

	private boolean planeAngleWithinTolerance(Vector3f norm1, Vector3f norm2) {
		double dot = norm1.dot(norm2);
		if (dot > 1.0) // due to floating point arithmetic
			dot = 1.0;

		// Angle between 0 and 180 degree because angle of normal vector may be exactly 0 or 180
		// degree for same plane
		double angle = Math.acos(dot) + Math.PI * 2;
		angle = angle % Math.PI;

		return (angle <= PLANE_TOLERANCE || angle >= Math.PI - PLANE_TOLERANCE);
	}

	/* (non-Javadoc)
	 * @see edu.tum.cs.vis.model.uima.analyzer.MeshAnalyzer#processStart(edu.tum.cs.vis.model.uima.cas.MeshCas)
	 */
	@Override
	public void processStart(final MeshCas cas) {
		allVertices = cas.getModel().getVertices();
		allTriangles = cas.getModel().getTriangles();

		List<Callable<Void>> threads = new LinkedList<Callable<Void>>();

		final int interval = 500;

		for (int start = 0; start < allVertices.size(); start += interval) {
			final int st = start;
			threads.add(new Callable<Void>() {

				@Override
				public Void call() throws Exception {
					int end = Math.min(st + interval, allVertices.size());
					for (int i = st; i < end; i++) {
						analyzeVertex(allVertices.get(i));
						itemsElaborated.incrementAndGet();
					}
					return null;
				}

			});
		};

		ThreadPool.executeInPool(threads);

		final int intervalTri = 100;

		HashSet<Triangle> alreadyInAnnotation = new HashSet<Triangle>();

		for (int start = 0; start < allTriangles.size(); start += intervalTri) {
			final int st = start;
			/*threads.add(new Callable<Void>() {

				@Override
				public Void call() throws Exception {*/
			int end = Math.min(st + intervalTri, allTriangles.size());
			for (int i = st; i < end; i++) {
				analyzeTriangle(cas, allTriangles.get(i), alreadyInAnnotation);
				itemsElaborated.incrementAndGet();
			}
			/*return null;
			}
			});
			};

			ThreadPool.executeInPool(threads);*/
		};

		// Combine very small annotations with surrounding larger ones
		for (Iterator<Annotation> it = cas.getAnnotations().iterator(); it.hasNext();) {
			Annotation a = it.next();
			if (a instanceof PrimitiveAnnotation) {
				PrimitiveAnnotation pa = (PrimitiveAnnotation) a;

				if (pa.getArea() == 0) {
					synchronized (cas.getAnnotations()) {
						it.remove();
					}
					continue;
				}

				HashSet<MeshAnnotation> neighborAnnotations = new HashSet<MeshAnnotation>();
				neighborAnnotations = pa.getNeighborAnnotations(cas, PrimitiveAnnotation.class);
				for (MeshAnnotation ma : neighborAnnotations) {
					PrimitiveAnnotation a1 = (PrimitiveAnnotation) ma;

					if (!isSamePlane(a1, pa))
						continue;

					float percentage = pa.getArea() / a1.getArea();

					// If annotation is smaller than 5% of the area of the surrounding annotation,
					// combine both into one
					if (percentage < 0.05f) {
						synchronized (cas.getAnnotations()) {
							it.remove();
						}

						a1.getMesh().getTriangles().addAll(pa.getMesh().getTriangles());
						a1.updateAnnotationArea();
						break;
					}
				}
			}
		}

		threads.add(new Callable<Void>() {

			@Override
			public Void call() throws Exception {

				for (Iterator<Annotation> it = cas.getAnnotations().iterator(); it.hasNext();) {
					Annotation a = it.next();
					if (a instanceof PrimitiveAnnotation) {
						((PrimitiveAnnotation) a).fit();
					}
				}
				return null;
			}
		});

		ThreadPool.executeInPool(threads);
		itemsElaborated.incrementAndGet();

	}

	/* (non-Javadoc)
	 * @see edu.tum.cs.vis.model.uima.analyzer.MeshAnalyzer#updateProgress()
	 */
	@Override
	public void updateProgress() {
		if (allVertices != null && allVertices.size() > 0) {
			setProgress(itemsElaborated.get()
					/ (float) (allVertices.size() + allTriangles.size() + 1) * 100.0f);
		}

	}

}
