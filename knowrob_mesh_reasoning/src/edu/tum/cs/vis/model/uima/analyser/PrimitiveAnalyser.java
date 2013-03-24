/*******************************************************************************
 * Copyright (c) 2012 Stefan Profanter. All rights reserved. This program and the accompanying
 * materials are made available under the terms of the GNU Public License v3.0 which accompanies
 * this distribution, and is available at http://www.gnu.org/licenses/gpl.html
 * 
 * Contributors: Stefan Profanter - initial API and implementation, Year: 2012
 ******************************************************************************/
package edu.tum.cs.vis.model.uima.analyser;

import java.util.Collection;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.List;
import java.util.Set;
import java.util.concurrent.Callable;
import java.util.concurrent.atomic.AtomicInteger;

import javax.vecmath.Vector3f;

import org.apache.log4j.Logger;

import edu.tum.cs.ias.knowrob.utils.ThreadPool;
import edu.tum.cs.uima.Annotation;
import edu.tum.cs.vis.model.uima.annotation.PrimitiveAnnotation;
import edu.tum.cs.vis.model.uima.annotation.primitive.ConeAnnotation;
import edu.tum.cs.vis.model.uima.annotation.primitive.PlaneAnnotation;
import edu.tum.cs.vis.model.uima.annotation.primitive.PrimitiveType;
import edu.tum.cs.vis.model.uima.annotation.primitive.SphereAnnotation;
import edu.tum.cs.vis.model.uima.cas.MeshCas;
import edu.tum.cs.vis.model.util.Curvature;
import edu.tum.cs.vis.model.util.Triangle;
import edu.tum.cs.vis.model.util.Vertex;

/**
 * 
 * Mesh analyzer to assign each triangle the corresponding primitive type according to its curvature
 * properties.
 * 
 * 
 * @author Stefan Profanter
 * 
 */
public class PrimitiveAnalyser extends MeshAnalyser {

	/**
	 * Log4J Logger
	 */
	private static Logger	logger			= Logger.getLogger(PrimitiveAnalyser.class);

	/**
	 * Tolerance in radiant between two surface normals of triangles to connect them as a single
	 * plane
	 */
	private static double	PLANE_TOLERANCE	= 2f * Math.PI / 180f;

	/**
	 * Set primitive type of vertex
	 * 
	 * @param curvatures
	 *            list of all curvature values for vertices
	 * @param v
	 *            vertex to analyze
	 */
	static void analyseVertex(HashMap<Vertex, Curvature> curvatures, Vertex v) {

		Curvature c = curvatures.get(v);
		c.setPrimitiveType(getPrimitiveType(curvatures, v));
	}

	@SuppressWarnings("rawtypes")
	private static void combineSameNeighboringAnnotations(MeshCas cas,
			List<Annotation> annotations, Set<Annotation> toRefit) {
		// Combine neighboring annotations which were previously divided by smaller annotations and
		// are now neighbors
		Set<Annotation> toRemove = new HashSet<Annotation>();
		for (Iterator<Annotation> it = annotations.iterator(); it.hasNext();) {
			Annotation a = it.next();
			if (a instanceof PrimitiveAnnotation) {
				PrimitiveAnnotation pa = (PrimitiveAnnotation) a;

				@SuppressWarnings("unchecked")
				Set<PrimitiveAnnotation> neighborAnnotations = pa.getNeighborAnnotations(cas,
						pa.getClass());
				for (PrimitiveAnnotation a1 : neighborAnnotations) {
					if (pa instanceof ConeAnnotation
							&& ((ConeAnnotation) pa).isConcave() != ((ConeAnnotation) a1)
									.isConcave()) {
						continue;
					} else if (pa instanceof SphereAnnotation
							&& ((SphereAnnotation) pa).isConcave() != ((SphereAnnotation) a1)
									.isConcave()) {
						continue;
					} else if (pa instanceof PlaneAnnotation
							&& Math.acos(((PlaneAnnotation) pa).getPlaneNormal().dot(
									((PlaneAnnotation) a1).getPlaneNormal())) > 45.0 * Math.PI / 180.0) {
						// Two planes, but angle bigger than 45 degree
						continue;
					}

					// Found same annotations
					toRemove.add(a);

					synchronized (a1.getMesh().getTriangles()) {
						a1.getMesh().getTriangles().addAll(pa.getMesh().getTriangles());
					}
					if (toRefit != null)
						toRefit.add(a1);
					break;
				}
			}
		}

		synchronized (annotations) {
			annotations.removeAll(toRemove);
		}
	}

	@SuppressWarnings("rawtypes")
	private static void combineSmallAnnotations(MeshCas cas) {
		// Combine very small annotations with surrounding larger ones and merge/remove invalid ones
		Set<Annotation> toRemove = new HashSet<Annotation>();
		for (Iterator<Annotation> it = cas.getAnnotations().iterator(); it.hasNext();) {
			Annotation a = it.next();
			if (a instanceof PrimitiveAnnotation) {
				PrimitiveAnnotation pa = (PrimitiveAnnotation) a;

				// zero area, remove
				if (pa.getArea() == 0) {
					toRemove.add(a);
					continue;
				}

				@SuppressWarnings("unchecked")
				Set<PrimitiveAnnotation> neighborAnnotations = pa.getNeighborAnnotations(cas,
						PrimitiveAnnotation.class);

				// check for cone annotation which has less than 2 triangles
				if (neighborAnnotations.size() > 0 && a instanceof ConeAnnotation
						&& ((PrimitiveAnnotation) a).getMesh().getTriangles().size() < 2) {
					toRemove.add(a);

					PrimitiveAnnotation a1 = neighborAnnotations.iterator().next();

					synchronized (a1.getMesh().getTriangles()) {
						a1.getMesh().getTriangles().addAll(pa.getMesh().getTriangles());
					}
					continue;
				}

				// merge small ones
				/*
				for (PrimitiveAnnotation a1 : neighborAnnotations) {

					if (!isSamePlane(a1, pa))
						continue;

					if (!(pa instanceof PlaneAnnotation)
							&& pa.getPrimitiveArea() / pa.getArea() > 0.8) {
						continue;
					}

					float percentage = pa.getArea() / a1.getArea();

					// If annotation is smaller than 5% of the area of the surrounding annotation,
					// combine both into one
					if (percentage < 0.05f) {
						toRemove.add(a);

						a1.getMesh().getTriangles().addAll(pa.getMesh().getTriangles());
						break;
					}
				}*/
			}
		}
		synchronized (cas.getAnnotations()) {
			cas.getAnnotations().removeAll(toRemove);
		}
	}

	/**
	 * @param annotations
	 * @return
	 */
	@SuppressWarnings("rawtypes")
	private static List<? extends PrimitiveAnnotation> fitAnnotations(
			final Collection<Annotation> annotations) {

		List<Callable<Void>> threads = new LinkedList<Callable<Void>>();
		final List<PrimitiveAnnotation> failedFittings = new LinkedList<PrimitiveAnnotation>();
		for (final Annotation a : annotations) {
			if (a instanceof PrimitiveAnnotation) {
				threads.add(new Callable<Void>() {
					@Override
					public Void call() throws Exception {
						if (!((PrimitiveAnnotation) a).fit()) {
							synchronized (failedFittings) {
								failedFittings.add((PrimitiveAnnotation) a);
							}
						} else
							((PrimitiveAnnotation) a).updateAnnotationArea();
						return null;
					}
				});
			}
		}

		ThreadPool.executeInPool(threads);
		return failedFittings;
	}

	/**
	 * Detect primitive type for vertex by checking curvature properties.
	 * 
	 * @param curvatures
	 *            list of curvatures for vertices
	 * @param v
	 *            vertex to analyze
	 * @return primitive type of vertex
	 */
	private static PrimitiveType getPrimitiveType(HashMap<Vertex, Curvature> curvatures, Vertex v) {

		Curvature c = curvatures.get(v);

		if (c.getSaturation() < 0.20)
			return PrimitiveType.PLANE;

		float hue = c.getHue();

		if (hue < 15 * Math.PI / 180)
			return PrimitiveType.SPHERE_CONVEX;
		else if (hue >= 15 * Math.PI / 180 && hue < 75 * Math.PI / 180)
			return PrimitiveType.CONE_CONVEX;
		else if (hue >= 75 * Math.PI / 180 && hue < 150 * Math.PI / 180
				|| hue >= 230 * Math.PI / 180)
			return PrimitiveType.SPHERE_CONCAVE;
		else
			return PrimitiveType.CONE_CONCAVE;
	}

	/**
	 * Get primitive type for given property counts. The biggest number indicates the primitive type
	 * 
	 * @param planeCnt
	 *            number of plane vertices for triangle
	 * @param sphereConvexCnt
	 *            number of convex sphere vertices for triangle
	 * @param sphereConcaveCnt
	 *            number of concave sphere vertices for triangle
	 * @param coneConvexCnt
	 *            number of convex cone vertices for triangle
	 * @param coneConcaveCnt
	 *            number of concave cone vertices for triangle
	 * @return primitive type indicated by biggest number value
	 */
	private static PrimitiveType getTypeForCounts(int planeCnt, int sphereConvexCnt,
			int sphereConcaveCnt, int coneConvexCnt, int coneConcaveCnt) {
		int max = Math.max(
				planeCnt,
				Math.max(sphereConvexCnt,
						Math.max(sphereConcaveCnt, Math.max(coneConvexCnt, coneConcaveCnt))));

		if (max == planeCnt) {
			return PrimitiveType.PLANE;
		} else if (max == sphereConvexCnt) {
			return PrimitiveType.SPHERE_CONVEX;
		} else if (max == sphereConcaveCnt) {
			return PrimitiveType.SPHERE_CONCAVE;
		} else if (max == coneConvexCnt) {
			return PrimitiveType.CONE_CONVEX;
		} else
			return PrimitiveType.CONE_CONCAVE;
	}

	/**
	 * Check if two plane annotations represent the same plane by comparing surface normal angle
	 * between annotations
	 * 
	 * @param a1
	 *            plane 1
	 * @param a2
	 *            plane 2
	 * @return true if plane annotations should be combined into one
	 */
	@SuppressWarnings({ "rawtypes", "unused" })
	private static boolean isSamePlane(PrimitiveAnnotation a1, PrimitiveAnnotation a2) {
		if (!(a1 instanceof PlaneAnnotation && a2 instanceof PlaneAnnotation))
			return true;
		return planeAngleWithinTolerance(((PlaneAnnotation) a1).getPlaneNormal(),
				((PlaneAnnotation) a2).getPlaneNormal());
	}

	/**
	 * @param failedFittings
	 */
	@SuppressWarnings("rawtypes")
	private static void mergeWithNeighbors(MeshCas cas, Set<PrimitiveAnnotation> failedFittings,
			Set<Annotation> toRefit) {

		for (PrimitiveAnnotation pa : failedFittings) {

			@SuppressWarnings("unchecked")
			Set<PrimitiveAnnotation> neighborAnnotations = pa.getNeighborAnnotations(cas,
					PrimitiveAnnotation.class);
			float maxArea = 0;
			PrimitiveAnnotation bestNeighbor = null;
			boolean bestIsSameInstance = false;
			// find the biggest neighbor annotation or one of the same type
			for (PrimitiveAnnotation a1 : neighborAnnotations) {

				if (failedFittings.contains(a1))
					continue;

				boolean sameConvexity = pa.getClass() == a1.getClass();
				if (sameConvexity && pa instanceof ConeAnnotation && a1 instanceof ConeAnnotation) {
					sameConvexity = ((ConeAnnotation) pa).isConcave() == ((ConeAnnotation) a1)
							.isConcave();
				}
				if (sameConvexity && pa instanceof SphereAnnotation
						&& a1 instanceof SphereAnnotation) {
					sameConvexity = ((SphereAnnotation) pa).isConcave() == ((SphereAnnotation) a1)
							.isConcave();
				}

				if (sameConvexity) {
					bestIsSameInstance = true;
					if (a1.getArea() > maxArea) {
						bestNeighbor = a1;
						maxArea = a1.getArea();
					}
				} else if (bestIsSameInstance)
					continue;

				if (a1.getArea() > maxArea) {
					bestNeighbor = a1;
					maxArea = a1.getArea();
				}
			}
			if (bestNeighbor != null) {
				// merge
				synchronized (bestNeighbor.getMesh().getTriangles()) {
					bestNeighbor.getMesh().getTriangles().addAll(pa.getMesh().getTriangles());
				}
				toRefit.add(bestNeighbor);
			}
		}
	}

	/**
	 * Check if angle between the given surface normals is within <tt>PLANE_TOLERANCE</tt>.
	 * 
	 * @param norm1
	 *            surface normal 1
	 * @param norm2
	 *            surface normal 2
	 * @return true if angle is within tolerance
	 */
	private static boolean planeAngleWithinTolerance(Vector3f norm1, Vector3f norm2) {
		double dot = norm1.dot(norm2);
		if (dot > 1.0) // due to floating point arithmetic
			dot = 1.0;

		// Angle between 0 and 180 degree because angle of normal vector may be exactly 0 or 180
		// degree for same plane
		double angle = Math.acos(dot) + Math.PI * 2;
		angle = angle % Math.PI;

		return (angle <= PLANE_TOLERANCE || angle >= Math.PI - PLANE_TOLERANCE);
	}

	/**
	 * Map which maps a primitive type to a triangle
	 */
	private final HashMap<Triangle, PrimitiveType>	trianglePrimitiveTypeMap	= new HashMap<Triangle, PrimitiveType>();

	/**
	 * list of all vertices of cad model
	 */
	List<Vertex>									allVertices;

	/**
	 * list of all triangles of cad model
	 */
	List<Triangle>									allTriangles;

	/**
	 * Number of triangles already elaborated/processed. Used for indicating current progress
	 */
	final AtomicInteger								itemsElaborated				= new AtomicInteger(
																						0);

	/**
	 * Analyse triangle for its primitive type
	 * 
	 * @param cas
	 *            main cas
	 * @param triangle
	 *            triangle to analyze
	 * @param alreadyInAnnotation
	 *            set of already analyzed triangles
	 */
	protected void analyseTriangle(MeshCas cas, Triangle triangle,
			HashSet<Triangle> alreadyInAnnotation) {
		if (alreadyInAnnotation.contains(triangle))
			return;

		@SuppressWarnings("rawtypes")
		PrimitiveAnnotation annotation;
		PrimitiveType type = getTrianglePrimitiveType(cas.getCurvatures(), triangle);
		if (type == PrimitiveType.PLANE)
			annotation = new PlaneAnnotation(cas.getCurvatures(), cas.getModel());
		else if (type == PrimitiveType.SPHERE_CONCAVE || type == PrimitiveType.SPHERE_CONVEX)
			annotation = new SphereAnnotation(cas.getCurvatures(), cas.getModel(),
					type == PrimitiveType.SPHERE_CONCAVE);
		else
			annotation = new ConeAnnotation(cas.getCurvatures(), cas.getModel(),
					type == PrimitiveType.CONE_CONCAVE);

		synchronized (annotation.getMesh().getTriangles()) {
			annotation.getMesh().getTriangles().add(triangle);
		}
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
			boolean isEqual = (type == getTrianglePrimitiveType(cas.getCurvatures(), currNeighbor));

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
	 * @see edu.tum.cs.vis.model.uima.analyser.MeshAnalyser#getLogger()
	 */
	@Override
	public Logger getLogger() {
		return logger;
	}

	/* (non-Javadoc)
	 * @see edu.tum.cs.vis.model.uima.analyser.MeshAnalyser#getName()
	 */
	@Override
	public String getName() {
		return "Primitive";
	}

	/**
	 * Get primitive type of triangle
	 * 
	 * @param curvatures
	 *            curvature property for each vertex
	 * @param triangle
	 *            triangle to analyze
	 * @return primitive type for triangle
	 */
	private PrimitiveType getTrianglePrimitiveType(HashMap<Vertex, Curvature> curvatures,
			Triangle triangle) {
		return getTrianglePrimitiveType(curvatures, triangle, true);
	}

	/**
	 * Get primitive type of triangle by optionally averaging over neighboring triangles. First
	 * determines primitive type of triangle and then checks if neighboring triangles are of the
	 * same type. If triangle is totally different then type of neighboring triangles is returned.
	 * 
	 * @param curvatures
	 *            curvature property for each vertex
	 * @param triangle
	 *            triangle to analyze
	 * @param checkNeighbors
	 *            set to true if smoothing by neighbor triangles should be enabled
	 * @return primitive type for triangle
	 */
	private PrimitiveType getTrianglePrimitiveType(HashMap<Vertex, Curvature> curvatures,
			Triangle triangle, boolean checkNeighbors) {
		if (!checkNeighbors && trianglePrimitiveTypeMap.containsKey(triangle))
			return trianglePrimitiveTypeMap.get(triangle);

		// determine type of triangle
		int planeCnt = 0;
		int sphereConvexCnt = 0;
		int sphereConcavCnt = 0;
		int coneConvexCnt = 0;
		int coneConcavCnt = 0;
		for (Vertex v : triangle.getPosition()) {

			Curvature c = curvatures.get(v);
			if (c.getPrimitiveType() == PrimitiveType.PLANE)
				planeCnt++;
			else if (c.getPrimitiveType() == PrimitiveType.SPHERE_CONVEX)
				sphereConvexCnt++;
			else if (c.getPrimitiveType() == PrimitiveType.SPHERE_CONCAVE)
				sphereConcavCnt++;
			else if (c.getPrimitiveType() == PrimitiveType.CONE_CONVEX)
				coneConvexCnt++;
			else if (c.getPrimitiveType() == PrimitiveType.CONE_CONCAVE)
				coneConcavCnt++;
		}

		trianglePrimitiveTypeMap.put(
				triangle,
				getTypeForCounts(planeCnt, sphereConvexCnt, sphereConcavCnt, coneConvexCnt,
						coneConcavCnt));

		if (checkNeighbors) {
			// smooth type by neighbors
			for (Triangle t : triangle.getNeighbors()) {
				PrimitiveType type = getTrianglePrimitiveType(curvatures, t, false);

				if (type == PrimitiveType.PLANE)
					planeCnt += 1;
				else if (type == PrimitiveType.SPHERE_CONVEX)
					sphereConvexCnt += 1;
				else if (type == PrimitiveType.SPHERE_CONCAVE)
					sphereConcavCnt += 1;
				else if (type == PrimitiveType.CONE_CONVEX)
					coneConvexCnt += 1;
				else if (type == PrimitiveType.CONE_CONCAVE)
					coneConcavCnt += 1;
			}
		}

		return getTypeForCounts(planeCnt, sphereConvexCnt, sphereConcavCnt, coneConvexCnt,
				coneConcavCnt);

	}

	/* (non-Javadoc)
	 * @see edu.tum.cs.vis.model.uima.analyser.MeshAnalyser#processStart(edu.tum.cs.vis.model.uima.cas.MeshCas)
	 */
	@SuppressWarnings("rawtypes")
	@Override
	public void processStart(final MeshCas cas) {
		allVertices = cas.getModel().getVertices();
		allTriangles = cas.getModel().getTriangles();

		// set primitive type for all vertices
		List<Callable<Void>> threads = new LinkedList<Callable<Void>>();

		final int interval = 500;

		for (int start = 0; start < allVertices.size(); start += interval) {
			final int st = start;
			threads.add(new Callable<Void>() {

				@Override
				public Void call() throws Exception {
					int end = Math.min(st + interval, allVertices.size());
					for (int i = st; i < end; i++) {
						analyseVertex(cas.getCurvatures(), allVertices.get(i));
						itemsElaborated.incrementAndGet();
					}
					return null;
				}

			});
		};

		ThreadPool.executeInPool(threads);
		threads.clear();

		// set primitive type for all triangles
		final HashSet<Triangle> alreadyInAnnotation = new HashSet<Triangle>();

		for (Triangle t : allTriangles) {
			analyseTriangle(cas, t, alreadyInAnnotation);
			itemsElaborated.incrementAndGet();
		}
		combineSmallAnnotations(cas);

		combineSameNeighboringAnnotations(cas, cas.getAnnotations(), null);

		final Set<PrimitiveAnnotation> failedFittings = new HashSet<PrimitiveAnnotation>();
		failedFittings.addAll(fitAnnotations(cas.getAnnotations()));

		Set<Annotation> toRefit = new HashSet<Annotation>();
		if (failedFittings.size() > 0) {
			logger.debug("Merging failed fittings into neighbors (" + failedFittings.size() + ")");
			mergeWithNeighbors(cas, failedFittings, toRefit);
			combineSameNeighboringAnnotations(cas, cas.getAnnotations(), toRefit);
		}
		fitAnnotations(toRefit);
		toRefit.clear();

		// now check if all sphere annotations are spheres of if they should be cones by evaluating
		// fit error.
		// The fit error should be smaller than 0.001 for good fitted spheres

		logger.debug("Checking spheres ...");

		Set<Annotation> toAdd = new HashSet<Annotation>();
		Set<Annotation> toRemove = new HashSet<Annotation>();

		// Merge spheres with small area coverage into neighbor
		Set<PrimitiveAnnotation> smallSpheres = new HashSet<PrimitiveAnnotation>();

		for (Iterator<Annotation> it = cas.getAnnotations().iterator(); it.hasNext();) {
			Annotation a = it.next();
			if (a instanceof SphereAnnotation) {
				SphereAnnotation sa = (SphereAnnotation) a;

				if (sa.getAreaCoverage() < 0.01) {
					smallSpheres.add(sa);
					continue;
				}

				if (sa.getSphere().getFitError() < 0.005)
					continue;

				// create a temporary cone annotation and let it fit:
				ConeAnnotation tmp = new ConeAnnotation(cas.getCurvatures(), cas.getModel(),
						sa.isConcave());
				synchronized (tmp.getMesh().getTriangles()) {
					tmp.getMesh().getTriangles().addAll(sa.getMesh().getTriangles());
				}
				if (!tmp.fit())
					continue;
				tmp.updateAnnotationArea();
				if (tmp.getCone().getFitError() < sa.getSphere().getFitError()) {
					toRemove.add(a);
					toAdd.add(tmp);
					// logger.debug("Changing sphere annotation to cone because fit error is smaller: "
					// + tmp.getCone().getFitError() + "<" + sa.getSphere().getFitError());
				}
			}
		}

		synchronized (cas.getAnnotations()) {
			cas.getAnnotations().removeAll(smallSpheres);
			cas.getAnnotations().removeAll(toRemove);
			cas.getAnnotations().addAll(toAdd);
		}
		mergeWithNeighbors(cas, smallSpheres, toRefit);
		logger.debug("Combining neighbors ...");
		// Combine neighboring annotations which were previously different types before changing
		// sphere to cone
		combineSameNeighboringAnnotations(cas, cas.getAnnotations(), toRefit);
		fitAnnotations(toRefit);

		itemsElaborated.incrementAndGet();

	}

	/* (non-Javadoc)
	 * @see edu.tum.cs.vis.model.uima.analyser.MeshAnalyser#updateProgress()
	 */
	@Override
	public void updateProgress() {
		if (allVertices != null && allVertices.size() > 0) {
			setProgress(itemsElaborated.get()
					/ (float) (allVertices.size() + allTriangles.size() + 1) * 100.0f);
		}

	}

}
