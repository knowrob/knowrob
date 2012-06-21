/*******************************************************************************
 * Copyright (c) 2012 Stefan Profanter. All rights reserved. This program and the accompanying
 * materials are made available under the terms of the GNU Public License v3.0 which accompanies
 * this distribution, and is available at http://www.gnu.org/licenses/gpl.html
 * 
 * Contributors: Stefan Profanter - initial API and implementation, Year: 2012
 ******************************************************************************/
package edu.tum.cs.vis.model.uima.analyzer;

import java.awt.Color;
import java.util.ArrayList;
import java.util.LinkedList;
import java.util.concurrent.atomic.AtomicInteger;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import org.apache.log4j.Logger;

import processing.core.PGraphics;
import edu.tum.cs.vis.model.uima.annotation.CurvatureAnnotation;
import edu.tum.cs.vis.model.uima.cas.MeshCas;
import edu.tum.cs.vis.model.util.Group;
import edu.tum.cs.vis.model.util.Mesh;
import edu.tum.cs.vis.model.util.Quaternion;
import edu.tum.cs.vis.model.util.Triangle;
import edu.tum.cs.vis.model.util.TriangleNeighbor;
import edu.tum.cs.vis.model.view.MeshReasoningView;

/**
 * @author Stefan Profanter
 * 
 */
public class CurvatureAnalyzer extends MeshAnalyzer {

	/**
	 * Log4J Logger
	 */
	private static Logger	logger					= Logger.getLogger(CurvatureAnalyzer.class);

	/**
	 * Factor divided by PI between area of circle used for curvature calculation and area of the
	 * triangle.
	 */
	private static float	GEODESIC_RADIUS_FACTOR	= 7 / (float) Math.PI;

	/**
	 * The circle will be cut in the following number of segments and for each segment the curvature
	 * is calculated. This is used to find min and max curvature.
	 * 
	 * MUST BE MULTIPLE OF 4!! (Because when max curvature is found the circle is divided into 4
	 * parts for getting kmin)
	 */
	private static int		NUMBER_OF_SEGMENTS		= 20;

	/**
	 * The circle will be cut in the following segment sizes (in radiant) and for each segment the
	 * curvature is calculated. This is used to find min and max curvature.
	 * 
	 */
	private static double	SEGMENT_ANGLE			= 2 * Math.PI / NUMBER_OF_SEGMENTS;

	private static void drawNeighbors(Triangle start, int maxLevel, PGraphics g) {

		// List of already visited triangles for BFS
		LinkedList<Triangle> visited = new LinkedList<Triangle>();
		visited.add(start);

		// FIFO queue for triangles to visit for BFS
		LinkedList<ItemWithLevel> queue = new LinkedList<ItemWithLevel>();
		if (start.getNeighbors() != null) {
			// Add all neighbor triangles to the queue
			for (TriangleNeighbor n : start.getNeighbors())
				queue.add(new ItemWithLevel(n, 1));
		}

		while (!queue.isEmpty()) {
			ItemWithLevel itm = queue.pop();
			TriangleNeighbor currNeighbor = itm.item;
			Triangle currNeighborTriangle;
			if (visited.contains(currNeighbor.getTriangle1()))
				currNeighborTriangle = currNeighbor.getTriangle2();

			else
				currNeighborTriangle = currNeighbor.getTriangle1();
			visited.add(currNeighborTriangle);

			currNeighborTriangle.draw(g, new Color(255, 255, itm.level / maxLevel * 255));

			// Add all neighbors of current triangle to queue
			synchronized (currNeighborTriangle.getNeighbors()) {
				for (TriangleNeighbor a : currNeighborTriangle.getNeighbors()) {
					Triangle newTriangle = a.getNeighbor(currNeighborTriangle);

					if (visited.contains(newTriangle))
						continue;
					queue.add(new ItemWithLevel(a, itm.level + 1));
				}

			}
		}
	}

	private static Triangle findIntersectionPoint(Point3d p1, Point3d p2, Triangle start,
			Point3d intersectionPoint, int maxLevel) {
		if (start.intersectsRay(p1, p2, intersectionPoint))
			return start;

		// List of already visited triangles for BFS
		LinkedList<Triangle> visited = new LinkedList<Triangle>();
		visited.add(start);

		// FIFO queue for triangles to visit for BFS
		LinkedList<ItemWithLevel> queue = new LinkedList<ItemWithLevel>();
		if (start.getNeighbors() != null) {
			// Add all neighbor triangles to the queue
			for (TriangleNeighbor n : start.getNeighbors())
				queue.add(new ItemWithLevel(n, 1));
		}

		while (!queue.isEmpty()) {
			ItemWithLevel itm = queue.pop();
			if (maxLevel >= 0 && itm.level > maxLevel)
				continue;
			TriangleNeighbor currNeighbor = itm.item;
			Triangle currNeighborTriangle;
			if (visited.contains(currNeighbor.getTriangle1()))
				currNeighborTriangle = currNeighbor.getTriangle2();

			else
				currNeighborTriangle = currNeighbor.getTriangle1();
			visited.add(currNeighborTriangle);

			if (currNeighborTriangle.intersectsRay(p1, p2, intersectionPoint))
				return currNeighborTriangle;

			// Add all neighbors of current triangle to queue
			synchronized (currNeighborTriangle.getNeighbors()) {
				for (TriangleNeighbor a : currNeighborTriangle.getNeighbors()) {
					Triangle newTriangle = a.getNeighbor(currNeighborTriangle);

					if (visited.contains(newTriangle))
						continue;
					queue.add(new ItemWithLevel(a, itm.level + 1));
				}

			}
		}

		return null;
	}

	/**
	 * Calculate curvature according to
	 * http://en.wikipedia.org/wiki/Curvature#Curvature_of_plane_curves
	 * 
	 * k = ||dT|| / ||ds||
	 * 
	 * dT is the distance between the two normal vectors in the same origin. ds is the distance
	 * between the two points
	 * 
	 * @param refPoint
	 * @param refNorm
	 * @param intersectionPoint
	 * @param curvNorm
	 * @return
	 */
	private static double getCurvatureValue(Point3d refPoint, Vector3d refNorm, Point3d curvPoint,
			Vector3d curvNorm) {

		Vector3d dT = (Vector3d) refNorm.clone();
		dT.sub(curvNorm);

		Vector3d ds = new Vector3d(refPoint);
		ds.sub(curvPoint);

		return dT.length() / ds.length();

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
	private static void processGroup(ArrayList<Triangle> allTriangles, Group g, MeshCas cas) {
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
	private static void processMesh(ArrayList<Triangle> allTriangles, Mesh m, final MeshCas cas) {

		if (m.getTriangles().size() == 0)
			return;

		allTriangles.addAll(m.getTriangles());
	}

	public static void triangleCurvature(Triangle start, MeshCas cas, PGraphics g) {
		double radius = Math.sqrt(start.getArea() * GEODESIC_RADIUS_FACTOR);
		Point3d cent = start.getCentroid();
		Vector3d norm = start.getNormalVector();

		// Get a perpendicular vector to the surface normal. Simply take the Vector from centroid to
		// a corner point
		Vector3d perp = new Vector3d(start.getPosition()[0]);
		perp.sub(cent);
		perp.normalize();

		Double normalCurvature[] = new Double[NUMBER_OF_SEGMENTS];
		Vector3d normalCurvatureDirection[] = new Vector3d[NUMBER_OF_SEGMENTS];

		Quaternion rotQuat = Quaternion.FromAxis(norm, SEGMENT_ANGLE);
		Quaternion rotQuatConj = rotQuat.getConjugate();

		// Rotate vector by 360° and measure the curvature for each step
		for (int i = 0; i < NUMBER_OF_SEGMENTS; i++) {

			perp = Quaternion.rotate(perp, rotQuat, rotQuatConj);
			// Set length of vector to radius
			perp.scale(radius);

			// Find an intersecting triangle. If radius was too large, it's getting smaller and
			// smaller until an intersecting triangle is found.
			while (normalCurvature[i] == null) {
				// Point on tangential plane where perp points to, with current angle
				Point3d circlePoint = new Point3d(cent.x + perp.x, cent.y + perp.y, cent.z + perp.z);
				// Point to indicate direction where to search for intersection. Direction is the
				// same as normal vector
				Point3d directionPoint = new Point3d(circlePoint.x + norm.x,
						circlePoint.y + norm.y, circlePoint.z + norm.z);

				Point3d intersectionPoint = new Point3d();

				Triangle intersectionTriangle = findIntersectionPoint(directionPoint, circlePoint,
						start, intersectionPoint, 6);
				if (intersectionTriangle != null) {
					normalCurvature[i] = getCurvatureValue(cent, norm, intersectionPoint,
							intersectionTriangle.getNormalVector());
					if (g != null)
						intersectionTriangle.draw(g, new Color(255, 255, 0));
				} else {
					perp.scale(0.8); // reduce radius and try again
				}
			}
			normalCurvatureDirection[i] = (Vector3d) perp.clone();

			Point3d circlePoint = new Point3d(cent.x + perp.x, cent.y + perp.y, cent.z + perp.z);
			Point3d directionPoint = new Point3d(circlePoint.x + norm.x, circlePoint.y + norm.y,
					circlePoint.z + norm.z);

			if (g != null) {

				g.noFill();
				g.strokeWeight(1);
				g.stroke(255, 0, 0);

				g.line((float) cent.x, (float) cent.y, (float) cent.z, (float) circlePoint.x,
						(float) circlePoint.y, (float) circlePoint.z);

				g.stroke(0, 0, 255);

				g.noFill();
				g.line((float) cent.x, (float) cent.y, (float) cent.z, (float) cent.x
						+ (float) norm.x, (float) cent.y + (float) norm.y, (float) cent.z
						+ (float) norm.z);

				start.draw(g, new Color(0, 255, 0, 125));

				// drawNeighbors(start, 4000, g);
			}

		}

		int maxIdx = 0;

		for (int i = 1; i < NUMBER_OF_SEGMENTS; i++) {
			if (normalCurvature[i] > normalCurvature[maxIdx])
				maxIdx = i;
		}

		int minIdx = (maxIdx + NUMBER_OF_SEGMENTS / 4) % NUMBER_OF_SEGMENTS;
		if (normalCurvature[(minIdx + NUMBER_OF_SEGMENTS / 2) % NUMBER_OF_SEGMENTS] < normalCurvature[minIdx])
			minIdx = (minIdx + NUMBER_OF_SEGMENTS / 2) % NUMBER_OF_SEGMENTS;
		int origMinIdx = minIdx;
		if (normalCurvature[(origMinIdx + 1) % NUMBER_OF_SEGMENTS] < normalCurvature[minIdx])
			minIdx = (minIdx + 1) % NUMBER_OF_SEGMENTS;
		else if (normalCurvature[(origMinIdx - 1 + NUMBER_OF_SEGMENTS) % NUMBER_OF_SEGMENTS] < normalCurvature[minIdx])
			minIdx = (minIdx + 1) % NUMBER_OF_SEGMENTS;
		// reset length to 1 and then multiply by curvature value
		normalCurvatureDirection[maxIdx].normalize();
		normalCurvatureDirection[maxIdx].scale(normalCurvature[maxIdx]);
		normalCurvatureDirection[minIdx].normalize();
		normalCurvatureDirection[minIdx].scale(normalCurvature[minIdx]);
		CurvatureAnnotation ann = new CurvatureAnnotation(start, normalCurvatureDirection[minIdx],
				normalCurvatureDirection[maxIdx]);
		start.addAnnotation(ann);

		if (MeshReasoningView.testIdx >= NUMBER_OF_SEGMENTS)
			MeshReasoningView.testIdx = 0;
		if (MeshReasoningView.testIdx < 0)
			MeshReasoningView.testIdx = NUMBER_OF_SEGMENTS - 1;

		if (g != null) {
			g.strokeWeight(5);
			g.stroke(0, 255, 255);

			g.line((float) cent.x, (float) cent.y, (float) cent.z, (float) cent.x
					+ (float) normalCurvatureDirection[MeshReasoningView.testIdx].x, (float) cent.y
					+ (float) normalCurvatureDirection[MeshReasoningView.testIdx].y, (float) cent.z
					+ (float) normalCurvatureDirection[MeshReasoningView.testIdx].z);
			// System.out.println("Curv: " + normalCurvature[MeshReasoningView.testIdx]);
		}

	}

	public static void triangleCurvature1(Triangle start, MeshCas cas, PGraphics g) {
		double radius = Math.sqrt(start.getArea() * GEODESIC_RADIUS_FACTOR);
		Point3d cent = start.getCentroid();
		Vector3d norm = start.getNormalVector();

		// Get a perpendicular vector to the surface normal. Simply take the Vector from centroid to
		// a corner point
		Vector3d perp = new Vector3d(start.getPosition()[0]);
		perp.sub(cent);
		perp.normalize();

		Double maxCurvature = Double.MIN_VALUE;
		Vector3d maxCurvatureDirection = new Vector3d(start.getPosition()[0]);

		Double minCurvature = Double.MAX_VALUE;
		Vector3d minCurvatureDirection = new Vector3d(start.getPosition()[1]);

		// List of already visited triangles for BFS
		LinkedList<Triangle> visited = new LinkedList<Triangle>();
		visited.add(start);

		// FIFO queue for triangles to visit for BFS
		LinkedList<ItemWithLevel> queue = new LinkedList<ItemWithLevel>();
		if (start.getNeighbors() != null) {
			// Add all neighbor triangles to the queue
			for (TriangleNeighbor n : start.getNeighbors())
				queue.add(new ItemWithLevel(n, 1));
		}

		while (!queue.isEmpty()) {
			ItemWithLevel itm = queue.pop();
			if (itm.level > 2)
				continue;
			TriangleNeighbor currNeighbor = itm.item;
			Triangle currNeighborTriangle;
			if (visited.contains(currNeighbor.getTriangle1()))
				currNeighborTriangle = currNeighbor.getTriangle2();

			else
				currNeighborTriangle = currNeighbor.getTriangle1();
			visited.add(currNeighborTriangle);

			double tmpCurvature = getCurvatureValue(cent, norm, currNeighborTriangle.getCentroid(),
					currNeighborTriangle.getNormalVector());

			if (tmpCurvature > maxCurvature) {
				maxCurvature = tmpCurvature;
				maxCurvatureDirection = new Vector3d(currNeighborTriangle.getCentroid());
				maxCurvatureDirection.sub(cent);
			}
			if (tmpCurvature < minCurvature) {
				minCurvature = tmpCurvature;
				minCurvatureDirection = new Vector3d(currNeighborTriangle.getCentroid());
				minCurvatureDirection.sub(cent);
			}

			// Add all neighbors of current triangle to queue
			synchronized (currNeighborTriangle.getNeighbors()) {
				for (TriangleNeighbor a : currNeighborTriangle.getNeighbors()) {
					Triangle newTriangle = a.getNeighbor(currNeighborTriangle);

					if (visited.contains(newTriangle))
						continue;
					queue.add(new ItemWithLevel(a, itm.level + 1));
				}

			}
		}

		maxCurvatureDirection.normalize();
		maxCurvatureDirection.scale(maxCurvature / 10);

		minCurvatureDirection.normalize();
		minCurvatureDirection.scale(minCurvature / 10);

		if (g != null) {
			start.draw(g, new Color(0, 255, 0, 125));

			g.stroke(255, 0, 0);
			g.line((float) cent.x - (float) maxCurvatureDirection.x, (float) cent.y
					- (float) maxCurvatureDirection.y, (float) cent.z
					- (float) maxCurvatureDirection.z, (float) cent.x
					+ (float) maxCurvatureDirection.x, (float) cent.y
					+ (float) maxCurvatureDirection.y, (float) cent.z
					+ (float) maxCurvatureDirection.z);

			g.stroke(0, 0, 255);

			g.line((float) cent.x - (float) minCurvatureDirection.x, (float) cent.y
					- (float) minCurvatureDirection.y, (float) cent.z
					- (float) minCurvatureDirection.z, (float) cent.x
					+ (float) minCurvatureDirection.x, (float) cent.y
					+ (float) minCurvatureDirection.y, (float) cent.z
					+ (float) minCurvatureDirection.z);
		}

		CurvatureAnnotation ann = new CurvatureAnnotation(start, minCurvatureDirection,
				maxCurvatureDirection);
		start.addAnnotation(ann);
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
		return "Curvature";
	}

	/* (non-Javadoc)
	 * @see edu.tum.cs.vis.model.uima.analyzer.MeshAnalyzer#processStart(edu.tum.cs.vis.model.uima.cas.MeshCas)
	 */
	@Override
	public void processStart(MeshCas cas) {
		allTriangles = new ArrayList<Triangle>();

		processGroup(allTriangles, cas.getGroup(), cas);

		for (int i = 0; i < allTriangles.size(); i++) {
			MeshReasoningView.test = i;
			triangleCurvature(allTriangles.get(i), cas, null);
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

class ItemWithLevel {
	TriangleNeighbor	item;
	int					level;

	public ItemWithLevel(TriangleNeighbor i, int l) {
		item = i;
		level = l;
	}
}
