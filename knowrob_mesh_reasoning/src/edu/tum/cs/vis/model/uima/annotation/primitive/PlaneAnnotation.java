/*******************************************************************************
 * Copyright (c) 2012 Stefan Profanter. All rights reserved. This program and the accompanying
 * materials are made available under the terms of the GNU Public License v3.0 which accompanies
 * this distribution, and is available at http://www.gnu.org/licenses/gpl.html
 * 
 * Contributors: Stefan Profanter - initial API and implementation, Year: 2012
 ******************************************************************************/
package edu.tum.cs.vis.model.uima.annotation.primitive;

import java.awt.Color;
import java.util.HashMap;
import java.util.Map.Entry;

import javax.vecmath.Point2f;
import javax.vecmath.Point3f;
import javax.vecmath.Tuple3f;
import javax.vecmath.Vector2f;
import javax.vecmath.Vector3f;

import org.ejml.simple.SimpleMatrix;
import org.ejml.simple.SimpleSVD;

import processing.core.PConstants;
import processing.core.PGraphics;
import edu.tum.cs.vis.model.Model;
import edu.tum.cs.vis.model.uima.annotation.PrimitiveAnnotation;
import edu.tum.cs.vis.model.util.Curvature;
import edu.tum.cs.vis.model.util.Triangle;
import edu.tum.cs.vis.model.util.Vertex;
import edu.tum.cs.vis.model.util.algorithm.BestFitRectangle2D;

/**
 * @author Stefan Profanter
 * 
 */
public class PlaneAnnotation extends PrimitiveAnnotation {

	/**
	 * 
	 */
	private static final long	serialVersionUID	= 7758656289829843165L;

	private static Point2f[] projectPointsInto2D(HashMap<Vertex, Float> vertices,
			Vector3f normalVector, Vector3f centroid, Vector3f v, Vector3f u) {

		// Get an additional vector besides the normal for creating an orthonormal basis
		Vector3f rightVector = null;

		if (Math.abs(normalVector.x) > Math.abs(normalVector.y))
			rightVector = new Vector3f(0, 1, 0);
		else

			rightVector = new Vector3f(1, 0, 0);

		// Once you have a normal and a "right," you can calculate an orthonormal basis:
		v.cross(rightVector, normalVector);
		v.normalize();

		u.cross(normalVector, v);

		Point2f[] points = new Point2f[vertices.size()];

		// And once you have your orthonormal basis, you can simply classify each point by dot
		// products:
		int i = 0;

		for (Entry<Vertex, Float> e : vertices.entrySet()) {
			Vertex p3d = e.getKey();
			Vector3f po = new Vector3f(p3d);
			po.sub(centroid);
			points[i++] = new Point2f(u.dot(po), v.dot(po));
		}

		return points;

	}

	private final Vector3f	planeNormal	= new Vector3f();
	private Vector3f		centroid	= new Vector3f();

	private final Vector3f	shortSide	= new Vector3f();

	private final Vector3f	longSide	= new Vector3f();

	private final Point3f	corner[]	= new Point3f[4];

	/**
	 * @param annotationColor
	 */
	public PlaneAnnotation(HashMap<Vertex, Curvature> curvatures, Model model) {
		super(PlaneAnnotation.class, curvatures, model, new Color(250, 200, 255));
	}

	/* (non-Javadoc)
	 * @see edu.tum.cs.vis.model.uima.annotation.PrimitiveAnnotation#drawAnnotation(processing.core.PGraphics)
	 */
	@Override
	public void drawPrimitiveAnnotation(PGraphics g) {
		/*g.fill(255, 0, 0);
		g.stroke(255, 255, 255);
		g.strokeWeight(2);

		g.line(centroid.x, centroid.y, centroid.z, centroid.x + planeNormal.x, centroid.y
				+ planeNormal.y, centroid.z + planeNormal.z);

		g.stroke(255, 255, 0);
		g.line(centroid.x, centroid.y, centroid.z, centroid.x + shortSide.x, centroid.y
				+ shortSide.y, centroid.z + shortSide.z);
		g.stroke(255, 0, 255);
		g.line(centroid.x, centroid.y, centroid.z, centroid.x + longSide.x,
				centroid.y + longSide.y, centroid.z + longSide.z);*/

		g.fill(getDrawColor().getRed(), getDrawColor().getGreen(), getDrawColor().getBlue(), 200);
		g.noStroke();

		g.beginShape(PConstants.QUADS);

		for (int i = 0; i < 4; i++) {
			g.vertex(corner[i].x, corner[i].y, corner[i].z);
		}

		g.endShape(PConstants.CLOSE);

	}

	/* (non-Javadoc)
	 * @see edu.tum.cs.vis.model.uima.annotation.PrimitiveAnnotation#fitAnnotation()
	 */
	@Override
	public void fitAnnotation() {
		/*
		 * Best fitting plane.
		 * 
		 * Formula for plane: a*x + b*y + c*z + d = 0
		 * d = -(a*x + b*y + c*z)
		 * 
		 * Centroid (average values): x0,y0,z0
		 * 
		 * To minimize: Sum [|a(xi-x0) + b(yi-y0) + c(zi-z0)|^2]
		 * 
		 * 
		 *   v^T  = [a  b  c]
		 *
		 *        --                             --
		 *        | x1 - x0    y1 - y0    z1 - z0 |
		 *        | x2 - x0    y2 - y0    z2 - z0 |
		 *   M =  |    .          .          .    |
		 *        |    .          .          .    |
		 *        |    .          .          .    |
		 *        | xn - x0    yn - y0    zn - z0 |
		 *        --                             --
		 * 
		 */

		HashMap<Vertex, Float> vertices = new HashMap<Vertex, Float>();
		centroid = getVerticesWithWeight(vertices);

		int numberOfPoints = vertices.size();

		SimpleMatrix A = new SimpleMatrix(
				numberOfPoints == 3 ? numberOfPoints + 1 : numberOfPoints, 4);

		int row = 0;

		// Weighted SVD according to
		// http://www.mathworks.com/matlabcentral/newsreader/view_thread/262996
		for (Entry<Vertex, Float> e : vertices.entrySet()) {
			float weight = (float) Math.sqrt(e.getValue());
			Vertex v = e.getKey();

			A.setRow(row++, 0, (v.x - centroid.x) * weight, (v.y - centroid.y) * weight,
					(v.z - centroid.z) * weight, 1);
		}

		if (numberOfPoints == 3) {
			A.setRow(row++, 0, 0, 0, 0);
		}

		@SuppressWarnings("rawtypes")
		SimpleSVD svd = A.svd();

		planeNormal.x = (float) svd.getV().get(0, 3);
		planeNormal.y = (float) svd.getV().get(1, 3);
		planeNormal.z = (float) svd.getV().get(2, 3);
		if (planeNormal.length() == 0) { // if svd fails, should never happen
			planeNormal.get(mesh.getTriangles().get(0).getNormalVector());
		} else
			planeNormal.normalize();

		// Check if plane normal is in the right direction by comparing it with average on all
		// triangle normals
		Vector3f trianglesNormal = new Vector3f();
		for (Triangle t : mesh.getTriangles()) {
			trianglesNormal.add(t.getNormalVector());
		}
		trianglesNormal.scale(1f / mesh.getTriangles().size());
		// trianglesNormal.sub(planeNormal);
		if (trianglesNormal.dot(planeNormal) < 0) // if normals are in opposite direction, inverse
													// planeNormal
			planeNormal.scale(-1f);

		updateFeatures(vertices);
	}

	/**
	 * @return the centroid
	 */
	public Vector3f getCentroid() {
		return centroid;
	}

	/**
	 * @return the centroid
	 */
	public Tuple3f getCentroidUnscaled() {
		return model.getUnscaled(centroid);
	}

	/**
	 * @return the corner
	 */
	public Point3f[] getCorner() {

		return corner;
	}

	/**
	 * @return the corner
	 */
	public Tuple3f[] getCornerUnscaled() {

		return model.getUnscaled(corner);
	}

	/**
	 * @return the longSide
	 */
	public Vector3f getLongSide() {
		return longSide;
	}

	/**
	 * @return the longSide
	 */
	public Vector3f getLongSideUnscaled() {
		return model.getUnscaled(longSide);
	}

	/**
	 * @return the planeNormal
	 */
	public Vector3f getPlaneNormal() {
		return planeNormal;
	}

	/* (non-Javadoc)
	 * @see edu.tum.cs.vis.model.uima.annotation.PrimitiveAnnotation#getPrimitiveArea()
	 */
	@Override
	public float getPrimitiveArea() {
		// shortSide is half length of the rectangles short side. Same with long side.
		return 4f * shortSide.length() * longSide.length();
	}

	/* (non-Javadoc)
	 * @see edu.tum.cs.vis.model.uima.annotation.PrimitiveAnnotation#getPrimitiveAreaUnscaled()
	 */
	@Override
	public float getPrimitiveAreaUnscaled() {
		// shortSide is half length of the rectangles short side. Same with long side.
		return model.getUnscaled(getPrimitiveArea());
	}

	/**
	 * @return the shortSide
	 */
	public Vector3f getShortSide() {
		return shortSide;
	}

	/**
	 * @return the shortSide
	 */
	public Vector3f getShortSideUnscaled() {
		return model.getUnscaled(shortSide);
	}

	private void updateFeatures(HashMap<Vertex, Float> vertices) {
		// Find the 2D minimal bounding box

		Vector3f axisU = new Vector3f();
		Vector3f axisV = new Vector3f();

		// 3d points to 2d
		Point2f points[] = projectPointsInto2D(vertices, planeNormal, centroid, axisU, axisV);

		Vector2f axisURect = new Vector2f();
		Vector2f axisVRect = new Vector2f();
		// get corner of 2D best fit rectangle
		Point2f corner2d[] = BestFitRectangle2D.getBestFitRectangle(points, axisURect, axisVRect);

		// project corner2d back to 3D corner
		for (int i = 0; i < 4; i++) {
			Vector3f tmpU = (Vector3f) axisU.clone();
			Vector3f tmpV = (Vector3f) axisV.clone();
			tmpV.scale(corner2d[i].x);
			tmpU.scale(corner2d[i].y);
			tmpU.add(tmpV);
			tmpU.add(centroid);
			corner[i] = new Point3f(tmpU);
		}

		// reset centroid to center of rectangle
		centroid.scale(0);

		for (int i = 0; i < 4; i++) {
			centroid.add(corner[i]);
		}
		centroid.scale(1f / 4);

		// calculate length of each side
		Vector3f c1 = new Vector3f(corner[1]);
		Vector3f c2 = new Vector3f(corner[2]);
		Vector3f c3 = new Vector3f(corner[3]);
		c1.sub(centroid);
		c2.sub(centroid);
		c3.sub(centroid);

		c1.add(c2);
		c1.scale(0.5f);
		c3.add(c2);
		c3.scale(0.5f);

		if (c1.length() < c3.length()) {
			c1.get(shortSide);
			c3.get(longSide);
		} else {
			c3.get(shortSide);
			c1.get(longSide);
		}

	}

}
