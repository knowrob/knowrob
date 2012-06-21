/*******************************************************************************
 * Copyright (c) 2012 Stefan Profanter. All rights reserved. This program and the accompanying
 * materials are made available under the terms of the GNU Public License v3.0 which accompanies
 * this distribution, and is available at http://www.gnu.org/licenses/gpl.html
 * 
 * Contributors: Stefan Profanter - initial API and implementation, Year: 2012
 ******************************************************************************/
package edu.tum.cs.vis.model.uima.annotation.primitive;

import java.awt.Color;
import java.util.ArrayList;
import java.util.HashSet;

import javax.vecmath.Vector3f;

import org.ejml.alg.dense.decomposition.SingularMatrixException;
import org.ejml.simple.SimpleMatrix;
import org.ejml.simple.SimpleSVD;

import processing.core.PGraphics;
import edu.tum.cs.vis.model.uima.annotation.PrimitiveAnnotation;
import edu.tum.cs.vis.model.util.Curvature;
import edu.tum.cs.vis.model.util.Triangle;
import edu.tum.cs.vis.model.util.Vertex;
import edu.tum.cs.vis.model.view.MeshReasoningView;

/**
 * @author Stefan Profanter
 * 
 */
public class ConeAnnotation extends PrimitiveAnnotation {

	/**
	 * 
	 */
	private static final long	serialVersionUID	= -7420446109108464883L;

	private static Vector3f getIntersectionPoint(Vertex pos1, Vector3f dir1, Vertex pos2,
			Vector3f dir2) {
		SimpleMatrix A = new SimpleMatrix(2, 2);
		SimpleMatrix b = new SimpleMatrix(2, 1);

		if (dir1.x == 0 && dir2.x == 0)
			A.setRow(0, 0, dir1.z, -dir2.z);
		else
			A.setRow(0, 0, dir1.x, -dir2.x);

		if (dir1.y == 0 && dir2.y == 0)
			A.setRow(1, 0, dir1.z, -dir2.z);
		else
			A.setRow(1, 0, dir1.y, -dir2.y);

		b.setRow(1, 0, pos2.y - pos1.y);

		try {
			SimpleMatrix x = A.solve(b);
			float t = (float) x.get(0, 0);

			if (t <= 0.5 || t >= 1.1)
				return null;

			return new Vector3f(pos1.x + t * dir1.x, pos1.y + t * dir1.y, pos1.z + t * dir1.z);

		} catch (SingularMatrixException e) {
			return null;
		}

	}

	private final boolean				concav;

	private final Vector3f				centroid		= new Vector3f();

	private final Vector3f				direction		= new Vector3f();
	private float						radiusLarge		= 0;

	private float						radiusSmall		= 0;

	private final ArrayList<Vector3f>	intersections	= new ArrayList<Vector3f>();

	public ConeAnnotation(boolean concav) {
		super(concav ? new Color(0, 125, 125) : new Color(255, 255, 0));
		this.concav = concav;
	}

	/* (non-Javadoc)
	 * @see edu.tum.cs.vis.model.uima.annotation.PrimitiveAnnotation#fitAnnotation()
	 */
	/* (non-Javadoc)
	 * @see edu.tum.cs.vis.model.uima.annotation.PrimitiveAnnotation#drawAnnotation(processing.core.PGraphics)
	 */
	@Override
	public void drawPrimitiveAnnotation(PGraphics g) {

		for (Triangle t : mesh.getTriangles()) {
			for (Vertex v : t.getPosition()) {
				Curvature c = v.getCurvature();
				Vector3f norm = (Vector3f) v.getNormalVector().clone();
				norm.scale(-1 / c.getCurvatureMax());

				g.fill(255, 0, 0);
				g.stroke(255, 255, 255);
				g.strokeWeight(2);

				g.line(v.x, v.y, v.z, v.x + norm.x, v.y + norm.y, v.z + norm.z);

				g.stroke(255, 0, 0);
				g.line(v.x, v.y, v.z, v.x + c.getPrincipleDirectionMax().x,
						v.y + c.getPrincipleDirectionMax().y, v.z + c.getPrincipleDirectionMax().z);
			}
		}
		g.stroke(0, 255, 0);
		g.strokeWeight(5);

		for (Vector3f v : intersections) {
			g.point(v.x, v.y, v.z);
		}

		g.fill(getDrawColor().getRed(), getDrawColor().getGreen(), getDrawColor().getBlue(), 120);

		g.pushMatrix();

		g.translate(centroid.x, centroid.y, centroid.z);

		float dirLen = direction.length();

		float rx = (float) Math.asin(-direction.y / dirLen);
		float ry = (float) Math.atan2(direction.x / dirLen, direction.z / dirLen);
		g.rotateY(ry);
		g.rotateX(rx);

		MeshReasoningView.drawCylinder(g, 30, radiusLarge, radiusSmall, dirLen * 2);
		g.popMatrix();

	}

	@Override
	public void fitAnnotation() {

		HashSet<Vertex> vertices = new HashSet<Vertex>();
		Vector3f centerOrig = getVertices(vertices);

		intersections.clear();

		Vertex[] vert = new Vertex[vertices.size()];
		int idx = 0;
		for (Vertex v : vertices)
			vert[idx++] = v;

		for (int i = 0; i < vert.length; i++) {
			Vertex v = vert[i];

			Vector3f norm = (Vector3f) v.getNormalVector().clone();
			norm.scale(-1 / v.getCurvature().getCurvatureMax());

			float distNorm = norm.length();

			for (int j = i + 1; j < vert.length; j++) {
				Vertex v2 = vert[j];
				if (v.equals(v2))
					continue;

				Vector3f norm2 = (Vector3f) v2.getNormalVector().clone();

				norm2.scale(-1 / v2.getCurvature().getCurvatureMax());

				Vector3f p = getIntersectionPoint(v, norm, v2, norm2);
				if (p != null) {
					Vector3f distP = (Vector3f) p.clone();
					distP.sub(v);

					if (distP.length() < distNorm)
						intersections.add(p);
				}
			}
		}

		centroid.scale(0);

		for (Vector3f v : intersections) {
			centroid.add(v);
		}
		centroid.scale(1f / intersections.size());

		int maxRows = 2000;

		int increment = (intersections.size() / maxRows);
		if (increment == 0)
			increment = 1;
		int numberOfPoints = intersections.size() / increment;
		if (intersections.size() % maxRows > 0)
			numberOfPoints++;

		SimpleMatrix A = new SimpleMatrix(
				numberOfPoints == 2 ? numberOfPoints + 1 : numberOfPoints, 4);

		int row = 0;

		// Weighted SVD according to
		// http://www.mathworks.com/matlabcentral/newsreader/view_thread/262996
		System.out.println("intersect: " + intersections.size());

		for (int i = 0; i < intersections.size(); i += increment) {
			Vector3f v = intersections.get(i);
			A.setRow(row++, 0, v.x - centroid.x, v.y - centroid.y, v.z - centroid.z, 1);
		}

		if (numberOfPoints == 2) {
			A.setRow(row++, 0, 0, 0, 0);
		}

		@SuppressWarnings("rawtypes")
		SimpleSVD svd = A.svd();

		direction.x = (float) svd.getV().get(0, 1);
		direction.y = (float) svd.getV().get(1, 1);
		direction.z = (float) svd.getV().get(2, 1);
		direction.normalize();

		// centroid may be shifted a bit upwards/downwards because on cones the intersection points
		// are a bit moved but it is on the generating line.
		// centerOrig is in the middle of the cylinder/cone but not on the generating line. So move
		// centroid to the height of centerOrig by dot product
		centerOrig.sub(centroid);
		float dist = centerOrig.dot(direction);
		direction.scale(dist);
		centroid.add(direction);
		direction.normalize();

		float distToCenter = 0;

		float dist1 = 0;
		float dist2 = 0;
		int dist1Cnt = 0;
		int dist2Cnt = 0;

		for (int i = 0; i < vert.length; i++) {

			Vector3f tmp = new Vector3f(vert[i]);
			tmp.sub(centroid);
			float dot = tmp.dot(direction);
			if (dot < 0) {
				dist1 += tmp.length();
				dist1Cnt++;
			} else {
				dist2 += tmp.length();
				dist2Cnt++;
			}
			distToCenter += Math.abs(dot);
		}

		distToCenter /= vert.length;
		dist1 *= -1;

		dist1 /= dist1Cnt;
		dist2 /= dist2Cnt;

		float r1 = (float) Math.sqrt(Math.pow(dist1, 2) - Math.pow(distToCenter, 2));
		float r2 = (float) Math.sqrt(Math.pow(dist2, 2) - Math.pow(distToCenter, 2));

		radiusLarge = r1;
		radiusSmall = r2;

		direction.scale(distToCenter);
	}

	/**
	 * @return the centroid
	 */
	public Vector3f getCentroid() {
		return centroid;
	}

	/**
	 * @return the direction
	 */
	public Vector3f getDirection() {
		return direction;
	}

	/**
	 * @return the radiusLarge
	 */
	public float getRadiusLarge() {
		return radiusLarge;
	}

	/**
	 * @return the radiusSmall
	 */
	public float getRadiusSmall() {
		return radiusSmall;
	}

}
