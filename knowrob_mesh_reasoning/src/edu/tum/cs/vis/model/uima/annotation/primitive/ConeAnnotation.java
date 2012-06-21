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
import java.util.Iterator;
import java.util.LinkedHashMap;

import javax.vecmath.Vector3f;

import org.ejml.simple.SimpleMatrix;
import org.ejml.simple.SimpleSVD;

import processing.core.PGraphics;
import edu.tum.cs.vis.model.uima.annotation.PrimitiveAnnotation;
import edu.tum.cs.vis.model.util.Vertex;
import edu.tum.cs.vis.model.view.MeshReasoningView;

/**
 * @author Stefan Profanter
 * 
 */
public class ConeAnnotation extends PrimitiveAnnotation {

	class VectorWeight {
		public Vector3f	v;
		public float	w;

		public VectorWeight(Vector3f v, float w) {
			this.v = v;
			this.w = w;
		}
	}

	/**
	 * 
	 */
	private static final long	serialVersionUID	= -7420446109108464883L;

	/** Difference between 1.0 and the minimum float greater than 1.0 **/
	private static double		FLT_EPSILON			= 1.1920929e-07f;

	/**
	 * Calculate the line segment PaPb that is the shortest route between two lines P1P2 and P3P4.
	 * Calculate also the values of mua and mub where Pa = line1pos + mua (line1dir) Pb = line2pos +
	 * mub (line2dir) Return FALSE if no solution exists.
	 * 
	 * Ported from: http://local.wasp.uwa.edu.au/~pbourke/geometry/lineline3d/
	 **/
	private static boolean calculateShortestRoute(Vertex line1pos, Vector3f line1dir,
			Vertex line2pos, Vector3f line2dir, Vector3f pa, Vector3f pb) {
		Vector3f p13;
		double d1343, d4321, d1321, d4343, d2121;
		double numer, denom;

		p13 = new Vector3f(line1pos);
		p13.sub(line2pos);

		if (line2dir.lengthSquared() < FLT_EPSILON)
			return false;

		if (line1dir.lengthSquared() < FLT_EPSILON)
			return false;

		d1343 = p13.x * line2dir.x + p13.y * line2dir.y + p13.z * line2dir.z;
		d4321 = line2dir.x * line1dir.x + line2dir.y * line1dir.y + line2dir.z * line1dir.z;
		d1321 = p13.x * line1dir.x + p13.y * line1dir.y + p13.z * line1dir.z;
		d4343 = line2dir.x * line2dir.x + line2dir.y * line2dir.y + line2dir.z * line2dir.z;
		d2121 = line1dir.x * line1dir.x + line1dir.y * line1dir.y + line1dir.z * line1dir.z;

		denom = d2121 * d4343 - d4321 * d4321;
		if (Math.abs(denom) < FLT_EPSILON)
			return false;
		numer = d1343 * d4321 - d1321 * d4343;

		float mua = (float) (numer / denom);
		float mub = (float) ((d1343 + d4321 * mua) / d4343);

		if (mua < 0 || mub < 0 || mua > 1.5 || mub > 1.5)
			return false;

		pa.x = line1pos.x + mua * line1dir.x;
		pa.y = line1pos.y + mua * line1dir.y;
		pa.z = line1pos.z + mua * line1dir.z;
		pb.x = line2pos.x + mub * line2dir.x;
		pb.y = line2pos.y + mub * line2dir.y;
		pb.z = line2pos.z + mub * line2dir.z;

		return true;
	}

	private final boolean					concav;
	private final Vector3f					centroid		= new Vector3f();

	private final Vector3f					direction		= new Vector3f();

	private float							radiusLarge		= 0;

	private float							radiusSmall		= 0;

	private final ArrayList<VectorWeight>	intersections	= new ArrayList<VectorWeight>();

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

		g.noStroke();

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

		LinkedHashMap<Vertex, Float> vertices = new LinkedHashMap<Vertex, Float>();
		getVerticesWithWeight(vertices);

		/*
		 * Get all intersection points between the vector normals to each other.
		 */

		intersections.clear();

		Vector3f pa = new Vector3f();
		Vector3f pb = new Vector3f();

		HashSet<Vertex> alreadyAdded = new HashSet<Vertex>();

		int cnt = 0;
		for (Iterator<Vertex> it = vertices.keySet().iterator(); it.hasNext(); cnt++) {

			// Find shortest route from this vector normal to another
			Vertex v = it.next();
			if (alreadyAdded.contains(v))
				continue;

			float weight = vertices.get(v);

			// Get vector normal and set its length according to the curvature radius (= 1/ curv).
			// So on a perfect cylinder the vectors end in the same point on each end.
			Vector3f norm = (Vector3f) v.getNormalVector().clone();
			norm.scale(-1 / v.getCurvature().getCurvatureMax());

			// Find the shortest intersection route
			float minDist = Float.MAX_VALUE;
			VectorWeight addCandidateA = null;
			VectorWeight addCandidateB = null;
			Vertex vAdded = null;

			// float distNorm = norm.length();
			Iterator<Vertex> it2 = vertices.keySet().iterator();
			// forward iterator to position after it
			for (int i = 0; i <= cnt && it2.hasNext(); i++)
				it2.next();
			for (; it2.hasNext();) {

				Vertex v2 = it2.next();
				if (v.equals(v2))
					continue;

				Vector3f norm2 = (Vector3f) v2.getNormalVector().clone();
				norm2.scale(-1 / v2.getCurvature().getCurvatureMax());

				if (calculateShortestRoute(v, norm, v2, norm2, pa, pb)) {
					// v and v2 have an intersection route. Check if it is the shortest possible for
					// v
					Vector3f tmp = new Vector3f(pa);
					tmp.sub(pb);
					if (tmp.lengthSquared() < minDist) {
						minDist = tmp.lengthSquared();

						float weight2 = vertices.get(v2);
						// Get middle point of intersection route
						pa.add(pb);
						pa.scale(1f / 2f);
						addCandidateA = new VectorWeight((Vector3f) pa.clone(), weight);
						addCandidateB = new VectorWeight((Vector3f) pb.clone(), weight2);
						vAdded = v2;
					}
				}
			}

			alreadyAdded.add(v);
			if (addCandidateA != null) {
				alreadyAdded.add(vAdded);
				intersections.add(addCandidateA);
				intersections.add(addCandidateB);
			}
		}
		alreadyAdded.clear();
		alreadyAdded = null;

		// If no intersections found use the endpoints of each normalvector multiplied by the
		// curvature radius.
		// Better than nothing.
		if (intersections.size() == 0) {

			for (Iterator<Vertex> it = vertices.keySet().iterator(); it.hasNext();) {
				Vertex v = it.next();
				float weight = vertices.get(v);

				Vector3f norm = (Vector3f) v.getNormalVector().clone();
				norm.scale(-1 / v.getCurvature().getCurvatureMax());
				norm.add(v);
				intersections.add(new VectorWeight(norm, weight));
			}

		}

		// Now intersections are found. Find the best fitting line between through these points.
		// Should be the generating line (line in the middle of cylinder).
		getBestFittingLine(intersections, direction);
		direction.normalize();

		centroid.scale(0);
		for (VectorWeight vw : intersections) {

			centroid.x += vw.v.x;
			centroid.y += vw.v.y;
			centroid.z += vw.v.z;
		}
		centroid.scale(1f / intersections.size());

		double radiusBottom = 0;
		double radiusTop = 0;
		double radiusBottomWeight = 0;
		double radiusTopWeight = 0;
		double distBottom = 0;
		double distTop = 0;

		for (Iterator<Vertex> it = vertices.keySet().iterator(); it.hasNext();) {
			Vertex v = it.next();
			double weight = vertices.get(v);

			// Project v onto direction vector
			Vector3f tmp = new Vector3f(v);
			tmp.sub(centroid);
			double dot = tmp.dot(direction); // distance from center to v on direction vector

			// Calculate point on direction vector where v is perpendicular to direction vector
			tmp = new Vector3f(direction);
			tmp.scale((float) dot);
			tmp.add(centroid);

			// Sub v to get perpendicular distance between point on direction and v = Radius
			tmp.sub(v);
			float rad = tmp.length();

			if (dot < 0) {
				// v is on bottom of cone
				radiusBottom += rad * weight;
				radiusBottomWeight += weight;
				distBottom += Math.abs(dot) * weight;
			} else {
				// v is on top of cone
				radiusTop += rad * weight;
				radiusTopWeight += weight;
				distTop += Math.abs(dot) * weight;
			}
		}

		distBottom /= radiusBottomWeight;
		distTop /= radiusTopWeight;

		// Move centroid along direction vector to set it to the center of bottom and top
		float diff = (float) (distTop - distBottom) / 2;
		Vector3f tmp = new Vector3f(direction);
		tmp.scale(diff);
		centroid.add(tmp);

		radiusLarge = (float) (radiusBottom / radiusBottomWeight);
		radiusSmall = (float) (radiusTop / radiusTopWeight);

		// Set direction length to the height of cone
		direction.scale((float) (distTop + distBottom) / 2);
	}

	private void getBestFittingLine(ArrayList<VectorWeight> points, Vector3f dir) {
		// Try to best fit a line through the intersection points:

		// Number of maximum rows for SVD. If value is too large SVD will throw a Out of Heap
		// Exception
		int maxRows = 1000;

		// Calculate the increment value if number of intersections is larger than maxRows.
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

		for (int i = 0; i < intersections.size(); i += increment) {
			VectorWeight vw = intersections.get(i);
			A.setRow(row++, 0, (vw.v.x - centroid.x) * vw.w, (vw.v.y - centroid.y) * vw.w,
					(vw.v.z - centroid.z) * vw.w, vw.w);
		}

		if (numberOfPoints == 2) {
			A.setRow(row++, 0, 0, 0, 0);
		}

		@SuppressWarnings("rawtypes")
		SimpleSVD svd = A.svd();

		dir.x = (float) svd.getV().get(0, 1);
		dir.y = (float) svd.getV().get(1, 1);
		dir.z = (float) svd.getV().get(2, 1);
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

	/**
	 * @return the concav
	 */
	public boolean isConcav() {
		return concav;
	}

}
