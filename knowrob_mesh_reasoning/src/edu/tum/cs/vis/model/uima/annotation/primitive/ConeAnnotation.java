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
import java.util.HashMap;
import java.util.HashSet;
import java.util.Iterator;
import java.util.LinkedHashMap;

import javax.vecmath.Point3f;
import javax.vecmath.Tuple3f;
import javax.vecmath.Vector3f;

import processing.core.PGraphics;
import edu.tum.cs.vis.model.Model;
import edu.tum.cs.vis.model.uima.annotation.PrimitiveAnnotation;
import edu.tum.cs.vis.model.uima.cas.MeshCas;
import edu.tum.cs.vis.model.util.Curvature;
import edu.tum.cs.vis.model.util.Vertex;
import edu.tum.cs.vis.model.util.algorithm.BestFitLine3D;
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

	private final boolean				concav;

	private final Point3f				centroid		= new Point3f();

	/**
	 * Defines the generating axis of the cone. Points into the direction of radiusSmall. Length of
	 * this vector is half of the height of the cone.
	 */
	private final Vector3f				direction		= new Vector3f();

	private float						radiusLarge		= 0;

	private float						radiusSmall		= 0;

	private final ArrayList<Point3f>	intersections	= new ArrayList<Point3f>();

	public ConeAnnotation(HashMap<Vertex, Curvature> curvatures, Model model, boolean concav) {
		super(ConeAnnotation.class, curvatures, model, concav ? new Color(0, 125, 125) : new Color(
				255, 255, 0));
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

		/*for (Triangle t : mesh.getTriangles()) {
			for (Vertex v : t.getPosition()) {
				Curvature c = v.getCurvature();
				Vector3f norm = (Vector3f) v.getNormalVector().clone();
				norm.scale(-1 / c.getCurvatureMax());

				g.fill(255, 0, 0);
				// g.stroke(255, 255, 255);
				g.stroke(50, 50, 50);
				g.strokeWeight(2);

				g.line(v.x, v.y, v.z, v.x + norm.x, v.y + norm.y, v.z + norm.z);

				// g.stroke(255, 0, 0);
				// g.line(v.x, v.y, v.z, v.x + c.getPrincipleDirectionMax().x,
				// v.y + c.getPrincipleDirectionMax().y, v.z + c.getPrincipleDirectionMax().z);
			}
		}

		g.strokeWeight(5);
		g.stroke(0, 255, 0);

		for (Point3f p : intersections) {

			// Color c = Color.getHSBColor((vw.w - min) * factor, 1, 1);
			// g.stroke(c.getRed(), c.getGreen(), c.getBlue());
			// g.strokeWeight(10 * factor * (vw.w - min) + 5);
			g.point(p.x, p.y, p.z);
		}

		g.stroke(255, 255, 0);
		g.strokeWeight(20);
		g.point(centroid.x, centroid.y, centroid.z);

		g.stroke(0, 255, 255);
		g.strokeWeight(5);

		g.line(centroid.x, centroid.y, centroid.z, centroid.x + direction.x, centroid.y
				+ direction.y, centroid.z + direction.z);*/

		g.noStroke();

		g.fill(getDrawColor().getRed(), getDrawColor().getGreen(), getDrawColor().getBlue(), 150);

		g.pushMatrix();

		g.translate(centroid.x, centroid.y, centroid.z);

		float dirLen = direction.length();

		float rx = (float) Math.asin(-direction.y / dirLen);
		float ry = (float) Math.atan2(direction.x / dirLen, direction.z / dirLen);
		g.rotateY(ry);
		g.rotateX(rx);

		MeshReasoningView.drawCylinder(g, 30, radiusLarge, radiusSmall, dirLen * 2, false, false);
		g.popMatrix();

	}

	@Override
	public boolean fitAnnotation() {

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

			// Get vector normal and set its length according to the curvature radius (= 1/ curv).
			// So on a perfect cylinder the vectors end in the same point.
			Vector3f norm = (Vector3f) v.getNormalVector().clone();
			norm.scale(-1 / curvatures.get(v).getCurvatureMax());

			// Find the shortest intersection route
			float minDist = Float.MAX_VALUE;
			Point3f addCandidateA = null;
			Point3f addCandidateB = null;
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
				norm2.scale(-1 / curvatures.get(v2).getCurvatureMax());

				if (calculateShortestRoute(v, norm, v2, norm2, pa, pb)) {
					// v and v2 have an intersection route. Check if it is the shortest possible for
					// v
					Vector3f tmp = new Vector3f(pa);
					tmp.sub(pb);
					if (tmp.lengthSquared() < minDist) {
						minDist = tmp.lengthSquared();

						// Get middle point of intersection route
						pa.add(pb);
						pa.scale(1f / 2f);
						addCandidateA = new Point3f(pa);
						addCandidateB = new Point3f(pb);
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
				Vector3f norm = (Vector3f) v.getNormalVector().clone();
				norm.scale(-1 / curvatures.get(v).getCurvatureMax());
				norm.add(v);
				intersections.add(new Point3f(norm));
			}

		}

		// Now intersections are found. Find the best fitting line between through these points.
		// Should be the generating line (line in the middle of cylinder).
		BestFitLine3D.getBestFitLine(intersections, direction, centroid);
		double radiusBottom = 0;
		double radiusTop = 0;
		double radiusBottomWeight = 0;
		double radiusTopWeight = 0;
		double heightBottom = 0;
		double heightTop = 0;

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
				heightBottom += Math.abs(dot) * weight;
			} else {
				// v is on top of cone
				radiusTop += rad * weight;
				radiusTopWeight += weight;
				heightTop += Math.abs(dot) * weight;
			}
		}

		if (radiusBottomWeight == 0 || radiusTopWeight == 0) {
			// Combine this annotation into neighbor annotation because if one of the weights are
			// null, it is no cone

			return false;
		}

		heightBottom /= radiusBottomWeight;
		heightTop /= radiusTopWeight;

		// Move centroid along direction vector to set it to the center of bottom and top
		float diff = (float) (heightTop - heightBottom) / 2;
		Vector3f tmp = new Vector3f(direction);
		tmp.scale(diff);
		centroid.add(tmp);

		radiusLarge = (float) (radiusBottom / radiusBottomWeight);
		radiusSmall = (float) (radiusTop / radiusTopWeight);

		// Set direction length to the height of cone
		direction.scale((float) (heightTop + heightBottom) / 2);

		return true;
	}

	/**
	 * @return the centroid
	 */
	public Point3f getCentroid() {
		return centroid;
	}

	/**
	 * @return the centroid
	 */
	public Tuple3f getCentroidUnscaled() {
		return model.getUnscaled(centroid);
	}

	/**
	 * @return the direction
	 */
	public Vector3f getDirection() {
		return direction;
	}

	/**
	 * @return the direction
	 */
	public Vector3f getDirectionUnscaled() {
		return new Vector3f(model.getUnscaled(direction));
	}

	/* (non-Javadoc)
	 * @see edu.tum.cs.vis.model.uima.annotation.MeshAnnotation#getNeighborAnnotations(edu.tum.cs.vis.model.uima.cas.MeshCas)
	 */
	@Override
	public HashSet<ConeAnnotation> getNeighborAnnotations(MeshCas cas) {
		return getNeighborAnnotations(cas, ConeAnnotation.class);
	}

	/* (non-Javadoc)
	 * @see edu.tum.cs.vis.model.uima.annotation.PrimitiveAnnotation#getPrimitiveArea()
	 */
	@Override
	public float getPrimitiveArea() {
		float r = radiusLarge - radiusSmall;
		float h = direction.length() * 2;
		return (float) (Math.PI * (radiusLarge + radiusSmall) * Math.sqrt(r * r + h * h));
	}

	public float getPrimitiveAreaUnscaled() {

		return model.getUnscaled(getPrimitiveArea());
	}

	/**
	 * @return the radiusLarge
	 */
	public float getRadiusLarge() {
		return radiusLarge;
	}

	/**
	 * @return the radiusLarge
	 */
	public float getRadiusLargeUnscaled() {
		return model.getUnscaled(radiusLarge);
	}

	/**
	 * @return the radiusSmall
	 */
	public float getRadiusSmall() {
		return radiusSmall;
	}

	/**
	 * @return the radiusSmall
	 */
	public float getRadiusSmallUnscaled() {
		return model.getUnscaled(radiusSmall);
	}

	public float getVolume() {

		float h = direction.length() * 2;
		return (float) ((h * Math.PI) / 3f * (Math.pow(radiusLarge, 2) + radiusLarge * radiusSmall + Math
				.pow(radiusSmall, 2)));
	}

	public float getVolumeUnscaled() {

		return model.getUnscaled(getVolume());
	}

	/**
	 * @return the concav
	 */
	public boolean isConcav() {
		return concav;
	}

}
