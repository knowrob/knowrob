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
import java.util.LinkedHashMap;

import javax.vecmath.Matrix3f;
import javax.vecmath.Matrix4f;
import javax.vecmath.Point3f;
import javax.vecmath.Tuple3f;
import javax.vecmath.Vector3f;

import org.ejml.factory.SingularMatrixException;
import org.ejml.simple.SimpleMatrix;

import processing.core.PGraphics;
import edu.tum.cs.vis.model.Model;
import edu.tum.cs.vis.model.uima.annotation.PrimitiveAnnotation;
import edu.tum.cs.vis.model.uima.cas.MeshCas;
import edu.tum.cs.vis.model.util.Curvature;
import edu.tum.cs.vis.model.util.Triangle;
import edu.tum.cs.vis.model.util.Vertex;
import edu.tum.cs.vis.model.view.MeshReasoningView;

/**
 * Annotation for primitive type: Cone (convex and concave)
 * 
 * @author Stefan Profanter
 * 
 */
public class ConeAnnotation extends PrimitiveAnnotation<ConeAnnotation> {

	/**
	 * auto generated
	 */
	private static final long			serialVersionUID	= -7420446109108464883L;

	/**
	 * indicates if cone is concave or convex
	 */
	private final boolean				concave;

	/**
	 * Centroid of cone
	 */
	private final Point3f				centroid			= new Point3f();

	/**
	 * Defines the generating axis of the cone. Points into the direction of radiusSmall. Length of
	 * this vector is half of the height of the cone.
	 */
	private final Vector3f				direction			= new Vector3f();

	/**
	 * radius at bottom of cone
	 */
	private float						radiusLarge			= 0;

	/**
	 * radius at top of cone
	 */
	private float						radiusSmall			= 0;

	/**
	 * list of all found intersection points needed for fitting cone into mesh
	 */
	private final ArrayList<Point3f>	intersections		= new ArrayList<Point3f>();

	/**
	 * Create new cone annotation.
	 * 
	 * @param curvatures
	 *            Map of curvatures for vertices
	 * @param model
	 *            parent model
	 * @param concave
	 *            is cone concave or convex
	 * 
	 */
	public ConeAnnotation(HashMap<Vertex, Curvature> curvatures, Model model, boolean concave) {
		super(ConeAnnotation.class, curvatures, model, concave ? new Color(0, 125, 125)
				: new Color(255, 255, 0));
		this.concave = concave;
	}

	@SuppressWarnings("unused")
	private void DrawDebugLines(PGraphics g) {
		{

			for (Triangle t : mesh.getTriangles()) {
				for (Vertex v : t.getPosition()) {
					Curvature c = curvatures.get(v);
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
					+ direction.y, centroid.z + direction.z);
		}
	}

	/* (non-Javadoc)
	 * @see edu.tum.cs.vis.model.uima.annotation.PrimitiveAnnotation#drawAnnotation(processing.core.PGraphics)
	 */
	@Override
	public void drawPrimitiveAnnotation(PGraphics g) {

		// DrawDebugLines(g);

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

	/* (non-Javadoc)
	 * @see edu.tum.cs.vis.model.uima.annotation.PrimitiveAnnotation#fitAnnotation()
	 */
	@Override
	public void fitAnnotation() {

		LinkedHashMap<Vertex, Float> vertices = new LinkedHashMap<Vertex, Float>();
		getVerticesWithWeight(vertices);

		// we need at least 3 points
		if (vertices.size() < 3)
			return;

		Vertex vert[] = vertices.keySet().toArray(new Vertex[0]);

		Vector3f bestAxis = new Vector3f();

		float totalError = 0;

		// Based on: http://cg.cs.uni-bonn.de/aigaion2root/attachments/schnabel-2007-efficient.pdf
		for (int i = 0; i < vert.length; i++) {

			// Run multiple tests trying to find the best solution
			for (int test = 0; test < 10; test++) {
				boolean isCylinder = false;

				// get two other random vertices
				int idx2;
				do {
					idx2 = (int) Math.floor(Math.random() * vert.length);
				} while (idx2 == i || vert[i].equals(vert[idx2]));
				int idx3;
				do {
					idx3 = (int) Math.floor(Math.random() * vert.length);
				} while (idx3 == i || idx3 == idx2 || vert[i].equals(vert[idx3])
						|| vert[idx2].equals(vert[idx3]));

				Vertex selection[] = new Vertex[3];
				selection[0] = vert[i];
				selection[1] = vert[idx2];
				selection[2] = vert[idx3];

				// each vertex represents a plane. (vertex point is a point on the plane, vertex
				// normal
				// is the plane normal). nx*x + xy*y + xz*z = nx*px+ny*py+nz*pz.
				// See: http://mathworld.wolfram.com/Plane.html

				SimpleMatrix a = new SimpleMatrix(3, 3);
				SimpleMatrix b = new SimpleMatrix(3, 1);
				for (int r = 0; r < 3; r++) {
					a.setRow(r, 0, selection[r].getNormalVector().x,
							selection[r].getNormalVector().y, selection[r].getNormalVector().z);
					b.setRow(r, 0, selection[r].x * selection[r].getNormalVector().x
							+ selection[r].y * selection[r].getNormalVector().y + selection[r].z
							* selection[r].getNormalVector().z);
				}
				// This is the apex (tip) of the cone. It is the point where all three planes
				// intersect.
				SimpleMatrix x;
				try {
					x = a.solve(b);
				} catch (SingularMatrixException e) {
					continue;
				}
				Point3f apex = new Point3f((float) x.get(0, 0), (float) x.get(1, 0), (float) x.get(
						2, 0));

				float coords[] = new float[3];
				apex.get(coords);
				for (int j = 0; j < 3; j++) {
					if (Float.isInfinite(coords[j]) || Float.isNaN(coords[j])) {
						// there is no valid apex, this means that it is a cylinder
						isCylinder = true;
						break;
					}
				}
				// Now calculate the axis:

				Vector3f points[] = new Vector3f[3];
				Vector3f axis = new Vector3f();
				if (!isCylinder) {
					// try to find the axis of the cone
					Vector3f pointsCenter = new Vector3f();
					for (int v = 0; v < 3; v++) {
						points[v] = new Vector3f(selection[v]);
						points[v].sub(apex);
						points[v].scale(1 / points[v].length());
						points[v].add(apex);
						points[v].normalize();
						pointsCenter.add(points[v]);
					}
					pointsCenter.scale((float) (1.0 / 3.0));
					points[0].sub(pointsCenter);
					points[1].sub(pointsCenter);
					axis.cross(points[0], points[1]);
					axis.normalize();
					axis.scale(-1f);
					if (axis.lengthSquared() == 0 || Float.isInfinite(axis.lengthSquared())
							|| Float.isNaN(axis.lengthSquared())) {
						// we have a cylinder because axis is zero which means there is no real apex
						axis = points[2];
						isCylinder = true;
					}

				}
				float openingAngle = 0;

				if (!isCylinder) {
					for (int v = 0; v < 3; v++) {
						points[v] = new Vector3f(selection[v]);
						points[v].sub(apex);
						float dot = points[v].dot(axis);
						if (dot > 1.0)
							dot = 1.0f;
						if (dot < -1.0)
							dot = -1.0f;
						openingAngle += Math.acos(dot / (points[v].length()/* *axis.length=1*/));

					}
					openingAngle *= 1.0 / 3.0;
					if (Float.isNaN(openingAngle)) {
						isCylinder = true;
					} else if (openingAngle > Math.PI / 2) {
						// angle is outer angle, not inner, so convert. This is because an inverted
						// axis
						// direction
						axis.scale(-1f);
						openingAngle = (float) (Math.PI - openingAngle);
					}
				}
				if (isCylinder) {
					// Take the normal of the points cros product. This is the direction of the axis
					axis.cross(selection[0].getNormalVector(), selection[1].getNormalVector());
					axis.normalize();
					if (Float.isNaN(axis.lengthSquared())) {
						axis.cross(selection[0].getNormalVector(), selection[2].getNormalVector());
					}
					if (Float.isNaN(axis.lengthSquared())) {
						axis.cross(selection[1].getNormalVector(), selection[2].getNormalVector());
					}

					if (Float.isNaN(axis.lengthSquared())) {
						continue;
					}
					openingAngle = (float) (Math.PI / 2f);
				}

				// Ok, now we have the apex, axis and opening angle for cone OR axis for cylinder.

				// Now measure how good these values fit the whole annotation by calculating the
				// distance of each point to the cone.
				float error = 0;

				// If it is a cone, we have a valid apex. So check if all the points have approx.
				// the openingAngle.
				// If it is a cylinder, check if all the point normals are perpendicular to the
				// axis

				float boundingSphere = model.getBoundingSphere().getR();

				for (int j = 0; j < vert.length; j++) {
					if (j == i || j == idx2 || j == idx3)
						continue;
					float dot;
					float len;
					if (isCylinder) {
						dot = vert[j].getNormalVector().dot(axis);
						len = 1;
					} else {

						Vector3f v = new Vector3f(vert[j]);
						v.sub(apex);
						if (v.lengthSquared() < 0.000001 * boundingSphere)
							// vector is at the same position as apex, therefore error is 0
							continue;
						dot = v.dot(axis);
						len = v.length();
					}
					if (dot > 1.0)
						dot = 1.0f;
					if (dot < -1.0)
						dot = -1.0f;
					float angleToPoint = (float) Math.acos(dot / (len/* *axis.length=1*/));

					// Weighted error sum
					error += (Math.abs(angleToPoint - openingAngle)) * vertices.get(vert[j]);

				}
				float errorWeight = (float) (Math.PI - error);
				Vector3f tmp = new Vector3f(axis);
				tmp.scale(errorWeight);
				bestAxis.add(tmp);
				totalError += errorWeight;
			}
		}

		bestAxis.scale(-1f / totalError);
		bestAxis.normalize();

		direction.set(bestAxis);

		// calculate a temporary centroid
		centroid.x = 0;
		centroid.y = 0;
		centroid.z = 0;

		for (int i = 0; i < vert.length; i++) {
			centroid.add(vert[i]);
		}
		centroid.scale(1f / vert.length);

		double heightBottom = 0;
		double heightTop = 0;

		for (int run = 0; run < 10; run++) {

			// Calculate centroid and radii

			// calculate the bottom and top height:
			double heightBottomWeight = 0;
			double heightTopWeight = 0;
			heightBottom = 0;
			heightTop = 0;

			double radiusBottom = 0;
			double radiusTop = 0;

			Vector3f directionCorrectorTop = new Vector3f();
			Vector3f directionCorrectorBottom = new Vector3f();

			for (Vertex v : vert) {
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
					heightBottomWeight += weight;
					heightBottom += Math.abs(dot) * weight;
					radiusBottom += rad * weight;
					if (run > 0) {
						Vector3f corr = new Vector3f(v.getNormalVector());
						if (isConcave())
							corr.scale(-1f);
						corr.scale((radiusSmall - rad) * (float) weight);
						directionCorrectorBottom.add(corr);
					}
				} else {
					// v is on top of cone
					heightTopWeight += weight;
					heightTop += Math.abs(dot) * weight;
					radiusTop += rad * weight;
					if (run > 0) {
						Vector3f corr = new Vector3f(v.getNormalVector());
						if (!isConcave())
							corr.scale(-1f);
						corr.scale((radiusLarge - rad) * (float) weight);
						directionCorrectorTop.add(corr);
					}
				}

			}
			if (heightBottomWeight == 0) {
				heightBottom = 0;
			} else {
				heightBottom /= heightBottomWeight;
				radiusLarge = (float) (radiusBottom / heightBottomWeight);
				directionCorrectorBottom.scale(1 / (float) heightBottomWeight);
			}
			if (heightTopWeight == 0) {
				heightTop = 0;
			} else {
				heightTop /= heightTopWeight;
				radiusSmall = (float) (radiusTop / heightTopWeight);
				directionCorrectorTop.scale(1 / (float) heightTopWeight);
			}

			// Move centroid along direction vector to set it to the center of bottom and top
			float diff = (float) (heightBottom - heightTop) / 2;
			Vector3f tmp = new Vector3f(direction);
			tmp.scale(diff);
			centroid.sub(tmp);

			if (run > 0) {
				centroid.sub(directionCorrectorBottom);
				centroid.add(directionCorrectorTop);
				float botLen = directionCorrectorBottom.length();
				float topLen = directionCorrectorTop.length();
				float newLen = (botLen + topLen) / 2f;
				if (botLen != 0)
					directionCorrectorBottom.scale(newLen / botLen);
				if (topLen != 0)
					directionCorrectorTop.scale(newLen / topLen);
				direction.scale((float) (heightTop + heightBottom) / 2);
				direction.add(directionCorrectorBottom);
				direction.normalize();
				if (newLen == 0 && diff == 0) {
					break;
				}
			}
		}

		direction.scale((float) (heightTop + heightBottom) / 2);
		return;
	}

	/**
	 * Get centroid of cone
	 * 
	 * @return the centroid
	 */
	public Point3f getCentroid() {
		return centroid;
	}

	/**
	 * get centroid of cone at unscaled position
	 * 
	 * @return the centroid
	 */
	public Tuple3f getCentroidUnscaled() {
		return model.getUnscaled(centroid);
	}

	/**
	 * get direction of cone. Direction is aligned with generating line and shows from centroid to
	 * small radius cap. Length of direction is half height of the cone (center to one end).
	 * 
	 * @return the direction
	 */
	public Vector3f getDirection() {
		return direction;
	}

	/**
	 * 
	 * get direction (unscaled) of cone. Direction is aligned with generating line and shows from
	 * centroid to small radius cap. Length of direction is half height of the cone (center to one
	 * end).
	 * 
	 * @return the direction
	 */
	public Vector3f getDirectionUnscaled() {
		return new Vector3f(model.getUnscaled(direction));
	}

	/**
	 * Get total unscaled height of cone from bottom cap to top cap which is 2*directionUnscaled.
	 * 
	 * @return unscaled height
	 */
	public float getHeightUnscaled() {
		return getDirectionUnscaled().length() * 2;
	}

	/* (non-Javadoc)
	 * @see edu.tum.cs.vis.model.uima.annotation.MeshAnnotation#getNeighborAnnotations(edu.tum.cs.vis.model.uima.cas.MeshCas)
	 */
	@Override
	public HashSet<ConeAnnotation> getNeighborAnnotations(MeshCas cas) {
		return getNeighborAnnotations(cas, ConeAnnotation.class);
	}

	/**
	 * Get pose matrix for cone.
	 * 
	 * @return 4x4 pose matrix of the plane relative to the object centroid
	 */
	public Matrix4f getPoseMatrix() {

		Matrix3f or = new Matrix3f();

		Vector3f x1 = new Vector3f();
		x1.normalize(direction);

		Vector3f z1 = new Vector3f(direction.y, direction.x, direction.z);
		z1.normalize();

		Vector3f y1 = new Vector3f();
		y1.cross(z1, x1);

		or.setColumn(0, x1);
		or.setColumn(1, y1);
		or.setColumn(2, z1);
		or.normalize();

		Matrix4f res = new Matrix4f(or, new Vector3f(centroid), 1.0f);
		return res;
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

	/* (non-Javadoc)
	 * @see edu.tum.cs.vis.model.uima.annotation.PrimitiveAnnotation#getPrimitiveAreaUnscaled()
	 */
	@Override
	public float getPrimitiveAreaUnscaled() {

		return model.getUnscaled(getPrimitiveArea());
	}

	/**
	 * Get average radius of cone which is the average between small and large radius
	 * 
	 * @return the average radius
	 */
	public float getRadiusAvg() {
		return (radiusLarge + radiusSmall) / 2f;
	}

	/**
	 * Get average radius (unscaled) of cone which is the average between small and large radius
	 * 
	 * @return average radius unscaled
	 */
	public float getRadiusAvgUnscaled() {
		return model.getUnscaled(getRadiusAvg());
	}

	/**
	 * Get large radius, which is at the bottom of cone.
	 * 
	 * @return the radiusLarge
	 */
	public float getRadiusLarge() {
		return radiusLarge;
	}

	/**
	 * Get large radius (unscaled), which is at the bottom of cone.
	 * 
	 * @return the radiusLarge
	 */
	public float getRadiusLargeUnscaled() {
		return model.getUnscaled(radiusLarge);
	}

	/**
	 * Get small radius, which is at the bottom of cone.
	 * 
	 * @return the radiusSmall
	 */
	public float getRadiusSmall() {
		return radiusSmall;
	}

	/**
	 * Get small radius (unscaled), which is at the bottom of cone.
	 * 
	 * @return the radiusSmall
	 */
	public float getRadiusSmallUnscaled() {
		return model.getUnscaled(radiusSmall);
	}

	/**
	 * Get volume of cone.
	 * 
	 * @return the volume
	 */
	public float getVolume() {

		float h = direction.length() * 2;
		return (float) ((h * Math.PI) / 3f * (Math.pow(radiusLarge, 2) + radiusLarge * radiusSmall + Math
				.pow(radiusSmall, 2)));
	}

	/**
	 * Get unscaled volume of cone.
	 * 
	 * @return the volume unscaled.
	 */
	public float getVolumeUnscaled() {

		return model.getUnscaled(getVolume());
	}

	/**
	 * Is cone concave or convex?
	 * 
	 * @return the concave
	 */
	public boolean isConcave() {
		return concave;
	}

}
