/*******************************************************************************
 * Copyright (c) 2012 Stefan Profanter. All rights reserved. This program and the accompanying
 * materials are made available under the terms of the GNU Public License v3.0 which accompanies
 * this distribution, and is available at http://www.gnu.org/licenses/gpl.html
 * 
 * Contributors: Stefan Profanter - initial API and implementation, Year: 2013
 ******************************************************************************/
package edu.tum.cs.vis.model.uima.annotation.primitive;

import java.awt.Color;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;

import javax.vecmath.Matrix3f;
import javax.vecmath.Matrix4f;
import javax.vecmath.Point3f;
import javax.vecmath.Tuple3f;
import javax.vecmath.Vector3f;

import org.ejml.factory.SingularMatrixException;
import org.ejml.simple.SimpleMatrix;

import processing.core.PGraphics;
import edu.tum.cs.vis.model.util.Triangle;
import edu.tum.cs.vis.model.util.Vertex;
import edu.tum.cs.vis.model.view.MeshReasoningView;

/**
 * Represents a Cone as PrimitiveShape. A cone is also a special case of a cylinder where top and
 * bottom radius are equal.
 * 
 * @author Stefan Profanter
 * 
 */
public class Cone extends PrimitiveShape {

	private static Vector3f estimageDirection(Vertex vert[], Map<Vertex, Float> weights) {

		Vector3f meanAxis = new Vector3f();

		float totalError = 0;

		// We need the extrema
		float extremaX[] = { Float.MAX_VALUE, Float.MIN_VALUE };
		float extremaY[] = { Float.MAX_VALUE, Float.MIN_VALUE };
		float extremaZ[] = { Float.MAX_VALUE, Float.MIN_VALUE };

		for (int i = 0; i < vert.length; i++) {
			extremaX[0] = Math.min(extremaX[0], vert[i].x);
			extremaX[1] = Math.max(extremaX[0], vert[i].x);
			extremaY[0] = Math.min(extremaX[0], vert[i].y);
			extremaY[1] = Math.max(extremaX[0], vert[i].y);
			extremaZ[0] = Math.min(extremaX[0], vert[i].z);
			extremaZ[1] = Math.max(extremaX[0], vert[i].z);
		}
		float maxSize = Math.max(extremaX[1] - extremaX[0],
				Math.max(extremaY[1] - extremaY[0], extremaZ[1] - extremaZ[0]));

		// Based on: http://cg.cs.uni-bonn.de/aigaion2root/attachments/schnabel-2007-efficient.pdf
		int skip = 1;
		if (vert.length > 300) {
			skip = Math.round(vert.length / 300f);
		}
		for (int i = 0; i < vert.length; i += skip) {
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
						float factor = dot / (points[v].length()/* *axis.length=1*/);
						if (factor > 1.0)
							factor = 1.0f;
						if (factor < -1.0)
							factor = -1.0f;
						openingAngle += Math.acos(factor);

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
				float goodness = 0;

				// If it is a cone, we have a valid apex. So check if all the points have approx.
				// the openingAngle.
				// If it is a cylinder, check if all the point normals are perpendicular to the
				// axis
				for (int j = 0; j < vert.length; j++) {
					if (j == i || j == idx2 || j == idx3)
						continue;
					float dot;
					float len;
					if (isCylinder) {
						if (Float.isNaN(vert[j].getNormalVector().lengthSquared()))
							// somehow there are sometimes NAN normal vectors
							continue;
						dot = vert[j].getNormalVector().dot(axis);
						len = 1;
					} else {

						Vector3f v = new Vector3f(vert[j]);
						v.sub(apex);
						if (v.lengthSquared() < 0.00001 * maxSize)
							// vector is at the same position as apex, therefore error is 0
							continue;
						dot = v.dot(axis);
						len = v.length();
					}

					float factor = dot / (len/* *axis.length=1*/);
					if (factor > 1.0)
						factor = 1.0f;
					if (factor < -1.0)
						factor = -1.0f;
					float angleToPoint = (float) Math.acos(factor);

					// Weighted sum
					goodness += (Math.PI - Math.abs(angleToPoint - openingAngle))
							* Math.pow(weights.get(vert[j]), 2);

				}

				float errorWeight = (float) (goodness * Math.pow(weights.get(selection[0])
						* weights.get(selection[1]) * weights.get(selection[2]), 2));
				Vector3f tmp = new Vector3f(axis);
				tmp.scale(errorWeight);
				meanAxis.add(tmp);
				totalError += errorWeight;
			}
		}
		if (totalError != 0) {
			meanAxis.scale(-1f / totalError);
		}
		if (meanAxis.lengthSquared() == 0)
			return null;
		meanAxis.normalize();
		return meanAxis;
	}

	private static void estimateRadius(Cone c, Vertex vert[], Map<Vertex, Float> weights,
			boolean smallRadius) {
		double heightBottom = 0;
		double heightTop = 0;
		int prevTopCount = 0;
		int topCount = 0;

		Set<Vector3f> checked = new HashSet<Vector3f>();

		for (int run = 0; run < 50; run++) {

			// Calculate centroid and radii

			// calculate the bottom and top height:
			double heightBottomWeight = 0;
			double heightTopWeight = 0;
			heightBottom = 0;
			heightTop = 0;

			double radiusBottom = 0;
			double radiusTop = 0;
			prevTopCount = topCount;
			topCount = 0;

			Vector3f directionCorrectorTop = new Vector3f();
			Vector3f directionCorrectorBottom = new Vector3f();

			checked.clear();

			if (smallRadius) {
				c.radiusSmall *= 0.60;
				c.radiusLarge *= 0.60;
			}

			for (Vertex v : vert) {
				double weight = weights.get(v);
				Vector3f check = new Vector3f(v);
				if (checked.contains(check))
					continue;
				checked.add(check);

				// Project v onto direction vector
				Vector3f tmp = new Vector3f(v);
				tmp.sub(c.centroid);
				double dot = tmp.dot(c.direction); // distance from center to v on direction vector

				// Calculate point on direction vector where v is perpendicular to direction vector
				tmp = new Vector3f(c.direction);
				tmp.scale((float) dot);
				tmp.add(c.centroid);

				// Sub v to get perpendicular distance between point on direction and v = Radius
				tmp.sub(v);
				float rad = tmp.length();
				if (dot < 0) {
					// v is on bottom of cone
					heightBottomWeight += weight;
					heightBottom += Math.abs(dot) * weight;
					radiusBottom += rad * weight;
					if (run > 0) {
						Vector3f corr = new Vector3f(tmp);
						corr.normalize();
						corr.scale((c.radiusLarge - rad) * (float) weight);
						directionCorrectorBottom.add(corr);
					}
				} else {
					// v is on top of cone
					topCount++;
					heightTopWeight += weight;
					heightTop += Math.abs(dot) * weight;
					radiusTop += rad * weight;
					if (run > 0 && tmp.lengthSquared() > 0) {
						Vector3f corr = new Vector3f(tmp);
						corr.normalize();
						if (prevTopCount == 1)
							corr.scale((-rad) * (float) weight);
						else
							corr.scale((c.radiusSmall - rad) * (float) weight);
						directionCorrectorTop.add(corr);
					}
				}

			}
			if (heightBottomWeight == 0) {
				heightBottom = 0;
			} else {
				heightBottom /= heightBottomWeight;
				c.radiusLarge = (float) (radiusBottom / heightBottomWeight);
				directionCorrectorBottom.scale(1 / (float) heightBottomWeight);
			}
			if (heightTopWeight == 0) {
				heightTop = 0;
			} else {
				heightTop /= heightTopWeight;
				c.radiusSmall = (float) (radiusTop / heightTopWeight);
				directionCorrectorTop.scale(1 / (float) heightTopWeight);
			}

			// Move centroid along direction vector to set it to the center of bottom and top
			float diff = (float) (heightBottom - heightTop) / 2;
			Vector3f tmp = new Vector3f(c.direction);
			tmp.scale(diff);
			c.centroid.sub(tmp);

			c.botCorr.set(directionCorrectorBottom);
			c.topCorr.set(directionCorrectorTop);

			if (run > 0) {
				Vector3f corrCentroid = new Vector3f(directionCorrectorBottom);
				corrCentroid.add(directionCorrectorTop);
				corrCentroid.scale(0.5f);
				c.centroid.add(corrCentroid);

				directionCorrectorTop.sub(directionCorrectorBottom);
				directionCorrectorTop.scale(0.5f);// momentum/learning rate
				c.direction.scale((float) (heightTop + heightBottom) / 2);
				c.direction.add(directionCorrectorTop);
				c.direction.normalize();
				if (directionCorrectorTop.lengthSquared() < 1.0E-8 && diff < 1.0E-8)
					break;
			}

		}
		c.direction.scale((float) (heightTop + heightBottom) / 2);
	}

	/**
	 * indicates if cone is concave or convex
	 */
	private final boolean	concave;

	/**
	 * Centroid of cone
	 */
	private final Point3f	centroid	= new Point3f();

	/**
	 * Defines the generating axis of the cone. Points into the direction of radiusSmall. Length of
	 * this vector is half of the height of the cone.
	 */
	private final Vector3f	direction	= new Vector3f();

	/**
	 * radius at bottom of cone
	 */
	private float			radiusLarge	= 0;

	/**
	 * radius at top of cone
	 */
	private float			radiusSmall	= 0;

	private final Vector3f	topCorr		= new Vector3f();

	private final Vector3f	botCorr		= new Vector3f();

	public Cone(boolean concave) {
		this.concave = concave;
	}

	/* (non-Javadoc)
	 * @see edu.tum.cs.vis.model.uima.annotation.primitive.PrimitiveShape#calculateFitError(java.util.Set, java.util.Map, java.util.List)
	 */
	@Override
	protected void calculateFitError(Set<Vertex> vertices, Map<Vertex, Float> weights,
			List<Triangle> triangles) {
		// Fitting quality is measured by the error between distance from Point to generating axis
		// and the expected radius at this point
		float vertError = 0;
		float totWeight = 0;
		Vector3f dirNorm = new Vector3f(direction);
		dirNorm.normalize();
		float halfHeight = direction.length();
		// First check all vertices
		for (Vertex v : vertices) {

			vertError += getRadiusError(v, dirNorm, halfHeight) * weights.get(v);
			totWeight += weights.get(v);
		}
		fitError = 0;
		fitError += (vertError / totWeight);
		// now also check for triangles
		float triangleError = 0;
		totWeight = 0;
		for (Triangle t : triangles) {
			triangleError += getRadiusError(t.getCentroid(), dirNorm, halfHeight) * t.getArea();
			totWeight += t.getArea();
		}
		fitError += (triangleError / totWeight);
	}

	/* (non-Javadoc)
	 * @see edu.tum.cs.vis.model.uima.annotation.primitive.PrimitiveShape#draw(processing.core.PGraphics, java.awt.Color)
	 */
	@Override
	public void draw(PGraphics g, Color drawColor) {
		// DrawDebugLines(g);

		g.noStroke();

		g.fill(drawColor.getRed(), drawColor.getGreen(), drawColor.getBlue(), 150);

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

	@SuppressWarnings("unused")
	private void DrawDebugLines(PGraphics g) {
		{

			/*for (Triangle t : mesh.getTriangles()) {
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
			}*/

			g.stroke(255, 255, 0);
			g.strokeWeight(20);
			g.point(centroid.x, centroid.y, centroid.z);

			g.stroke(0, 255, 255);
			g.strokeWeight(5);

			g.line(centroid.x, centroid.y, centroid.z, centroid.x + direction.x, centroid.y
					+ direction.y, centroid.z + direction.z);

			g.stroke(0, 255, 0);
			g.strokeWeight(5);

			g.line(centroid.x + direction.x, centroid.y + direction.y, centroid.z + direction.z,
					centroid.x + direction.x + topCorr.x, centroid.y + direction.y + topCorr.y,
					centroid.z + direction.z + topCorr.z);
			g.line(centroid.x - direction.x, centroid.y - direction.y, centroid.z - direction.z,
					centroid.x - direction.x + botCorr.x, centroid.y - direction.y + botCorr.y,
					centroid.z - direction.z + botCorr.z);
		}
	}

	/* (non-Javadoc)
	 * @see edu.tum.cs.vis.model.uima.annotation.primitive.PrimitiveShape#fit(javax.vecmath.Vector3f, java.util.Map, java.util.List)
	 */
	@Override
	public boolean fit(Vector3f centroid1, Set<Vertex> vertices, Map<Vertex, Float> weights,
			List<Triangle> triangles) {
		// we need at least 3 points
		if (vertices.size() <= 3)
			return false;

		centroid.set(centroid1);
		Vertex vert[] = vertices.toArray(new Vertex[0]);

		Vector3f axis = estimageDirection(vert, weights);
		if (axis == null)
			return false;
		direction.set(axis);

		Cone c1 = new Cone(concave);

		c1.direction.set(axis);
		estimateRadius(c1, vert, weights, false);
		c1.calculateFitError(vertices, weights, triangles);
		float error1 = c1.fitError;

		Cone c2 = new Cone(concave);

		c2.direction.set(axis);
		estimateRadius(c2, vert, weights, true);
		c2.calculateFitError(vertices, weights, triangles);
		float error2 = c2.fitError;

		if (error1 < error2)
			set(c1);
		else
			set(c2);
		return true;

	}

	/* (non-Javadoc)
	 * @see edu.tum.cs.vis.model.uima.annotation.primitive.PrimitiveShape#getArea()
	 */
	@Override
	public float getArea() {
		float r = radiusLarge - radiusSmall;
		float h = direction.length() * 2;
		return (float) (Math.PI * (radiusLarge + radiusSmall) * Math.sqrt(r * r + h * h));
	}

	/**
	 * @return the centroid
	 */
	public Point3f getCentroid() {
		return centroid;
	}

	/**
	 * @return the direction
	 */
	public Vector3f getDirection() {
		return direction;
	}

	public float getHeight() {
		return direction.length() * 2f;
	}

	/**
	 * Get pose matrix for cone.
	 * 
	 * @return 4x4 pose matrix of the plane relative to the object centroid
	 */
	@Override
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

	/**
	 * Get average radius of cone which is the average between small and large radius
	 * 
	 * @return the average radius
	 */
	public float getRadiusAvg() {
		return (radiusLarge + radiusSmall) / 2f;
	}

	private float getRadiusError(Tuple3f v, Vector3f dirNorm, float halfHeight) {
		Vector3f tmp = new Vector3f(v);
		tmp.sub(centroid);
		float dot = tmp.dot(dirNorm); // distance from center to v on
										// direction vector

		// Calculate point on direction vector where v is perpendicular to direction vector
		tmp = new Vector3f(dirNorm);
		tmp.scale(dot);
		tmp.add(centroid);

		// Sub v to get perpendicular distance between point on direction and v = Radius
		tmp.sub(v);
		// rad is the perpendicular distance from generating axis to point
		float rad = tmp.length();

		// now we need the expected radius at this position
		float expRad = (1f - dot / halfHeight) / 2 * (radiusLarge - radiusSmall) + radiusSmall;

		return Math.abs(rad - expRad);

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

	/* (non-Javadoc)
	 * @see edu.tum.cs.vis.model.uima.annotation.primitive.PrimitiveShape#getVolume()
	 */
	@Override
	public float getVolume() {
		float h = direction.length() * 2;
		return (float) ((h * Math.PI) / 3f * (Math.pow(radiusLarge, 2) + radiusLarge * radiusSmall + Math
				.pow(radiusSmall, 2)));
	}

	/**
	 * @return the concave
	 */
	public boolean isConcave() {
		return concave;
	}

	private void set(Cone c) {
		botCorr.set(c.botCorr);
		topCorr.set(c.topCorr);
		centroid.set(c.centroid);
		direction.set(c.direction);
		radiusLarge = c.radiusLarge;
		radiusSmall = c.radiusSmall;
		fitError = c.fitError;
	}

}
