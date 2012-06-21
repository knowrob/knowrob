/*******************************************************************************
 * Copyright (c) 2012 Stefan Profanter. All rights reserved. This program and the accompanying
 * materials are made available under the terms of the GNU Public License v3.0 which accompanies
 * this distribution, and is available at http://www.gnu.org/licenses/gpl.html
 * 
 * Contributors: Stefan Profanter - initial API and implementation, Year: 2012
 ******************************************************************************/
package edu.tum.cs.vis.model.util;

public class TriangleNeighbor {

	/**
	 * Calculates the dihedral angle between p1 and p2. The dihedral Angle is the angle between the
	 * two surface normales.
	 * 
	 * @param p1
	 *            polygon 1
	 * @param p2
	 *            polygon 2
	 * @return the dihedral angle in radiant between 0 to PI
	 */
	private static double calcDihedralAngle(Triangle p1, Triangle p2) {
		/*
		 * Calculate angle (range: 0 - PI) between the two surface normals
		 */
		if (p1.normalVector.equals(p2.normalVector))
			return 0;

		double val = p1.normalVector.dot(p2.getNormalVector());

		// due to floating point arithmetic val may be slightly bigger than 1 or -1 where acos isn't
		// defined. So make sure val is in range:

		return Math.acos(Math.min(Math.max(val, -1.0), 1.0));
	}

	private final double	dihedralAngle;

	private final Triangle	p1;
	private final Triangle	p2;

	public TriangleNeighbor(Triangle parent, Triangle neighbor) {
		p1 = parent;
		p2 = neighbor;
		dihedralAngle = calcDihedralAngle(parent, neighbor);
	}

	@Override
	public boolean equals(Object o) {
		if (o == null)
			return false;
		if (!(o instanceof TriangleNeighbor))
			return false;
		TriangleNeighbor pairo = (TriangleNeighbor) o;
		return ((p1.equals(pairo.p1) && p2.equals(pairo.p2)) || (p1.equals(pairo.p2) && p2
				.equals(pairo.p1)));
	}

	/**
	 * @return the dihedralAngle
	 */
	public double getDihedralAngle() {
		return dihedralAngle;
	}

	public Triangle getNeighbor(Triangle p) {
		if (p.equals(p1))
			return p2;
		else if (p.equals(p2))
			return p1;
		else
			return null;
	}

	/**
	 * @return the p
	 */
	public Triangle getPolygon1() {
		return p1;
	}

	/**
	 * @return the p
	 */
	public Triangle getPolygon2() {
		return p2;
	}

	@Override
	public int hashCode() {
		return p1.hashCode() ^ p2.hashCode();
	}

}
