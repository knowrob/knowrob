/*******************************************************************************
 * Copyright (c) 2012 Stefan Profanter. All rights reserved. This program and the accompanying
 * materials are made available under the terms of the GNU Public License v3.0 which accompanies
 * this distribution, and is available at http://www.gnu.org/licenses/gpl.html
 * 
 * Contributors: Stefan Profanter - initial API and implementation, Year: 2013
 ******************************************************************************/
package edu.tum.cs.vis.model.uima.annotation.primitive;

import java.awt.Color;
import java.util.List;
import java.util.Map;
import java.util.Set;

import javax.vecmath.Matrix4f;
import javax.vecmath.Vector3f;

import processing.core.PGraphics;
import edu.tum.cs.vis.model.util.Triangle;
import edu.tum.cs.vis.model.util.Vertex;

/**
 * @author Stefan Profanter
 * 
 */
public abstract class PrimitiveShape {

	public abstract void draw(PGraphics g, Color drawColor);

	public abstract boolean fit(Vector3f centroid, Set<Vertex> vertices,
			Map<Vertex, Float> weights, List<Triangle> triangles);

	public abstract float getArea();

	public abstract Matrix4f getPoseMatrix();

	public abstract float getVolume();

}
